#include <pip_line/pip_line.h>
#include <grid_map_core/GridMap.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <PEAC_AHFP/plane_fitter_pcl_AHFP.hpp>
#include <opencv2/opencv.hpp>
#include <AstarHierarchicalFootstepPlanner/AstarHierarchicalFootstepPlanner.h>
#include <tf2/buffer_core.h>
#include <grid_map_msgs/GridMap.h>
#include <Eigen/Dense>
pip_line::pip_line(ros::NodeHandle & n):nh(n)
{
    pointcloud_sub = nh.subscribe("/trigger_points", 1, &pip_line::pointcloud_callback, this);
    string goal_point_topic = "/move_base_simple/goal";
    goal_point_sub = nh.subscribe(goal_point_topic, 1, &pip_line::goal_point_callback, this);
    height_map_upper = nh.advertise<grid_map_msgs::GridMap>("height_map_upper", 1);
    height_map_lower = nh.advertise<grid_map_msgs::GridMap>("height_map_lower", 1);
    grid_map::Length length(4, 2);
    grid_map::Position position(1, 0);
    map.setGeometry(length, 0.02, position);
    map.add("elevation", NAN);
    Eigen::Matrix4d T_velodyne_camera = Eigen::Matrix4d::Identity();
    Eigen::Matrix4d T_base_velodyne = Eigen::Matrix4d::Identity();
    Eigen::Matrix4d T_world_base = Eigen::Matrix4d::Identity();
    T_velodyne_camera.block<3,3>(0,0) = Eigen::Quaterniond(0.229184, 0, 0.973383, 0);
    T_velodyne_camera.block<3,1>(0,3) = Eigen::Vector3d(0.15, 0, -0.4024);
    T_base_velodyne.block<3,1>(0,3) = Eigen::Vector3d(-0.01, 0, 0.7217);
    T_world_base.block<3,1>(0,3) = Eigen::Vector3d(0, 0, 0.665);
    T_world_camera = T_world_base * T_base_velodyne * T_velodyne_camera;
}

pcl::PointCloud<pcl::PointXYZ> gridMap2Pointcloud(grid_map::GridMap & map)
{
    pcl::PointCloud<pcl::PointXYZ> pc;
    for (int i = 0; i < map.getSize().x(); i++)
    {
        for (int j = 0; j < map.getSize().y(); j++)
        {
            grid_map::Index index(i, j);
            grid_map::Position3 p3;
            if (map.getPosition3("elevation", index, p3))
            {
                if (!std::isnan(p3.z()))
                {
                    pc.emplace_back(pcl::PointXYZ(p3.x(), p3.y(), p3.z()));
                }
                else
                {
                    pc.emplace_back(pcl::PointXYZ(NAN, NAN, NAN));
                }
            }
            else
            {
                pc.emplace_back(pcl::PointXYZ(NAN, NAN, NAN));
            }
        }
    }
    pc.width = map.getSize().y();
    pc.height = map.getSize().x();
    return pc;
}

void pip_line::pointcloud_callback(const sensor_msgs::PointCloud2::ConstPtr msg)
{
    // 构建高程图
    pcl::PointCloud<pcl::PointXYZ> pc;
    pcl::fromROSMsg(*msg, pc);
    for (auto & point : pc)
    {
        Eigen::Vector3d po_w = T_world_camera.block<3,3>(0,0)* Eigen::Vector3d(point.x, point.y, point.z) + T_world_camera.block<3,1>(0,3);
        grid_map::Index index;
        if (map.getIndex(po_w.head(2), index))
        {
            map["elevation"](index.x(), index.y()) = po_w.z;
        }
    }
    // 使用地图转成有序点云
    pcl::PointCloud<pcl::PointXYZ> org_pc = gridMap2Pointcloud(map);
    // 平面检测
    AHFP::PlanarContourExtraction pce(org_pc);
    pce.run();
    cv::Mat plane_image = pce.getSegImage();
    vector<cv::Mat> planes = pce.getSegPlanes();
    vector<Eigen::Vector3d> obstacle, plane_points;
    for (int i = 0; i < plane_image.rows; i++)
    {
        for (int j = 0; j < plane_image.cols; j++)
        {
            if (plane_image.at<cv::Vec3b>(i, j) ==  cv::Vec3b(0, 0, 0))
            {
                grid_map::Position3 p3;
                if (map.getPosition3("elevation", grid_map::Index(i, j), p3))
                {
                    if (!std::isnan(p3.z()))
                    {
                        obstacle.emplace_back(p3);
                    }
                }
            }
            else
            {
                grid_map::Position3 p3;
                if (map.getPosition3("elevation", grid_map::Index(i, j), p3))
                {
                    if (!std::isnan(p3.z()))
                    {
                        plane_points.emplace_back(p3);
                    }
                }
            }
        }
    }

    for (auto & image : planes)
    {
        Eigen::Vector3d center = Eigen::Vector3d::Zero();
        vector<Eigen::Vector3d> points;
        for (int i = 0; i < image.rows; i++)
        {
            for (int j = 0; j < image.cols; j++)
            {
                if (image.at<uchar>(i, j) == 255)
                {
                    grid_map::Position3 p3;
                    if (map.getPosition3("elevation", grid_map::Index(i, j), p3))
                    {
                        points.emplace_back(p3);
                        center += p3;
                    }
                }
            }
        }
        center = center/points.size();
        if (center.z() > 0.5)
        {
            obstacle.insert(obstacle.end(), points.begin(), points.end());
        }
        
        Eigen::Matrix3d M = Eigen::Matrix3d::Zero();
        for (auto & point : points)
        {
            M += (point - center) * (point - center).transpose();
        }
        Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> eigensolver(M);
        Eigen::Vector3d minEigenvector = eigenvectors.col(0);
        if (abs(minEigenvector.z()) > 0.9)
        {
            obstacle.insert(obstacle.end(), points.begin(), points.end());
        }
    }
    
    grid_map::GridMap height_map_upper = map;
    grid_map::GridMap height_map_lower = map;
    height_map_upper.clearAll();
    height_map_lower.clearAll();
    cv::Mat height_map_upper_image = cv::Mat::zeros(height_map_upper.getLength().x(), height_map_upper.getLength().y(), CV_8UC1);
    cv::Mat height_map_lower_image = cv::Mat::zeros(height_map_lower.getLength().x(), height_map_lower.getLength().y(), CV_8UC1);
    height_map_upper.add("elevation", NAN);
    height_map_lower.add("elevation", NAN);
    grid_map::GridMap plane_map = map;
    plane_map.clearAll();
    plane_map.add("elevation", NAN);
    // 构建不可落脚区域的高程图
    for (auto & point : obstacle)
    {
        if (point.z() > 0.6)
        {
            grid_map::Index index;
            if (height_map_upper.getIndex(point.head(2), index))
            {
                height_map_upper["elevation"](index.x(), index.y()) = point.z();
                height_map_upper_image.at<uchar>(index.x(), index.y()) = 255;
            }
        }
        else
        {
            grid_map::Index index;
            if (height_map_lower.getIndex(point.head(2), index))
            {
                height_map_lower["elevation"](index.x(), index.y()) = point.z();
                height_map_lower_image.at<uchar>(index.x(), index.y()) = 255;
            }
        }
    }
    
    for (auto & point : plane_points)
    {
        grid_map::Index index;
        if (plane_map.getIndex(point.head(2), index))
        {
            plane_map["elevation"](index.x(), index.y()) = point.z();
        }
    }
    
    // 下半身图像扩展
    int lower_pixel = 0.3/height_map_lower.getResolution();
    int upper_pixel = 0.5/height_map_upper.getResolution();

    // Define the structuring element (3x3 square kernel)
    cv::Mat kernel_lower = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(lower_pixel, lower_pixel));

    // Perform erosion
    cv::Mat eroded_image_lower;
    cv::erode(height_map_lower_image, eroded_image_lower, kernel_lower);

    // Define the structuring element (3x3 square kernel)
    cv::Mat kernel_upper = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(upper_pixel, upper_pixel));

    // Perform erosion
    cv::Mat eroded_image_upper;
    cv::erode(height_map_upper_image, eroded_image_upper, kernel_upper);

    // 根据高程图对平面高程图进行切割
    for (int i = 0; i < height_map_upper_image.rows; i++)
    {
        for (int j = 0; j < height_map_upper_image.cols; j++)
        {
            if (height_map_upper_image.at<uchar>(i, j) == 255 || height_map_lower_image.at<uchar>(i, j) == 255)
            {
                plane_map["elevation"](i, j) = NAN;
            }
        }
    }
    
    // 再在切割后的高程图下进行规划

    FootParam footparam(0.13, 0.11, 0.065, 0.065);
    AstarHierarchicalFootstepPlanner planner(local_map, footparam, 0.2);
    Eigen::Vector3d left_foot(0, 0.1, 0);
    Eigen::Vector3d right_foot(0, -0.1, 0);
    while (!get_goal)
    {
        usleep(100000);
    }
    Eigen::Vector3d goal_p;
    goal_p.x() = goal.position.x;
    goal_p.y() = goal.position.y;
    Eigen::Quaterniond qd(goal.orientation.w, goal.orientation.x, goal.orientation.y, goal.orientation.z);
    Eigen::Vector3d v_t = qd.toRotationMatrix() * Eigen::Vector3d::UnitX();
    double yaw = atan(v_t.y()/v_t.x());
    goal_p.z() = yaw;
    if (planner.initial(left_foot, right_foot, 0, goal_p))
    {
        LOG(INFO)<<"set start and goal";
        if (planner.plan())
        {
            steps = planner.getResultSteps();
            avoid_points = planner.computeAvoidPoints();
            for (auto & step : steps)
            {
                cout<<setw(8)<<step.x<<" "<<step.y<<" "<<step.z<<" "<<step.roll<<" "<<step.pitch<<" "<<step.yaw*57.3<<" "<<step.robot_side<<endl;
            }
        }
    }
}

void pip_line::goal_point_callback(geometry_msgs::PoseStamped::ConstPtr pose_p)
{
    ROS_INFO("get goal");
    std::lock_guard<std::mutex> lock(m_goal);
    goal.orientation = pose_p->pose.orientation;
    goal.position    = pose_p->pose.position;
    get_goal = true;
}

pip_line::~pip_line()
{
}