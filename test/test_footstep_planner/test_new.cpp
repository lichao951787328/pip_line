#include <grid_map_core/GridMap.hpp>
#include <grid_map_ros/GridMapRosConverter.hpp>
#include <grid_map_msgs/GridMap.h>
#include <iostream>
#include <ros/ros.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <plane_detection/plane_segmentation.h>
#include "plane_detection/load_parameter.h"
#include <AstarHierarchicalFootstepPlanner/AstarHierarchicalFootstepPlanner.h>
#include <geometry_msgs/PoseStamped.h>
#include <ros/timer.h>
#include <glog/logging.h>
class multiTest
{
private:
    ros::NodeHandle nh;
    ros::Timer timer;
    grid_map::GridMap map, feasible_map;

    geometry_msgs::Pose goal;
    bool get_goal = false;
    bool get_map = false;
    ros::Publisher map_pub;
    ros::Publisher feasible_map_pub;
    ros::Subscriber goal_sub;


    // 检测结果
    cv::Mat plane_image;
    vector<cv::Mat> collision_free_images;
    vector<plane_info> planes;
    unsigned char default_colors[10][3] =
    {
        {255, 0, 0},
        {255, 255, 0},
        {100, 20, 50},
        {0, 30, 255},
        {10, 255, 60},
        {80, 10, 100},
        {0, 255, 200},
        {10, 60, 60},
        {255, 0, 128},
        {60, 128, 128}
    };

public:
    multiTest(ros::NodeHandle & n);
    void goal_point_callback(geometry_msgs::PoseStamped::ConstPtr pose_p);
    void planner(grid_map::GridMap & map);
    void timerCallback(const ros::TimerEvent&);
    pcl::PointCloud<pcl::PointXYZ> gridMap2Pointcloud(grid_map::GridMap & map);
    ~multiTest();
};

pcl::PointCloud<pcl::PointXYZ> multiTest::gridMap2Pointcloud(grid_map::GridMap & map)
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

void multiTest::timerCallback(const ros::TimerEvent&)
{
    // cout<<"enter callback"<<endl;
    if (get_map)
    {
        // ROS_INFO("Timer triggered");
        grid_map_msgs::GridMap map_msg;
        grid_map::GridMapRosConverter::toMessage(map, map_msg);
        map_msg.info.header.frame_id = "map";
        map_pub.publish(map_msg);

        grid_map_msgs::GridMap feasible_map_msg;
        grid_map::GridMapRosConverter::toMessage(feasible_map, feasible_map_msg);
        feasible_map_msg.info.header.frame_id = "map";
        feasible_map_pub.publish(feasible_map_msg);
    }
}


multiTest::multiTest(ros::NodeHandle & n):nh(n)
{
    map_pub = nh.advertise<grid_map_msgs::GridMap>("map", 1);
    feasible_map_pub = nh.advertise<grid_map_msgs::GridMap>("feasible_map", 1);
    goal_sub = nh.subscribe("/move_base_simple/goal", 1, &multiTest::goal_point_callback, this);
    timer = nh.createTimer(ros::Duration(1.0), &multiTest::timerCallback, this);
    grid_map::Length length(4, 2);
    grid_map::Position position(1.5, 0);
    map.setGeometry(length, 0.02, position);
    
    
    // MAP1
    // map.add("elevation", 0);
    // for (int i = 150; i < map.getSize().x(); i++)
    // {
    //     for (int j = 0; j < 30; j++)
    //     {
    //         map["elevation"](i, j) = 0.4;
    //     }
    // }
    // for (int i = 50; i < 80; i++)
    // {
    //     for (int j = 70; j < map.getSize().y(); j++)
    //     {
    //         map["elevation"](i, j) = 0.8;
    //     }
    // }

    // slope map
    // map.add("elevation");
    // for (int i = 0; i < map.getSize().x(); i++)
    // {
    //     for (int j = 0; j < map.getSize().y(); j++)
    //     {
    //         if (i < map.getSize().x()/2)
    //         {
    //             map["elevation"](i, j) = 0.003 * i;
    //         }
    //         else
    //         {
    //             map["elevation"](i, j) = 0.003 * map.getSize().x()/2;
    //         }
    //     }
    // }
    // for (int i = 150; i < map.getSize().x(); i++)
    // {
    //     for (int j = 0; j < 30; j++)
    //     {
    //         map["elevation"](i, j) = 1;
    //     }
    // }
    // for (int i = 50; i < 80; i++)
    // {
    //     for (int j = 70; j < map.getSize().y(); j++)
    //     {
    //         map["elevation"](i, j) = 1.2;
    //     }
    // }

    // slope_y_plan
    // map.add("elevation");
    // for (int i = 0; i < map.getSize().x(); i++)
    // {
    //     for (int j = 0; j < map.getSize().y(); j++)
    //     {
    //         if (i < map.getSize().x()/2)
    //         {
    //             map["elevation"](i, j) = 0.002 * j;
    //         }
    //         else
    //         {
    //             map["elevation"](i, j) = 0.002 * map.getSize().y();
    //         }
    //     }
    // }
    
    // for (int i = 150; i < map.getSize().x(); i++)
    // {
    //     for (int j = 0; j < 30; j++)
    //     {
    //         map["elevation"](i, j) = 1;
    //     }
    // }
    // for (int i = 50; i < 80; i++)
    // {
    //     for (int j = 70; j < map.getSize().y(); j++)
    //     {
    //         map["elevation"](i, j) = 1.2;
    //     }
    // }

    // step plane
    // map.add("elevation");
    // for (int i = 0; i < map.getSize().x(); i++)
    // {
    //     for (int j = 0; j < map.getSize().y(); j++)
    //     {
    //         if (i < map.getSize().x()/2)
    //         {
    //             map["elevation"](i, j) = 0.002 * j;
    //         }
    //         else
    //         {
    //             map["elevation"](i, j) = 0.002 * map.getSize().y();
    //         }
    //     }
    // }
    // for (int i = 150; i < map.getSize().x(); i++)
    // {
    //     for (int j = 0; j < 30; j++)
    //     {
    //         map["elevation"](i, j) = 1;
    //     }
    // }
    // for (int i = 50; i < 80; i++)
    // {
    //     for (int j = 70; j < map.getSize().y(); j++)
    //     {
    //         map["elevation"](i, j) = 1.2;
    //     }
    // }
    // for (int i = 0; i < 0.75 * map.getSize().x(); i++)
    // {
    //     for (int j = 0; j < map.getSize().y(); j++)
    //     {
    //         map["elevation"](i, j) = 0.06;
    //     }
    // }

    // multi steps
    // map.add("elevation");
    // for (int i = 0; i < map.getSize().x(); i++)
    // {
    //     for (int j = 0; j < map.getSize().y(); j++)
    //     {
    //         if (i < map.getSize().x() * 0.25)
    //         {
    //             map["elevation"](i, j) = 0.01;
    //         }
    //         else if (i < map.getSize().x() * 0.5)
    //         {
    //             map["elevation"](i, j) = 0.13;
    //         }
    //         else if (i < map.getSize().x() * 0.75)
    //         {
    //             map["elevation"](i, j) = 0.01;
    //         }
    //         else
    //         {
    //             map["elevation"](i, j) = 0.13;
    //         }
    //     }
    // }
    // for (int i = 150; i < map.getSize().x(); i++)
    // {
    //     for (int j = 0; j < 30; j++)
    //     {
    //         map["elevation"](i, j) = 0.5;
    //     }
    // }
    // for (int i = 50; i < 80; i++)
    // {
    //     for (int j = 70; j < map.getSize().y(); j++)
    //     {
    //         map["elevation"](i, j) = 1.2;
    //     }
    // }

    // map.add("elevation");
    // for (int i = 0; i < map.getSize().x(); i++)
    // {
    //     for (int j = 0; j < map.getSize().y(); j++)
    //     {
    //         if (i < map.getSize().x() * 0.4)
    //         {
    //             map["elevation"](i, j) = map.getSize().x() * 0.4 * 0.005 - (0.005 * i);
    //         }
    //         else if (i < map.getSize().x() * 0.6)
    //         {
    //             map["elevation"](i, j) = 0.13;
    //         }
    //         else if (i < map.getSize().x() * 0.75)
    //         {
    //             map["elevation"](i, j) = 0.002 * map.getSize().y() - (0.002 * j);
    //         }
    //         else
    //         {
    //             map["elevation"](i, j) = 0.0;
    //         }
    //     }
    // }
    // for (int i = 150; i < map.getSize().x(); i++)
    // {
    //     for (int j = 0; j < 30; j++)
    //     {
    //         map["elevation"](i, j) = 0.5;
    //     }
    // }
    // for (int i = 50; i < 80; i++)
    // {
    //     for (int j = 70; j < map.getSize().y(); j++)
    //     {
    //         map["elevation"](i, j) = 1.2;
    //     }
    // }
    planner(map);
}

void multiTest::planner(grid_map::GridMap & map)
{
    pcl::PointCloud<pcl::PointXYZ> org_pc = gridMap2Pointcloud(map);
    orginazed_points raw_points;
    raw_points.initialByPCL(org_pc);
    size_t width = org_pc.width;
    size_t height = org_pc.height;
    string package_path = "/home/lichao/TCDS/src/pip_line";
    // string package_path = "/home/lichao/TCDS/src/pip_line";
    // // initial_package_path("pip_line", package_path);
    parameter param;
    load_parameter(param, package_path + "/config/parameter.xml");
    param.initial(std::max(raw_points.height, raw_points.width));
    Eigen::Matrix4f T_I = Eigen::Matrix4f::Identity();
    quatree::node::setStaticMember(width, height, param.quatree_width, T_I, raw_points, param);
    quatree::quatree qq(raw_points, param);
    plane_segmentation ps(height, width, &qq, raw_points, param);
    planes = ps.getPlaneResult();
    cv::Mat result = ps.getSegResult();
    // cv::imshow("result", result);
    // cv::waitKey(0);
    cv::Mat seg_result_image = result;
    vector<cv::Mat> single_results = ps.getSegResultSingle();

    // 如果某个平面角度过大，这个点再平面提取算法中已经考虑

    // for (auto & image : single_results)
    // {
    //     cv::imshow("image", image);
    //     cv::waitKey(0);
    // }
    
    LOG(INFO)<<single_results.size();
    plane_image = cv::Mat::zeros(result.size(), CV_8UC3);
    LOG(INFO)<<"plane_image";
    // // // 地图膨胀层
    double resolution = map.getResolution();
    double inflation_radius = 0.5;
    int inflation_pixel = 0.5/resolution;
    for (int i = 0; i < single_results.size(); i++)
    {
        cv::Mat image = single_results.at(i);
        // cv::imshow("image", image);
        // cv::waitKey(0);
        int kernel_size = inflation_pixel;
        cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(kernel_size, kernel_size));
        // 对图像进行膨胀操作
        cv::Mat dilated_image;
        cv::dilate(image, dilated_image, kernel);
        // cv::imshow("dilated_image", dilated_image);
        // cv::waitKey(0);
        // 计算膨胀后的边缘
        cv::Mat collision_layer = dilated_image - image;
        // cv::imshow("collision_layer", collision_layer);
        // cv::waitKey(0);
        cv::Mat upper_body = cv::Mat::zeros(collision_layer.size(), CV_8UC1);
        cv::Mat knee = cv::Mat::zeros(collision_layer.size(), CV_8UC1);

        std::vector<cv::Point> white_points;
        cv::findNonZero(collision_layer, white_points);
        for (auto & cv_p : white_points)
        {
            grid_map::Position3 p3;
            if (map.getPosition3("elevation", grid_map::Index(cv_p.y, cv_p.x), p3))
            {
                // cout<<"  mm: "<cv_p<<endl;
                if (!std::isnan(p3.z()))
                {

                    Eigen::Vector3f p3f(p3.x(), p3.y(), p3.z());
                    double dis = (p3f - planes.at(i).center).dot(planes.at(i).normal);

                    // LOG(INFO)<<"p3f: "<<p3f.transpose();
                    // LOG(INFO)<<"planes.at(i).center: "<<planes.at(i).center.transpose();
                    // LOG(INFO)<<"planes.at(i).normal: "<<planes.at(i).normal.transpose();

                    if (dis > 0.7)// 上半身
                    {
                        // cout<<"dis = "<<dis<<endl;
                        upper_body.at<uchar>(cv_p.y, cv_p.x) = 255;
                    }
                    else if (dis > 0.25) // 膝盖
                    {
                        // cout<<"dis = "<<dis<<endl;
                        knee.at<uchar>(cv_p.y, cv_p.x) = 255;
                    }
                }
            }
        }
        
        // cv::imshow("knee", knee);
        // cv::waitKey(0);
        // cv::imshow("upper_body", upper_body);
        // cv::waitKey(0);
        // // 进行不同半径的膨胀
        cv::Mat upper_body_image;
        cv::Mat upper_body_dilate;
        cv::Mat knee_image;
        cv::Mat knee_image_dilate;
        double upper_inflation = 0.3;
        double knee_inflation = 0.1;
        int inflation_radius_upper = upper_inflation/resolution;
        int inflation_radius_knee = knee_inflation/resolution;
        cv::Mat kernel_upper = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(inflation_radius_upper, inflation_radius_upper));
        cv::Mat kernel_knee = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(inflation_radius_knee, inflation_radius_knee));
        // // 对图像进行膨胀操作
        cv::Mat dilated_image_upper;
        cv::dilate(upper_body, dilated_image_upper, kernel_upper);
        upper_body_image = upper_body;
        upper_body_dilate = dilated_image_upper;
        cv::Mat dilated_image_knee;
        cv::dilate(knee, dilated_image_knee, kernel_knee);
        knee_image = knee;
        knee_image_dilate = dilated_image_knee;
        // cv::imshow("dilated_image_upper", dilated_image_upper);
        // cv::waitKey(0);
        // cv::imshow("dilated_image_knee", dilated_image_knee);
        // cv::waitKey(0);
        // 计算膨胀后的边缘
        cv::Mat collision_layer1 = image - dilated_image_upper;
        cv::Mat free_collision = collision_layer1 - dilated_image_knee;
        // cv::imshow("free_collision", free_collision);
        // cv::waitKey(0);
        collision_free_images.emplace_back(free_collision);
    }
    // // 构造适合落脚点规划的地图
    // 显示可通行地图
    LOG(INFO)<<"collision_free_images";

    feasible_map = map;
    
    feasible_map.add("label");
    
    for (int i = 0; i < map.getSize().x(); i++)
    {
        for (int j = 0; j < map.getSize().y(); j++)
        {
            bool flag = false;
            for (int label = 0; label < collision_free_images.size(); label++)
            {
                if (collision_free_images.at(label).at<uchar>(i, j) == 255)
                {
                    flag = true;
                    feasible_map["label"](i ,j) = label;
                    // 后续计算
                    plane_image.at<cv::Vec3b>(i, j) = cv::Vec3b(default_colors[int(label)][0], default_colors[int(label)][1], default_colors[int(label)][2]);
                    break;
                }
            }
            if (!flag)
            {
                // map["label"](i,j) = NAN;
                feasible_map["label"](i,j) = NAN;
                feasible_map["elevation"](i, j) = NAN;
            }
        }
    }
    get_map = true;
    
}

void multiTest::goal_point_callback(geometry_msgs::PoseStamped::ConstPtr pose_p)
{
    ROS_INFO("get goal********************************************************8");
    goal.orientation = pose_p->pose.orientation;
    goal.position    = pose_p->pose.position;
    LOG(INFO)<<goal.orientation.w<<" "<<goal.orientation.x<<" "<<goal.orientation.y<<" "<<goal.orientation.z;
    LOG(INFO)<<goal.position.x<<" "<<goal.position.y<<" "<<goal.position.z;
    get_goal = true;

    FootParam footparam(0.15, 0.11, 0.065, 0.065);
    AstarHierarchicalFootstepPlanner planner(feasible_map, plane_image, collision_free_images, planes, footparam, 0.2);
    Eigen::Vector3d left_foot(0.05, 0.1, 0.3);
    Eigen::Vector3d right_foot(0.05, -0.1, 0.3);
    Eigen::Vector3d goal_p;
    goal_p.x() = goal.position.x;
    goal_p.y() = goal.position.y;
    Eigen::Quaterniond qd(goal.orientation.w, goal.orientation.x, goal.orientation.y, goal.orientation.z);
    Eigen::Vector3d v_t = qd.toRotationMatrix() * Eigen::Vector3d::UnitX();
    double yaw = atan(v_t.y()/v_t.x());
    goal_p.z() = yaw;
    
    if (planner.initial(left_foot, right_foot, 0, goal_p))// 先迈右脚
    {
        LOG(INFO)<<"set start and goal";
        if (planner.plan())
        {
            vector<Footstep> steps = planner.getResultSteps();
            vector<vector<Eigen::Vector3d>> avoid_points = planner.computeAvoidPoints();
            // steps = planner.getResultSteps();
            // avoid_points = planner.computeAvoidPoints();
            // for (auto & step : steps)
            // {
            //     cout<<setw(8)<<"step: "<<step.x<<" "<<step.y<<" "<<step.z<<" "<<step.roll<<" "<<step.pitch<<" "<<step.yaw*57.3<<" "<<step.robot_side<<endl;
            //     cout<<setw(8)<<"points: "<<
            // }

            for (int i = 0; i < steps.size(); i++)
            {
                cout<<setw(8)<<"step "<<i<<": "<<steps.at(i).x<<" "<<steps.at(i).y<<" "<<steps.at(i).z<<" "<<steps.at(i).roll*57.3<<" "<<steps.at(i).pitch*57.3<<" "<<steps.at(i).yaw*57.3<<" "<<steps.at(i).robot_side<<endl;
                cout<<"points: "<<endl;
                for (auto & point : avoid_points.at(i))
                {
                    cout<<setw(8)<<point.transpose()<<endl;
                }
            }
            // publishSteps();
            // publishAvoidpoints();
            // publishVisualSteps();
            // publishVisualAvoidpoints();
        }
    }
    else
    {
        // LOG(INFO)<<"planning error";
        cout<<"planning error"<<endl;
    }
}

multiTest::~multiTest()
{
}


int main(int argc, char** argv)
{
    google::InitGoogleLogging(argv[0]); 
    google::InstallFailureSignalHandler();
    // google::SetCommandLineOptionWithMode("FLAGS_minloglevel", "2");
    FLAGS_minloglevel = 0;
    FLAGS_colorlogtostderr = true;
    FLAGS_log_dir = "./log"; 
    FLAGS_alsologtostderr = true;
    ros::init(argc, argv, "test_new");
    ros::NodeHandle n;
    multiTest mt(n);
    ros::AsyncSpinner spinner(3);  // Use n threads
    spinner.start();
    ros::waitForShutdown();
    return 0;
}