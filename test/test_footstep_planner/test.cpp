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
geometry_msgs::Pose goal;
bool get_goal = false;
void goal_point_callback(geometry_msgs::PoseStamped::ConstPtr pose_p)
{
    ROS_INFO("get goal********************************************************8");
    goal.orientation = pose_p->pose.orientation;
    goal.position    = pose_p->pose.position;
    LOG(INFO)<<goal.orientation.w<<" "<<goal.orientation.x<<" "<<goal.orientation.y<<" "<<goal.orientation.z;
    LOG(INFO)<<goal.position.x<<" "<<goal.position.y<<" "<<goal.position.z;
    get_goal = true;
}

void plaanner(grid_map::GridMap & map, grid_map::GridMap & feasible_map)
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
    vector<plane_info> planes = ps.getPlaneResult();
    cv::Mat result = ps.getSegResult();
    // cv::imshow("result", result);
    // cv::waitKey(0);
    cv::Mat seg_result_image = result;
    vector<cv::Mat> single_results = ps.getSegResultSingle();
    LOG(INFO)<<single_results.size();
    vector<cv::Mat> collision_free_images;
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
    
    if (!get_goal)
    {
        sleep(1);
    }
    

    feasible_map = map;
    map.add("label");
    cv::Mat plane_image = cv::Mat::zeros(result.size(), CV_8UC3);
    for (int i = 0; i < map.getSize().x(); i++)
    {
        for (int j = 0; j < map.getSize().y(); j++)
        {
            bool flag = false;
            for (int label = 1; label <= collision_free_images.size(); label++)
            {
                if (collision_free_images.at(label - 1).at<uchar>(i, j) == 255)
                {
                    flag = true;
                    map["label"](i ,j) = label;
                    // 后续计算
                    plane_image.at<cv::Vec3b>(i, j) = cv::Vec3b(default_colors[int(label)][0], default_colors[int(label)][1], default_colors[int(label)][2]);
                    break;
                }
            }
            if (!flag)
            {
                map["label"](i,j) = NAN;
                feasible_map["elevation"](i, j) = NAN;
            }
        }
    }

    FootParam footparam(0.15, 0.11, 0.065, 0.065);
    AstarHierarchicalFootstepPlanner planner(map, plane_image, collision_free_images, planes, footparam, 0.2);
    Eigen::Vector3d left_foot(0.05, 0.1, 0);
    Eigen::Vector3d right_foot(0.05, -0.1, 0);
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

int main(int argc, char** argv)
{
    ros::init(argc, argv, "test_planner");
    ros::NodeHandle nh;
    ros::Publisher publisher1 = nh.advertise<grid_map_msgs::GridMap>("plane_map", 1);
    ros::Publisher publisher2 = nh.advertise<grid_map_msgs::GridMap>("slope_plane_map", 1);
    ros::Publisher publisher3 = nh.advertise<grid_map_msgs::GridMap>("slope_plane_map_y", 1);
    ros::Publisher publisher4 = nh.advertise<grid_map_msgs::GridMap>("multi_step", 1);
    ros::Publisher publisher5 = nh.advertise<grid_map_msgs::GridMap>("slopexy_plane", 1);
    ros::Publisher publisher6 = nh.advertise<grid_map_msgs::GridMap>("slope_step_map", 1);

    ros::Publisher feasible_publisher1 = nh.advertise<grid_map_msgs::GridMap>("feasible_plane_map", 1);
    ros::Publisher feasible_publisher2 = nh.advertise<grid_map_msgs::GridMap>("feasible_slope_plane_map", 1);
    ros::Publisher feasible_publisher3 = nh.advertise<grid_map_msgs::GridMap>("feasible_slope_plane_map_y", 1);
    ros::Publisher feasible_publisher4 = nh.advertise<grid_map_msgs::GridMap>("feasible_slope_step_map", 1);
    ros::Publisher feasible_publisher5 = nh.advertise<grid_map_msgs::GridMap>("feasible_multi_step", 1);
    ros::Publisher feasible_publisher6 = nh.advertise<grid_map_msgs::GridMap>("feasible_slopexy_plane", 1);

    string goal_point_topic = "/move_base_simple/goal";
    ros::Subscriber goal_point_sub = nh.subscribe(goal_point_topic, 1, goal_point_callback);

    grid_map::GridMap plane_map;
    grid_map::Length length(4, 2);
    grid_map::Position position(1.5, 0);
    plane_map.setGeometry(length, 0.02, position);
    plane_map.add("elevation", 0);

    for (int i = 150; i < plane_map.getSize().x(); i++)
    {
        for (int j = 0; j < 30; j++)
        {
            plane_map["elevation"](i, j) = 0.4;
        }
    }
    for (int i = 50; i < 80; i++)
    {
        for (int j = 70; j < plane_map.getSize().y(); j++)
        {
            plane_map["elevation"](i, j) = 0.8;
        }
    }

    grid_map_msgs::GridMap plane_map_msg;
    grid_map::GridMapRosConverter::toMessage(plane_map, plane_map_msg);
    plane_map_msg.info.header.frame_id = "map";

    grid_map::GridMap feasible_plane_map;
    plaanner(plane_map, feasible_plane_map);
    grid_map_msgs::GridMap feasible_plane_map_msg;
    grid_map::GridMapRosConverter::toMessage(feasible_plane_map, feasible_plane_map_msg);
    feasible_plane_map_msg.info.header.frame_id = "map";


    grid_map::GridMap slope_plane_map;
    // grid_map::Length length(4, 2);
    // grid_map::Position position(0, 0);
    slope_plane_map.setGeometry(length, 0.02, position);
    slope_plane_map.add("elevation");
    for (int i = 0; i < slope_plane_map.getSize().x(); i++)
    {
        for (int j = 0; j < slope_plane_map.getSize().y(); j++)
        {
            if (i < slope_plane_map.getSize().x()/2)
            {
                slope_plane_map["elevation"](i, j) = 0.003 * i;
            }
            else
            {
                slope_plane_map["elevation"](i, j) = 0.003 * slope_plane_map.getSize().x()/2;
            }
        }
    }
    for (int i = 150; i < slope_plane_map.getSize().x(); i++)
    {
        for (int j = 0; j < 30; j++)
        {
            slope_plane_map["elevation"](i, j) = 1;
        }
    }
    for (int i = 50; i < 80; i++)
    {
        for (int j = 70; j < slope_plane_map.getSize().y(); j++)
        {
            slope_plane_map["elevation"](i, j) = 1.2;
        }
    }
    grid_map_msgs::GridMap slope_plane_map_msg;
    grid_map::GridMapRosConverter::toMessage(slope_plane_map, slope_plane_map_msg);
    slope_plane_map_msg.info.header.frame_id = "map";

    grid_map::GridMap feasible_slope_plane_map;
    plaanner(slope_plane_map, feasible_slope_plane_map);
    grid_map_msgs::GridMap feasible_slope_plane_map_msg;
    grid_map::GridMapRosConverter::toMessage(feasible_slope_plane_map, feasible_slope_plane_map_msg);
    feasible_slope_plane_map_msg.info.header.frame_id = "map";
    
    grid_map::GridMap slope_plane_map_y;
    // grid_map::Length length(4, 2);
    // grid_map::Position position(0, 0);
    slope_plane_map_y.setGeometry(length, 0.02, position);
    slope_plane_map_y.add("elevation");
    for (int i = 0; i < slope_plane_map_y.getSize().x(); i++)
    {
        for (int j = 0; j < slope_plane_map_y.getSize().y(); j++)
        {
            if (i < slope_plane_map_y.getSize().x()/2)
            {
                slope_plane_map_y["elevation"](i, j) = 0.002 * j;
            }
            else
            {
                slope_plane_map_y["elevation"](i, j) = 0.002 * slope_plane_map_y.getSize().y();
            }
        }
    }
    
    for (int i = 150; i < slope_plane_map_y.getSize().x(); i++)
    {
        for (int j = 0; j < 30; j++)
        {
            slope_plane_map_y["elevation"](i, j) = 1;
        }
    }
    for (int i = 50; i < 80; i++)
    {
        for (int j = 70; j < slope_plane_map_y.getSize().y(); j++)
        {
            slope_plane_map_y["elevation"](i, j) = 1.2;
        }
    }
    grid_map_msgs::GridMap slope_plane_map_y_msg;
    grid_map::GridMapRosConverter::toMessage(slope_plane_map_y, slope_plane_map_y_msg);
    slope_plane_map_y_msg.info.header.frame_id = "map";

    grid_map::GridMap feasible_slope_plane_map_y_map;
    plaanner(slope_plane_map_y, feasible_slope_plane_map_y_map);
    grid_map_msgs::GridMap feasible_slope_plane_map_y_map_msg;
    grid_map::GridMapRosConverter::toMessage(feasible_slope_plane_map_y_map, feasible_slope_plane_map_y_map_msg);
    feasible_slope_plane_map_y_map_msg.info.header.frame_id = "map";

    grid_map::GridMap slope_step_map = slope_plane_map_y;
    for (int i = 0; i < 0.75 * slope_step_map.getSize().x(); i++)
    {
        for (int j = 0; j < slope_step_map.getSize().y(); j++)
        {
            slope_step_map["elevation"](i, j) = 0.05;
        }
    }
    grid_map_msgs::GridMap slope_step_map_msg;
    grid_map::GridMapRosConverter::toMessage(slope_step_map, slope_step_map_msg);
    slope_step_map_msg.info.header.frame_id = "map";

    grid_map::GridMap feasible_slope_step_map;
    plaanner(slope_step_map, feasible_slope_step_map);
    grid_map_msgs::GridMap feasible_slope_step_map_msg;
    grid_map::GridMapRosConverter::toMessage(feasible_slope_step_map, feasible_slope_step_map_msg);
    feasible_slope_step_map_msg.info.header.frame_id = "map";

    grid_map::GridMap multi_step;
    // grid_map::Length length(4, 2);
    // grid_map::Position position(0, 0);
    multi_step.setGeometry(length, 0.02, position);
    multi_step.add("elevation");
    for (int i = 0; i < multi_step.getSize().x(); i++)
    {
        for (int j = 0; j < multi_step.getSize().y(); j++)
        {
            if (i < multi_step.getSize().x() * 0.25)
            {
                multi_step["elevation"](i, j) = 0.01;
            }
            else if (i < multi_step.getSize().x() * 0.5)
            {
                multi_step["elevation"](i, j) = 0.13;
            }
            else if (i < multi_step.getSize().x() * 0.75)
            {
                multi_step["elevation"](i, j) = 0.01;
            }
            else
            {
                multi_step["elevation"](i, j) = 0.13;
            }
        }
    }
    for (int i = 150; i < multi_step.getSize().x(); i++)
    {
        for (int j = 0; j < 30; j++)
        {
            multi_step["elevation"](i, j) = 0.5;
        }
    }
    for (int i = 50; i < 80; i++)
    {
        for (int j = 70; j < multi_step.getSize().y(); j++)
        {
            multi_step["elevation"](i, j) = 1.2;
        }
    }
    grid_map_msgs::GridMap multi_step_msg;
    grid_map::GridMapRosConverter::toMessage(multi_step, multi_step_msg);
    multi_step_msg.info.header.frame_id = "map";

    grid_map::GridMap feasible_multi_step;
    plaanner(multi_step, feasible_multi_step);
    grid_map_msgs::GridMap feasible_multi_step_msg;
    grid_map::GridMapRosConverter::toMessage(feasible_multi_step, feasible_multi_step_msg);
    feasible_multi_step_msg.info.header.frame_id = "map";
    
    grid_map::GridMap slopexy_plane;
    // grid_map::Length length(4, 2);
    // grid_map::Position position(0, 0);
    slopexy_plane.setGeometry(length, 0.02, position);
    slopexy_plane.add("elevation");
    for (int i = 0; i < slopexy_plane.getSize().x(); i++)
    {
        for (int j = 0; j < slopexy_plane.getSize().y(); j++)
        {
            if (i < slopexy_plane.getSize().x() * 0.4)
            {
                slopexy_plane["elevation"](i, j) = slopexy_plane.getSize().x() * 0.4 * 0.005 - (0.005 * i);
            }
            else if (i < slopexy_plane.getSize().x() * 0.6)
            {
                slopexy_plane["elevation"](i, j) = 0.13;
            }
            else if (i < slopexy_plane.getSize().x() * 0.75)
            {
                slopexy_plane["elevation"](i, j) = 0.002 * slopexy_plane.getSize().y() - (0.002 * j);
            }
            else
            {
                slopexy_plane["elevation"](i, j) = 0.0;
            }
        }
    }
    for (int i = 150; i < slopexy_plane.getSize().x(); i++)
    {
        for (int j = 0; j < 30; j++)
        {
            slopexy_plane["elevation"](i, j) = 0.5;
        }
    }
    for (int i = 50; i < 80; i++)
    {
        for (int j = 70; j < slopexy_plane.getSize().y(); j++)
        {
            slopexy_plane["elevation"](i, j) = 1.2;
        }
    }
    grid_map_msgs::GridMap slopexy_plane_msg;
    grid_map::GridMapRosConverter::toMessage(slopexy_plane, slopexy_plane_msg);
    slopexy_plane_msg.info.header.frame_id = "map";

    grid_map::GridMap feasible_slopexy_plane;
    plaanner(slopexy_plane, feasible_slopexy_plane);
    grid_map_msgs::GridMap feasible_slopexy_plane_msg;
    grid_map::GridMapRosConverter::toMessage(feasible_slopexy_plane, feasible_slopexy_plane_msg);
    feasible_slopexy_plane_msg.info.header.frame_id = "map";

    int index = 0;
    while (ros::ok())
    {
        int index_map = index%6;
        if (index_map == 0)
        {
            publisher1.publish(plane_map_msg);
            feasible_publisher1.publish(feasible_plane_map_msg);
        }
        else if (index_map == 1)
        {
            publisher2.publish(slope_plane_map_msg);
            feasible_publisher2.publish(feasible_slope_plane_map_msg);
        }
        else if (index_map == 2)
        {
            publisher3.publish(slope_plane_map_y_msg);
            feasible_publisher3.publish(feasible_slope_plane_map_y_map_msg);
        }
        else if (index_map == 3)
        {
            publisher4.publish(multi_step_msg);
            feasible_publisher5.publish(feasible_multi_step_msg);
        }
        else if (index_map == 4)
        {
            publisher5.publish(slopexy_plane_msg);
            feasible_publisher6.publish(feasible_slopexy_plane_msg);
            
        }
        else if (index_map == 5)
        {
            publisher6.publish(slope_step_map_msg);
            feasible_publisher4.publish(feasible_slope_step_map_msg);
        }
        index++;
        sleep(1);
    }
    

    return 0;
}