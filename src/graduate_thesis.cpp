#include <grid_map_core/GridMap.hpp>
#include <grid_map_msgs/GridMap.h>
#include <grid_map_ros/grid_map_ros.hpp>
#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <visualization_msgs/MarkerArray.h>
#include <iostream>
#include <diy_msgs/robotState.h>
#include <glog/logging.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <peac/PEAC_plane_detection.hpp>
using namespace std;

pcl::PointCloud<pcl::PointXYZ> gridMap2PointcloudOrganized(grid_map::GridMap & map)
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
int main(int argc, char** argv) 
{
    ros::init(argc, argv, "graduate_thesis");
    ros::NodeHandle nh;
    ros::Publisher pub = nh.advertise<grid_map_msgs::GridMap>("/localmap", 1);
    ros::Publisher state_foot_pub = nh.advertise<visualization_msgs::MarkerArray>("/state_foot", 1);
    ros::Publisher steps_pub = nh.advertise<visualization_msgs::MarkerArray>("/steps", 1);
    vector<grid_map::GridMap> maps;
    vector<grid_map_msgs::GridMap> msgs;
    vector<diy_msgs::robotState> states;

    Eigen::Vector3d left_color(1,1,0);
    Eigen::Vector3d right_color(0,1,1);
    vector<Eigen::Vector3d> steps;
    steps.emplace_back(Eigen::Vector3d(0.17, -0.1 -0.02, 0.02));
    steps.emplace_back(Eigen::Vector3d(0.36, 0.1 -0.005, 0.022));
    steps.emplace_back(Eigen::Vector3d(0.36, -0.1+ 0.01, 0.018));
    steps.emplace_back(Eigen::Vector3d(0.68, 0.1 - 0.01, 0.09));
    steps.emplace_back(Eigen::Vector3d(0.68, -0.1 - 0.02, 0.1));
    steps.emplace_back(Eigen::Vector3d(0.89, 0.1+ 0.008, 0.1));
    steps.emplace_back(Eigen::Vector3d(1.15, -0.1 -0.01, 0.11));
    steps.emplace_back(Eigen::Vector3d(1.15, 0.1 + 0.02, 0.09));
    steps.emplace_back(Eigen::Vector3d(1.48, -0.1 - 0.012, 0.18));
    steps.emplace_back(Eigen::Vector3d(1.46, 0.1 + 0.012, 0.18));
    steps.emplace_back(Eigen::Vector3d(1.485, -0.1, 0.16));
    steps.emplace_back(Eigen::Vector3d(1.485, 0.1, 0.16));
    steps.emplace_back(Eigen::Vector3d(1.775, -0.1, 0.24));
    steps.emplace_back(Eigen::Vector3d(1.775, 0.1, 0.24));

    rosbag::Bag bag;
    bag.open("/home/lichao/TCDS/src/pip_line/data/2025-03-04-11-07-07.bag", rosbag::bagmode::Read);

    for (rosbag::MessageInstance const m : rosbag::View(bag))
    {
        if (m.getTopic() == "/localmap")
        {
            grid_map_msgs::GridMap::ConstPtr grid_map_msg = m.instantiate<grid_map_msgs::GridMap>();
            if (grid_map_msg != nullptr)
            {
                // cout << "Received a grid map message" << endl;
                grid_map::GridMap map;
                grid_map::GridMapRosConverter::fromMessage(*grid_map_msg, map);
                maps.emplace_back(map);
                msgs.emplace_back(*grid_map_msg);
            }
        }
        else if (m.getTopic() == "/robot_state")
        {
            diy_msgs::robotState::ConstPtr state_msg = m.instantiate<diy_msgs::robotState>();
            if (state_msg != nullptr)
            {
                // cout << "Received a robot state message" << endl;
                states.emplace_back(*state_msg);
            }
        }
    }
    bag.close();
    cout<<"map size: "<<maps.size()<<endl;
    cout<<"msg size: "<<msgs.size()<<endl;
    cout<<"state size: "<<states.size()<<endl;

    // 将点转为有序点云并进行平面提取

    for (size_t i = 0; i < maps.size(); i++)
    {
        plane_detection pd;
        pd.initial("/home/lichao/TCDS/src/pip_line/config/plane_fitter_pcd_thesis.ini");
        pcl::PointCloud<pcl::PointXYZ> org_pc = gridMap2PointcloudOrganized(maps.at(i));
        pd.detect(org_pc);
        cv::imwrite("/home/lichao/TCDS/src/pip_line/data/result_thesis" + std::to_string(i) + ".png", pd.result);
        cv::imshow("result", pd.result);
        cv::waitKey(0);
    }
    

    return 0;
    sleep(5);
    for (int i = 0; i < maps.size() - 1; i++)
    {

        cout << "Press Enter to continue...";
        cin.ignore();

        grid_map::GridMap map = maps.at(i);
        grid_map_msgs::GridMap msg = msgs.at(i);
        pub.publish(msg);
        diy_msgs::robotState robot_state = states.at(i);
        Eigen::Quaterniond qdl;
        qdl.x() = robot_state.left_foot.orientation.x;
        qdl.y() = robot_state.left_foot.orientation.y;
        qdl.z() = robot_state.left_foot.orientation.z;
        qdl.w() = robot_state.left_foot.orientation.w;
        Eigen::Vector3d v_x = qdl.toRotationMatrix() * Eigen::Vector3d::UnitX();
        double yaw = std::atan2(v_x.y(), v_x.x());
        Eigen::Vector3d start_left = Eigen::Vector3d(robot_state.left_foot.position.x, robot_state.left_foot.position.y, yaw);

        Eigen::Quaterniond qdr;
        qdr.x() = robot_state.right_foot.orientation.x;
        qdr.y() = robot_state.right_foot.orientation.y;
        qdr.z() = robot_state.right_foot.orientation.z;
        qdr.w() = robot_state.right_foot.orientation.w;
        v_x = qdr.toRotationMatrix() * Eigen::Vector3d::UnitX();
        yaw = std::atan2(v_x.y(), v_x.x());
        Eigen::Vector3d start_right = Eigen::Vector3d(robot_state.right_foot.position.x, robot_state.right_foot.position.y, yaw);

        if (i == 0)
        {
            start_left = Eigen::Vector3d(0, 0.1, -0.7);
            start_right = Eigen::Vector3d(0, -0.1, -0.7);
        }
        
        int support_flag;
        LOG(INFO)<<"foot_state: "<<robot_state.foot_state;
        if (robot_state.foot_state == 2)
        {
            if (i == 0)
            {
                support_flag = 2; // 双脚支撑
                LOG(INFO)<<"double support";
            }
            else
            {
                support_flag = 0;// 右脚支撑
                LOG(INFO)<<"right support";
            }
        }
        else
        {
            support_flag = 1; // 左脚支撑
            LOG(INFO)<<"left support";
        }
        visualization_msgs::MarkerArray marker_array;
        visualization_msgs::Marker marker;
        marker.header.frame_id = "localmap";
        marker.header.stamp = ros::Time::now();
        marker.ns = "state";
        marker.type = visualization_msgs::Marker::CUBE;
        marker.action = visualization_msgs::Marker::ADD;

        marker.scale.x = 0.27; // 矩形的宽度
        marker.scale.y = 0.13; // 矩形的高度
        marker.scale.z = 0.01; // 矩形的厚度

        // 设置颜色
        marker.color.r = left_color.x();
        marker.color.g = left_color.y();
        marker.color.b = left_color.z();
        marker.color.a = 0.6;
        
        marker.id = 0;
        marker.pose.position.x = robot_state.left_foot.position.x;
        marker.pose.position.y = robot_state.left_foot.position.y;
        marker.pose.position.z = robot_state.left_foot.position.z + 0.02;
        marker.pose.orientation = robot_state.left_foot.orientation;

        if (i == 0)
        {
            marker.pose.position.x = 0;
            marker.pose.position.y = 0.1;
            marker.pose.position.z = -0.68;
            marker.pose.orientation.w = 1;
            marker.pose.orientation.x = 0;
            marker.pose.orientation.y = 0;
            marker.pose.orientation.z = 0;
        }
        marker_array.markers.push_back(marker);

        marker.color.r = right_color.x();
        marker.color.g = right_color.y();
        marker.color.b = right_color.z();
        marker.color.a = 0.6;
        // Right foot marker
        marker.id = 1;
        marker.pose.position.x = robot_state.right_foot.position.x;
        marker.pose.position.y = robot_state.right_foot.position.y;
        marker.pose.position.z = robot_state.right_foot.position.z +0.02;
        marker.pose.orientation = robot_state.right_foot.orientation;
        if (i == 0)
        {
            marker.pose.position.x = 0;
            marker.pose.position.y = - 0.1;
            marker.pose.position.z = -0.68;
            marker.pose.orientation.w = 1;
            marker.pose.orientation.x = 0;
            marker.pose.orientation.y = 0;
            marker.pose.orientation.z = 0;
        }
        marker_array.markers.push_back(marker);
        state_foot_pub.publish(marker_array);
        cout<<"publish state marker"<<endl;
        visualization_msgs::MarkerArray step_markers;
        visualization_msgs::Marker step_marker;
        step_marker.header.frame_id = "localmap";
        step_marker.header.stamp = ros::Time::now();
        step_marker.ns = "step";
        step_marker.type = visualization_msgs::Marker::CUBE;
        step_marker.action = visualization_msgs::Marker::ADD;
        step_marker.scale.x = 0.27;
        step_marker.scale.y = 0.13;
        step_marker.scale.z = 0.01;
        step_marker.pose.orientation.w = 1;
        step_marker.pose.orientation.x = 0;
        step_marker.pose.orientation.y = 0;
        step_marker.pose.orientation.z = 0;
        if (i == 0)
        {
            for (int j = 0; j < 5; j++)
            {
                step_marker.id = j;
                step_marker.pose.position.x = steps.at(j).x();
                step_marker.pose.position.y = steps.at(j).y();
                step_marker.pose.position.z = steps.at(j).z() - 0.7;
                if (j%2 == 0) //  右脚
                {
                    step_marker.color.r = right_color.x();
                    step_marker.color.g = right_color.y();
                    step_marker.color.b = right_color.z();
                    step_marker.color.a = 1;
                }
                else
                {
                    step_marker.color.r = left_color.x();
                    step_marker.color.g = left_color.y();
                    step_marker.color.b = left_color.z();
                    step_marker.color.a = 1;
                }
                step_markers.markers.push_back(step_marker);
            }
            
            
        }
        else
        {
            // 每次以落脚点序列来更新需要发布的落脚点，转换成想对于机器人的脚，再转到地图坐标系
            vector<Eigen::Vector3d> steps_temp;
            Eigen::Vector3d step_support = steps.at(i - 1);
            for (int j = 0; j < 5; j++)
            {
                if (i + j < steps.size() - 1)
                {
                    steps_temp.emplace_back(steps.at(i + j) - step_support);
                }
            }
            cout<<"steps_temp size: "<<steps_temp.size()<<endl;
            // 根据脚想对于base的坐标值，将其转换为机器人base坐标系下的坐标值
            Eigen::Matrix4d T_df = Eigen::Matrix4d::Identity();
            
            Eigen::Quaterniond qdf;
            if (i%2 == 0)
            {
                qdf.x() = robot_state.left_foot.orientation.x;
                qdf.y() = robot_state.left_foot.orientation.y;
                qdf.z() = robot_state.left_foot.orientation.z;
                qdf.w() = robot_state.left_foot.orientation.w;
                qdf.w() = robot_state.left_foot.orientation.w;

                T_df(0, 3) = robot_state.left_foot.position.x;
                T_df(1, 3) = robot_state.left_foot.position.y;
                T_df(2, 3) = robot_state.left_foot.position.z;


            }
            else
            {
                qdf.x() = robot_state.right_foot.orientation.x;
                qdf.y() = robot_state.right_foot.orientation.y;
                qdf.z() = robot_state.right_foot.orientation.z;
                qdf.w() = robot_state.right_foot.orientation.w;
                T_df(0, 3) = robot_state.right_foot.position.x;
                T_df(1, 3) = robot_state.right_foot.position.y;
                T_df(2, 3) = robot_state.right_foot.position.z;
            }
            T_df.block<3, 3>(0, 0) = qdf.toRotationMatrix();

            bool start_flag = false;
            if (i%2 == 0)
            {
                start_flag = true; // 右脚是初始脚
            }
            else
            {
                start_flag = false; // 左脚是初始脚
            }
            
            for (int j = 0; j < steps_temp.size(); j++)
            {
                if (start_flag) //youjiao
                {
                    step_marker.color.r = right_color.x();
                    step_marker.color.g = right_color.y();
                    step_marker.color.b = right_color.z();
                    step_marker.color.a = 1;
                }
                else //zuo jiao
                {
                    step_marker.color.r = left_color.x();
                    step_marker.color.g = left_color.y();
                    step_marker.color.b = left_color.z();
                    step_marker.color.a = 1;
                }
                start_flag = !start_flag;
                step_marker.id = j;
                Eigen::Vector3d step = T_df.block<3, 3>(0, 0) * steps_temp.at(j) + Eigen::Vector3d(T_df(0, 3), T_df(1, 3), T_df(2, 3));
                step_marker.pose.position.x = step.x();
                step_marker.pose.position.y = step.y();
                step_marker.pose.position.z = step.z();
                step_markers.markers.push_back(step_marker);
            }
            cout<<"publish step marker"<<endl;
        }
        steps_pub.publish(step_markers);
        sleep(1);
    }
    return 0;
}