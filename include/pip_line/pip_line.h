#pragma once 
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <Eigen/Core>
#include <grid_map_core/GridMap.hpp>
#include <vector>
#include <opencv2/opencv.hpp>
#include <geometry_msgs/Pose.h>
using namespace std;
class pip_line
{
private:
    ros::NodeHandle nh;
    Eigen::Matrix4d T_world_camera;
    ros::Subscriber pointcloud_sub;
    ros::Subscriber goal_point_sub;
    ros::Publisher height_map_upper;
    ros::Publisher height_map_lower;
    ros::Publisher planes_all;
    ros::Publisher planes_cutted;
    
    bool get_goal = false;
    std::mutex m_goal;
    geometry_msgs::Pose goal;
    grid_map::GridMap map;
    vector<cv::Mat> plane_images;
public:
    pip_line(ros::NodeHandle & n);
    void pointcloud_callback(const sensor_msgs::PointCloud2::ConstPtr msg);
    ~pip_line();
};


