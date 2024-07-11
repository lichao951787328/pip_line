/*
 * @Author: lichao951787328 951787328@qq.com
 * @Date: 2024-06-07 11:01:20
 * @LastEditors: lichao951787328 951787328@qq.com
 * @LastEditTime: 2024-06-24 22:08:56
 * @FilePath: /pip_line/include/pip_line/pip_line.h
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
#pragma once 
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <Eigen/Core>
#include <grid_map_core/GridMap.hpp>
#include <vector>
#include <opencv2/opencv.hpp>
#include <geometry_msgs/Pose.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <geometry_msgs/PoseStamped.h>
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>
#include <AstarHierarchicalFootstepPlanner/AstarHierarchicalFootstepPlanner.h>
using namespace std;
class pip_line
{
private:
    ros::NodeHandle nh;
    Eigen::Matrix4d T_world_camera;
    ros::Subscriber pointcloud_sub;
   
    ros::Publisher raw_heightmap_pub;
    // ros::Publisher raw_heightmap_pc_pub;
    // ros::Publisher height_map_upper_pub;
    // ros::Publisher height_map_lower_pub;
    // ros::Publisher seg_result_image_pub;
    // ros::Publisher feasible_map_pub;

   
    


    // ros::Publisher planes_polygon_pub;
    // ros::Publisher planes_polygon_cutted_pub;
    // ros::Timer timer;


    
  
    grid_map::GridMap map;
 
    
    cv::Mat seg_result_image;// 这是一张彩色图
    // grid_map::GridMap feasible_map;
    
    vector<cv::Mat> plane_images;
    // visualization_msgs::MarkerArray planes_msg;
    // visualization_msgs::MarkerArray planes_cutted_msg;

    // vector<Footstep> steps;
    // vector<vector<Eigen::Vector3d>> avoid_points;

    // bool is_finish = false;

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
    pip_line(ros::NodeHandle & n);
    void pointcloud_callback(const sensor_msgs::PointCloud2::ConstPtr msg);
    pcl::PointCloud<pcl::PointXYZ> gridMap2Pointcloud(grid_map::GridMap & map);
    // void goal_point_callback(geometry_msgs::PoseStamped::ConstPtr pose_p);
    // void timerCallback(const ros::TimerEvent & event);

    // void draw_planes(grid_map::GridMap map, visualization_msgs::MarkerArray & msg, double r, double g, double b);

    // void publishSteps();
    // void publishAvoidpoints();
    // void publishVisualSteps();
    // void publishVisualAvoidpoints();

    ~pip_line();
};

