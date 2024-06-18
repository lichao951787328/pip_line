/*
 * @description: 
 * @param : 
 * @return: 
 */
#include <ros/ros.h>
#include <iostream>
#include <pcl/point_cloud.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud2.h>
#include "sensor_msgs/Image.h"
#include "opencv2/highgui.hpp"
#include "cv_bridge/cv_bridge.h"
#include "plane_detection/type.h"
#include <fstream>
#include <chrono>
using namespace std;

// 参数的输入可使用ros
bool first = true;
void chatterCallback(const sensor_msgs::Image::ConstPtr& msg)
{
  if (!first)
  {
    return;
  }
  
  cout<<msg->encoding<<" "<<msg->header<<" "<<msg->height<<" "<<msg->width<<endl;

  cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_16UC1);
  cv::Mat depth_image = cv_ptr->image;
  size_t height = depth_image.rows;
  size_t width = depth_image.rows;
  cv::Mat clone_image = depth_image(cv::Rect(0, 600, 80, 80)).clone();
  orginazed_points pc;
  pc.initial(depth_image);
  pcl::PointCloud<pcl::PointXYZ> point_cloud;
  point_cloud.width = depth_image.cols;
  point_cloud.height = depth_image.rows;
  for (auto & iter_v_points : pc.points)
  {
    for (auto & iter_point : iter_v_points)
    {
      point_cloud.points.emplace_back(pcl::PointXYZ(iter_point.x(), iter_point.y(), iter_point.z()));
    }
  }
  std::cout<<"width: "<<point_cloud.width<<" height: "<<point_cloud.height<<std::endl;
  pcl::io::savePCDFileBinary("/home/humanoid/work/plane_detection/catkin_plane_detection_test_ws/src/plane_detection/bag/cloud_out.pcd", point_cloud);
  first = false;
}


int main(int argc, char** argv)
{
  ros::init(argc, argv, "listener");
  ros::NodeHandle n;
  ros::Subscriber sub = n.subscribe("/camera/aligned_depth_to_color/image_raw", 1000, chatterCallback);
  ros::spin();
  return 0;
}