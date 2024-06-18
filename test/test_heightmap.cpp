/*
 * @description: 
 * @param : 
 * @return: 
 */
/*
 * @description: 
 * @param : 
 * @return: 
 */
/*
 * @description: 
 * @param : 
 * @return: 
 */
/*
 * @description: 
 * @param : 
 * @return: 
 */
#include <iostream>
#include <plane_detection/heightmap.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/voxel_grid.h>
#include <ros/ros.h>
#include <visualization_msgs/MarkerArray.h>
using namespace std;

Eigen::Matrix4f getT(const float &px, const float &py, const float &pz, const float &rx, const float &ry, const float &rz)
{
  using namespace Eigen;
  Matrix4f res;
  res.setIdentity();
  res.block<3,3>(0,0) = (AngleAxisf(rz, Vector3f::UnitZ())*AngleAxisf(ry, Vector3f::UnitY())*AngleAxisf(rx, Vector3f::UnitX())).matrix();
  res(0,3) = px;
  res(1,3) = py;
  res(2,3) = pz;
  return res;
}

double _deg2rad(double degree)
{
  double rad = degree/57.3;
  return rad;
}

Eigen::Matrix4f initialMatrix()
{
  float BaseVisionZ = +1.1258e+00-+7.1365e-01+0.18938;
  BaseVisionZ -= (77.51 - 12.81)/1000;
  float BaseVisionX = 0.15995;
  BaseVisionX -= (82.4 - 65.17)/1000;
  float BaseVisionY = 0.0;
  float BaseVisionPitchDeg = 27.5;
  Eigen::Matrix4f Base_T_Vision, Vision_T_Tar, World_T_Base, World_T_Tar;
  Base_T_Vision = getT(BaseVisionX, BaseVisionY, BaseVisionZ, 0, 0, 0);
  Eigen::Matrix3f Base_R_VisionTemp;
  Base_R_VisionTemp << 0,-1,0, -1,0,0, 0,0,-1;
  Base_T_Vision.block<3,3>(0,0) = Base_R_VisionTemp*(Eigen::AngleAxisf(_deg2rad(BaseVisionPitchDeg),Eigen::Vector3f::UnitX())).matrix();
  World_T_Base = getT(-0.015, 0, 0.65, 0.0, 0.0, 0.0);
  Vision_T_Tar.setIdentity();
  World_T_Tar = World_T_Base*Base_T_Vision*Vision_T_Tar;
  return World_T_Tar;
}

int main(int argc, char** argv)
{

  ros::init(argc, argv, "height_map"); // 初始化节点名
  ros::NodeHandle n; // 
  ros::Publisher height_map_pub = n.advertise<visualization_msgs::MarkerArray>("/height_map", 1);
  visualization_msgs::MarkerArray height_map_markerarray;

  pcl::PointCloud<pcl::PointXYZ> cloud;
  pcl::io::loadPCDFile("/home/humanoid/work/plane_detection/catkin_plane_detection_test_ws/src/plane_detection/bag/walk_step/discard-rows 100cols 1000align2.pcd", cloud);
  Eigen::Matrix4f m;
  m = initialMatrix();
  heightmap::heightmap HM(0.01, 0.5, 0.5, m);
  // pcl::VoxelGrid<pcl::PointCloud<pcl::PointXYZ>> sor;
  // sor.setInputCloud (cloud.makeShared());
  // sor.setLeafSize (0.01f, 0.01f, 0.01f);
  // pcl::PointCloud<pcl::PointXYZ> cloud_filtered;
  // sor.filter (cloud_filtered);
  HM.initial_pcl(cloud);
  vector<vector<Eigen::Vector2f>> contours = HM.get2Dcontours();
  cout<<"contours size "<<contours.size()<<endl;
  sleep(5);
  vector< vector<float> > height_map = HM.getHeightMap();
  for (size_t i = 0; i < height_map.size(); i++)
  {
    for (size_t j = 0; j < height_map.at(i).size(); j++)
    {
      // std::cout<<height_map.at(i).at(j)<<std::endl;
      for (size_t k = 0; k < std::ceil((height_map.at(i).at(j))/0.01); k++)
      {
        visualization_msgs::Marker cubic;
        cubic.header.frame_id = "height_map";
        cubic.header.stamp = ros::Time::now();
        cubic.ns = "basic shape" + std::to_string(i) + std::to_string(j) + std::to_string(k);
        cubic.id = 0;
        cubic.type = 1;
        cubic.action = visualization_msgs::Marker::ADD;

        cubic.pose.position.x = 0.01 * i;
        cubic.pose.position.y = 0.01 * j;
        cubic.pose.position.z = 0.01 * k;
        cubic.pose.orientation.x = 0.0;
        cubic.pose.orientation.y = 0.0;
        cubic.pose.orientation.z = 0.0;
        cubic.pose.orientation.w = 1.0;
    
        // Set the scale of the marker -- 1x1x1 here means 1m on a side
        cubic.scale.x = 0.01;
        cubic.scale.y = 0.01;
        cubic.scale.z = 0.01;
    
        // Set the color -- be sure to set alpha to something non-zero!
        cubic.color.r = 0.0f;
        cubic.color.g = 1.0f;
        cubic.color.b = 0.0f;
        cubic.color.a = 1.0;

        height_map_markerarray.markers.emplace_back(cubic);
      }
      
    }
  }
  
  height_map_pub.publish(height_map_markerarray);

  return 0;
}
