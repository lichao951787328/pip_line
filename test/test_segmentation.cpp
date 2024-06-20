/*
 * @description: 
 * @param : 
 * @return: 
 */

// #include "plane_detection/plane_segmentation.h"
#include <iostream>
#include <pcl/io/pcd_io.h>
#include "plane_detection/load_parameter.h"
// #include <plane_detection/quadtree_new.h>
#include <plane_detection/plane_segmentation.h>
#include <glog/logging.h>
#include <ctime>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/visualization/cloud_viewer.h>
// #include <plane_detection/map_for_bipedrobot.h>
#include <ros/package.h>
// orginazed_points raw_points;
// parameter param;
ros::Publisher pointcloudPub;
// double refresh_quatree = 0.0;
Eigen::Matrix4f ROBOTWORLD_T_CAMERA;

string package_path;

void initial_package_path(string package_name)
{
  package_path = ros::package::getPath(package_name);
  // 检查是否成功找到包路径
  if (package_path.empty()) {
      std::cerr << "Error: Could not find package " << package_name << std::endl;
  }
  cout<<"package path: "<<package_path<<endl;
}

// 暂时不用
// void setStaticNumberVariable(size_t width_, size_t height_, size_t quatree_width_, Eigen::Matrix4f & ROBOTWORLD_T_CAMERA)
// {
//   quatree::node::data_width = width_; 
//   quatree::node::data_height = height_; 
//   quatree::node::quatree_width = quatree_width_;
//   quatree::node::check_normal = ROBOTWORLD_T_CAMERA.block<1,3>(2,0).transpose();
// }

// 根据这个确定reachable平面
void initialMatrix()
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
  ROBOTWORLD_T_CAMERA = World_T_Tar;
}

// double test_double = 0;
void initialRawPoints(size_t start_col_, size_t start_row_, size_t & width_, size_t & height_, orginazed_points & input)
{
  pcl::PointCloud<pcl::PointXYZ> cloud;
  pcl::io::loadPCDFile(package_path + "/bag/walk_step/cuting_discard.pcd", cloud);
  width_ = cloud.width; height_ = cloud.height;
  input.initialByPCL(cloud);
  // orginazed_points raw_points_pcl;
  // raw_points_pcl.initialByPCL(cloud);
  std::cout<<input.width<<" "<<input.height<<std::endl;
  std::cout<<input.points.begin()->size()<<" "<<input.points.size()<<std::endl;
  // raw_points = raw_points_pcl.Rect(start_col_, start_row_, width_, height_);
}


void getPointCloud(orginazed_points & input, pcl::PointCloud<pcl::PointXYZ> & output)
{
  output.clear();
  for (auto & iter_row : input.points)
  {
    for (auto & point : iter_row)
    {
      output.emplace_back(pcl::PointXYZ(point.x(), point.y(), point.z()));
    }
  }
}

std::vector<float> color = {
      51, 160, 44,  //0
      166, 206, 227 ,
      178 , 223 , 138 ,//6
      31, 120 , 180 ,
      251 , 154 , 153 ,// 12
      227 , 26 , 28 ,
      253 , 191 , 111 ,// 18
      106 , 61 , 154 ,
      255 , 127 , 0 , // 24
      202 , 178 , 214 ,
      255, 0.0, 0.0, // red // 30
      0.0, 255, 0.0, // green
      0.0, 0.0, 255, // blue// 36
      255, 255, 0.0,
      255, 0.0, 255, // 42
      0.0, 255, 255,
       177.5, 255, 0.0,
      255, 177.5, 0.0,
       177.5, 0.0, 255,
      255, 0.0,  177.5,
      0.0,  177.5, 255,
      0.0, 255,  177.5,
      255,  177.5,  177.5,
       177.5, 255,  177.5,
       177.5,  177.5, 255,
       177.5,  177.5, 255,
       177.5, 255,  177.5,
       177.5,  177.5, 255};


int main(int argc, char** argv)
{
  ros::init(argc, argv, "listener");
  ros::NodeHandle n;  
  pointcloudPub = n.advertise<sensor_msgs::PointCloud2>("plane_point", 1);
  google::InitGoogleLogging(argv[0]); 
  google::InstallFailureSignalHandler();
  // google::SetCommandLineOptionWithMode("FLAGS_minloglevel", "2");
  FLAGS_minloglevel = 2;
  FLAGS_colorlogtostderr = true;
  FLAGS_log_dir = "./log"; 
  FLAGS_alsologtostderr = true;
  clock_t start_time = clock();
  initial_package_path("plane_detection");
  // 不再需要这个
  initialMatrix();
  orginazed_points raw_points;
  size_t width, height;
  initialRawPoints(0, 0, width, height, raw_points);
  // 改成直接使用PCL的吧
  std::cout<<"iniitial points cost "<<(double)(clock() - start_time) / CLOCKS_PER_SEC <<" s. "<<std::endl;

  clock_t start_time_true = clock();
  cout<<"image info: "<<width<<" "<<height<<endl;
  std::cout<<"raw points: "<<raw_points.width<<" "<<raw_points.height<<std::endl;
  parameter param;
  load_parameter(param, package_path + "/config/parameter.xml");
  std::cout<<"load parameter finish"<<std::endl;
  param.initial(raw_points.width);
  // // param.showParameter();
  quatree::node::setStaticMember(width, height, param.quatree_width, ROBOTWORLD_T_CAMERA, raw_points, param);
  // // std::cout<<"construct quatree costs "<<(double)(clock() - start_time) / CLOCKS_PER_SEC <<" s. "<<std::endl;
  quatree::quatree qq(raw_points, param);
  std::cout<<"construct quatree costs "<<(double)(clock() - start_time_true) / CLOCKS_PER_SEC <<" s. "<<std::endl;
  // // qq.pindex2d->showInfo();
  // // qq.showPatchInfo();
  // // qq.showFrame();

  plane_segmentation ps(height, width, &qq, raw_points, param);
  vector<plane_info> planes = ps.getPlaneResult();
  cv::Mat result = ps.getSegResult();
  vector<cv::Mat> single_results = ps.getSegResultSingle();

  // for (auto & image : single_results)
  // {
  //   cv::imshow("single result", image);
  //   cv::waitKey(0);
  // }
  
  // cv::imshow("seg result", result);
  // cv::waitKey(0);
  // std::cout<<planes.size()<<std::endl;
  // std::cout<<"construct quatree costs "<<(double)(clock() - start_time_true) / CLOCKS_PER_SEC <<" s. "<<std::endl;
  // clock_t end_time = clock();
  // std::cout<<"cost "<<(double)(end_time - start_time) / CLOCKS_PER_SEC<<" s."<<std::endl;
  // 下面属于平面区域提取之外的内容
  // 先找到平面中不为平面或者超越法向量的点
  // cv::Mat colorImage = result.clone();
  // colorImage.convertTo(colorImage, CV_32FC3); // 将图像转换为浮点类型

  // // 创建掩码 (标记所有为0的像素)
  // cv::Mat zeroMask = result == 0;

  // // 将掩码转换为8位单通道图像
  // zeroMask.convertTo(zeroMask, CV_8UC1, 255.0);

  // int morph_size = 5; // 调整大小以适应你的噪声程度
  // cv::Mat element = cv::getStructuringElement(cv::MORPH_RECT,
  //                     cv::Size(2 * morph_size + 1, 2 * morph_size + 1),
  //                     cv::Point(morph_size, morph_size));

  // // 腐蚀后膨胀 (开操作) 以去除小的噪声点
  // cv::morphologyEx(zeroMask, zeroMask, cv::MORPH_OPEN, element);

  // // cv::imshow("nan image", zeroMask);
  // // cv::waitKey(0);

  // // 把中心过高的平面也当作障碍点
  // for (auto & plane_info : planes)
  // {
  //   if (plane_info.center.z())// 先将其转到机器人世界坐标系，
  //   {
  //     // 将彩色图像转换为灰度图像
  //     // cv::Mat grayImage1;
  //     // cv::cvtColor(colorImage, grayImage1, cv::COLOR_BGR2GRAY);

  //     // // 创建掩码 (标记灰度图像中的非0区域)
  //     // cv::Mat mask1 = grayImage1 != 0;

  //     // // 创建另一个掩码 (标记第二张灰度图像中的白色区域)
  //     // cv::Mat mask2 = grayImage2 == 255;

  //     // // 创建一个用于合并的最终灰度图像
  //     // cv::Mat resultImage = cv::Mat::zeros(colorImage.size(), CV_8UC1);

  //     // // 合并掩码
  //     // resultImage.setTo(255, mask1 | mask2);
  //   }
  // }
  
  // 使用 findNonZero 函数找到白色像素的坐标
  // cv::Mat locations; // 存储找到的坐标
  // cv::findNonZero(grayImage == 255, locations);

  // // 输出白色像素的数量
  // std::cout << "Number of white pixels: " << locations.total() << std::endl;

  // pcl::PointCloud<pcl::PointXYZ> cloud;
  // pcl::io::loadPCDFile("/home/humanoid/work/plane_detection/catkin_plane_detection_test_ws/src/plane_detection/bag/climb_steps_bag/discard-rows 100cols 1000align2.pcd", cloud);
  // heightmap::heightmap HM(0.01, 0.2, 0.2, ROBOTWORLD_T_CAMERA);
  // HM.initial_pcl(cloud);
  // vector<vector<Eigen::Vector2f>> contours = HM.get2Dcontours();
  // getLandedArea(planes, contours, ROBOTWORLD_T_CAMERA);
  // clock_t end_time = clock();
  // std::cout<<"cost "<<(double)(end_time - start_time) / CLOCKS_PER_SEC<<" s."<<std::endl;
  
  // 测试步骤
  // 1. 显示在xy平面上的平面信息
  // 2. 显示在xy平面上的障碍的轮廓信息
  // 3. 显示切割结果
  

  // sleep(2);
  // pcl::PointCloud<pcl::PointXYZRGB> cloud;
  // size_t index = 0;
  // for (auto & iter_plane : planes)
  // {
  //   for (auto & iter_point : iter_plane.valid_points)
  //   {
  //     pcl::PointXYZRGB rgbpoint;
  //     rgbpoint.x = iter_point.x();
  //     rgbpoint.y = iter_point.y();
  //     rgbpoint.z = iter_point.z();
  //     rgbpoint.r = color.at(index%10 * 3);
  //     rgbpoint.g = color.at(index%10 * 3 + 1);
  //     rgbpoint.b = color.at(index%10 * 3 + 2);
  //     cloud.emplace_back(rgbpoint);
  //   }
  //   index ++;
  // }
  // pcl::visualization::CloudViewer viewer("cloud");
  // viewer.showCloud(cloud.makeShared());
  // while (!viewer.wasStopped())
	// {
	// }
  // google::ShutdownGoogleLogging();
  return 0;
}
