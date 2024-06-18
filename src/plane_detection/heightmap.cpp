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
#include <plane_detection/heightmap.h>
#include <math.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/common/transforms.h>
#include <pcl/common/eigen.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/conditional_removal.h>
#include <plane_detection/type.h>
#include "matplotlibcpp.h"
namespace plt = matplotlibcpp;
#define DEBUG

namespace heightmap
{
  // cell_size后面用到为2cm
  heightmap::heightmap(float cell_size, float upper_body, float round_walk, Eigen::Matrix4f & ROBOTWORLD_T_CAMERA)
  {
    transform2 = Eigen::Affine3f(ROBOTWORLD_T_CAMERA);
    mcell_size = cell_size; mupper_body = upper_body; mround_walk = round_walk;
    mgrid_dimensions = std::ceil(3/mcell_size);
    if(mgrid_dimensions%2 != 0)
    {
      mgrid_dimensions ++;
    }
    upper_obstacle_grid = cv::Mat(mgrid_dimensions, mgrid_dimensions, CV_8UC1, cv::Scalar(0));
    // std::cout<<"initial upper_obstacle_grid: "<<std::endl;
    // for (size_t i = 0; i < upper_obstacle_grid.rows; i++)
    // {
    //   for (size_t j = 0; j < upper_obstacle_grid.cols; j++)
    //   {
    //     std::cout<<upper_obstacle_grid.at<uchar>(i, j)<<" ";
    //   }
    //   std::cout<<std::endl;
    // }
    // round_obstacle_grid = cv::Mat(mgrid_dimensions, mgrid_dimensions, CV_8UC1, cv::Scalar(0));
    // general_grid = cv::Mat(mgrid_dimensions, mgrid_dimensions, CV_8UC1, cv::Scalar(0));
    heightMap.resize(mgrid_dimensions);
    for (auto & row : heightMap)
    {
      row.resize(mgrid_dimensions);
    }
#ifdef DEBUG
    // for (auto & row : heightMap)
    // {
    //   for (auto & grid : row)
    //   {
    //     std::cout<<grid<<" ";
    //   }
    //   std::cout<<std::endl;
    // }
    // std::cout<<"mgrid_dimensions : "<<mgrid_dimensions<<std::endl;
#endif

    // upper_obstacle_grid.resize(mgrid_dimensions);
    // for (auto & grid_row : upper_obstacle_grid)
    // {
    //   grid_row.resize(mgrid_dimensions);
    // }
    // round_obstacle_grid.resize(mgrid_dimensions);
    // for (auto & grid_row : round_obstacle_grid)
    // {
    //   grid_row.resize(mgrid_dimensions);
    // }
  }
  // 输入在机器人世界坐标系的点集
  // 注意：机器人世界坐标系起点为mat正下方，而mat的起点为左上方的角点
  // 注意：这里其实相对机器人坐标系旋转了90度，因为机器人坐标系x是前向的，而这里高度图x是左右
  void heightmap::initial(vector<Eigen::Vector3f> & ps)
  {
    for (auto & point : ps)
    {
#ifdef DEBUG
      size_t x = (size_t)((point.x() - (mcell_size/2))/mcell_size) + mgrid_dimensions/2;
#else
      size_t x = mgrid_dimensions - (size_t)((point.x() - (mcell_size/2))  / mcell_size);// 行
#endif
      size_t y =  (size_t)((point.y() - (mcell_size/2) )/mcell_size) + mgrid_dimensions/2;//列

      heightMap.at(x).at(y) = std::max(heightMap.at(x).at(y), point.z());

      if (point.z() > mupper_body && !upper_obstacle_grid.at<uchar>(x, y))
      {
        upper_obstacle_grid.at<uchar>(x, y) = 255;
      }
      // if (point.z() > mround_walk && !round_obstacle_grid.at<uchar>(x, y))
      // {
      //   round_obstacle_grid.at<uchar>(x, y) = 255;
      //   if (point.z() > mupper_body && !upper_obstacle_grid.at<uchar>(x, y))
      //   {
      //     upper_obstacle_grid.at<uchar>(x, y) = 255;
      //   }
      // }
      // else
      // {
      //   general_grid.at<uchar>(x, y) = 255;
      // }
    }
  }

  void heightmap::initial_pcl(pcl::PointCloud<pcl::PointXYZ> & pc)
  {
    std::cout<<"raw point cloud size "<<pc.size()<<std::endl;
    pcl::VoxelGrid<pcl::PointXYZ> filter_pc;
    pcl::PointCloud<pcl::PointXYZ> filted_pc;
    pcl::PointCloud<pcl::PointXYZ> filted_pc_statistical;
    filter_pc.setInputCloud(pc.makeShared());
    filter_pc.setLeafSize(0.01f, 0.01f, 0.01f);
    filter_pc.filter(filted_pc);
    std::cout<<"filted point cloud size "<<filted_pc.size()<<std::endl;
    
    pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
    sor.setInputCloud(filted_pc.makeShared());//色湖之输入点云
    sor.setMeanK(50);//设置考虑查询点临近个数
    //sor.setMeanK(100);//设置考虑查询点临近个数
    sor.setStddevMulThresh(1.0);//设置判断是否为离群点的阈值
    sor.filter(filted_pc_statistical);
    std::cout<<"filted point cloud size "<<filted_pc_statistical.size()<<std::endl;

    
    // 转换矩阵
#ifdef DEBUG
    // pcl::visualization::PCLVisualizer viewer("viewer");
    // viewer.addCoordinateSystem();  
    pcl::PointCloud<pcl::PointXYZ> cloud_in_robotworld;
    // std::cout<<"transform :"<<std::endl; std::cout<<transform2.
    pcl::transformPointCloud(filted_pc_statistical, cloud_in_robotworld, transform2);
    // string dir = "/home/humanoid/work/plane_detection/catkin_plane_detection_test_ws/src/plane_detection/bag/";
    // pcl::io::savePCDFile(dir + "heightmap_pc" + ".pcd", cloud_in_robotworld);
    // viewer.addPointCloud(cloud_in_robotworld.makeShared());
    // viewer.spin();

    // pcl::ConditionOr<pcl::PointXYZ>::Ptr range_cond(new pcl::ConditionOr<pcl::PointXYZ>());
    // range_cond->addComparison(pcl::FieldComparison<pcl::PointXYZ>::ConstPtr(new pcl::FieldComparison<pcl::PointXYZ>("x", pcl::ComparisonOps::GT, 0.4)));
    // range_cond->addComparison(pcl::FieldComparison<pcl::PointXYZ>::ConstPtr(new pcl::FieldComparison<pcl::PointXYZ>("z", pcl::ComparisonOps::LT, 0.2)));
    // pcl::ConditionalRemoval<pcl::PointXYZ> condrem;
    // condrem.setCondition(range_cond);
    // condrem.setInputCloud(cloud_in_robotworld.makeShared());
    //condrem.setKeepOrganized(true);	//对于散乱点云，不需要执行此语句；若输入点云为有组织的点云，此语句可保持点云的原始组织结构，不会改变行列数，点数也不会减少，被过滤掉的点用 NaN 填充。
    //执行条件滤波
    pcl::PointCloud<pcl::PointXYZ> filted_pc_condition = cloud_in_robotworld;
    // condrem.filter(filted_pc_condition);
    std::cout<<"filted_pc_condition size "<<filted_pc_condition.size()<<std::endl;
    // string dir = "/home/humanoid/work/plane_detection/catkin_plane_detection_test_ws/src/plane_detection/bag/";
    // pcl::io::savePCDFile(dir + "filted_pc_condition" + ".pcd", filted_pc_condition);
    // pcl::visualization::PCLVisualizer viewer_condition("viewer condition");
    // viewer_condition.addCoordinateSystem();  
    // viewer_condition.addPointCloud(filted_pc_condition.makeShared());
    // viewer_condition.spin();

    // float x_max = 0.0;
    // float x_min = 3;
    // pcl::PointCloud<pcl::PointXYZ> upper_obstacle_cloud;
    // for (auto & point : filted_pc_condition)
    // {
    //   if (point.z >= mupper_body && point.x > 0.5)
    //   {
    //     x_max = std::max(x_max, point.x);
    //     x_min = std::min(x_min, point.x);
    //     upper_obstacle_cloud.emplace_back(point);
    //   }
    // }
    // std:cout<<"x range "<<x_min<<" "<<x_max<<std::endl;
    // pcl::visualization::PCLVisualizer viewer_upper_cloud;
    // viewer_upper_cloud.addCoordinateSystem();
    // viewer_upper_cloud.addPointCloud(upper_obstacle_cloud.makeShared());
    // viewer_upper_cloud.spin();

    // std::cout<<"upper_obstacle_cloud size "<<upper_obstacle_cloud.size()<<std::endl;
    // size_t index = 1;
    // for (auto & point : upper_obstacle_cloud)
    // {
    //   std::cout<<"index "<<index<<std::endl;
    //   index++;
    //   size_t x = mgrid_dimensions - (int)((point.x)/mcell_size);
    //   size_t y = mgrid_dimensions/2 - (int)((point.y)/mcell_size);
    //   float max_z = std::max(heightMap.at(x).at(y), point.z);
    //   heightMap.at(x).at(y) = max_z;

    //   std::cout<<"x = "<<x<<" y = "<<y<<std::endl;
    //   std::cout<<upper_obstacle_grid.size()<<endl;
    //   std::cout<<(upper_obstacle_grid.at<uchar>(x, y) == 0)<<" "<<(point.z > mupper_body)<<std::endl;
    //   if (upper_obstacle_grid.at<uchar>(x, y) == 0 && point.z > mupper_body)
    //   {
        
    //     upper_obstacle_grid.at<uchar>(x, y) = 255;
    //   }
    //   cv::imshow("upper obstacle grid", upper_obstacle_grid);
    //   cv::waitKey(0);
    // }
    

    // pcl::PointCloud<pcl::PointXYZRGB> color_pc;
    // int num_red = 0;
    // for (auto & gray_point : filted_pc_condition)
    // {
    //   if (gray_point.x >= 0)
    //   {
    //     pcl::PointXYZRGB blue_point;
    //     blue_point.r = 0;
    //     blue_point.g = 0;
    //     blue_point.b = 255;
    //     blue_point.x = gray_point.x;
    //     blue_point.y = gray_point.y;
    //     blue_point.z = gray_point.z;
    //     color_pc.emplace_back(blue_point);
    //   }
    //   else
    //   {
    //     pcl::PointXYZRGB red_point;
    //     red_point.r = 255;
    //     red_point.g = 0;
    //     red_point.b = 0;
    //     red_point.x = gray_point.x;
    //     red_point.y = gray_point.y;
    //     red_point.z = gray_point.z;
    //     color_pc.emplace_back(red_point);
    //     num_red++;
    //   }
    // }
    // std::cout<<"red points "<<num_red<<endl;
    // pcl::visualization::PCLVisualizer viewer_color("viewer color");
    // viewer_color.addCoordinateSystem();  
    // viewer_color.addPointCloud(color_pc.makeShared());
    // viewer_color.spin();

    

#endif
    // int index_round = 1;
    // int index_upper = 1;
    cout<<"initial heightmap grid:"<<endl;
    for (auto & point : filted_pc_condition)
    {
      // if (point.z < mupper_body)
      // {
      //   continue;
      // }
      
      // Eigen::Vector3f inputpoint(point.x, point.y, point.z);
      // Eigen::Vector3f outputpoint;
      // pcl::transformPoint(inputpoint, outputpoint, transform2);
      // 强制转换时是直接舍弃，不是四舍无入
      size_t x = mgrid_dimensions - (int)((point.x)/mcell_size) ;// 行
      // 换相机矩阵后需要修改
// #ifdef DEBUG
//       size_t x = (size_t)((point.x - (mcell_size/2))  / mcell_size) + mgrid_dimensions/2;
// #else
//       size_t x = mgrid_dimensions - (size_t)((point.x - (mcell_size/2))  / mcell_size);// 行
// #endif
      size_t y = mgrid_dimensions/2 - (int)((point.y)/mcell_size);//列
      // 当仅处于调试模式时，用(0.74 -point.z)
// #ifdef DEBUG
//       if ((0.74 -point.z) > mupper_body)
//       {
//         std::cout<<"info: "<<point.x<<" "<<point.y<<" "<<x<<" "<<y<<" "<<(0.74 -point.z)<<std::endl;
//       }  
// #endif
      // std::cout<<"point x "<<point.x<<std::endl;
      // std::cout<<(size_t)((point.x)  / mcell_size)<<std::endl;
      // std::cout<<"x = "<<x<<" y = "<<y<<std::endl;
      // if (x < 0 || x >= mgrid_dimensions)
      // {
      //   std::cout<<"out of range x "<<x<<std::endl;
      // }
      // if (y < 0 || y >= mgrid_dimensions)
      // {
      //   std::cout<<"out of range y "<<y<<std::endl;
      // }

      // if ((int)((point.x)/mcell_size) > mgrid_dimensions)
      // {
      //   std::cout<<"error, x = "<<point.x<<std::endl;
      // }

      // if ( std::abs((int)((point.y)/mcell_size)) > mgrid_dimensions/2)
      // {
      //   std::cout<<"error, y = "<<point.y<<std::endl;
      // }
      float z_tmp = (point.z + 0.006);
      float max_z = std::max(heightMap.at(x).at(y), z_tmp);
      heightMap.at(x).at(y) = max_z;
      // 为了与heightMap的行列统一
      if (upper_obstacle_grid.at<uchar>(x, y) == 0 && point.z > mupper_body)
      {
        upper_obstacle_grid.at<uchar>(x, y) = 255;
        // std::cout<<"upper_obstacle_grid "<<index_upper<<std::endl;
        // index_upper ++;
        // std::cout<<x<<" "<<y<<" "<<(0.74 -point.z)<<std::endl;
      }
      // if (!round_obstacle_grid.at<uchar>(x, y) && outputpoint.z() > mround_walk)
      // {
      //   round_obstacle_grid.at<uchar>(x, y) = 255;
      //   if (!upper_obstacle_grid.at<uchar>(x, y) && outputpoint.z() > mupper_body)
      //   {
      //     upper_obstacle_grid.at<uchar>(x, y) = 255;
      //     std::cout<<"upper_obstacle_grid "<<index_upper<<std::endl;
      //     index_upper ++;
      //     std::cout<<x<<" "<<y<<" "<<(0.74 -point.z)<<std::endl;
      //   }
      //   std::cout<<"round_obstacle_grid "<<index_round<<std::endl;
      //   index_round ++;
      // }
      // else
      // {
      //   general_grid.at<uchar>(x, y) = 255;
      // }
    }
#ifdef DEBUG
    // cv::imshow("upper obstacle grid", upper_obstacle_grid);
    // cv::imshow("round obstacle grid", round_obstacle_grid);
    // cv::imshow("general grid", general_grid);
    // cv::waitKey(0);
#endif
  }

  vector<vector<Eigen::Vector2f>> heightmap::get2Dcontours()
  {
    // 一个grid 1cm， 先按偏移10cm来考虑
#ifdef DEBUG
    // std::cout<<"upper_obstacle_grid: "<<std::endl;
    // for (size_t i = 0; i < upper_obstacle_grid.cols; i++)
    // {
    //   for (size_t j = 0; j < upper_obstacle_grid.rows; j++)
    //   {
    //     cv::Point P = cv::Point(i, j);
    //     if (upper_obstacle_grid.at<uchar>(P))
    //     {
    //       std::cout<<i<<" "<<j<<"; ";
    //     }
    //   }
    //   std::cout<<std::endl;
    // }
    // cv::imshow("upper obstacle grid", upper_obstacle_grid);
    // cv::waitKey(0);
#endif
// 这个地方需要根据机器人尺寸确定，初步确定是0.2m
    cv::Mat element = cv::getStructuringElement(0, cv::Size(31, 31));
    cv::Mat expand_grid;
    cv::dilate(upper_obstacle_grid, expand_grid, element);
#ifdef DEBUG
    // std::cout<<"expand_grid: "<<std::endl;
    // for (size_t i = 0; i < expand_grid.cols; i++)
    // {
    //   for (size_t j = 0; j < expand_grid.rows; j++)
    //   {
    //     cv::Point P = cv::Point(i, j);
    //     if (expand_grid.at<uchar>(P))
    //     {
    //       std::cout<<i<<" "<<j<<"; ";
    //     }
    //   }
    //   std::cout<<std::endl;
    // }
    // cv::imshow("expand grid", expand_grid);
    // cv::waitKey(0);
#endif
    std::vector<std::vector<cv::Point>> contours;
    std::vector<cv::Vec4i> hierarchy;
    cv::findContours(expand_grid, contours, hierarchy, cv::RETR_LIST, cv::CHAIN_APPROX_NONE, cv::Point());
    std::cout<<"contours size "<<contours.size()<<std::endl;
#ifdef DEBUG
    // cv::Mat imageContours = cv::Mat::zeros(expand_grid.size(), CV_8UC1);
    // cv::Mat Contours = cv::Mat::zeros(expand_grid.size(), CV_8UC1);  //绘制
    // for(int i = 0; i < contours.size(); i++)
    // {
    //   //contours[i]代表的是第i个轮廓，contours[i].size()代表的是第i个轮廓上所有的像素点数
    //   for(int j = 0; j < contours[i].size(); j++) 
    //   {
    //     //绘制出contours向量内所有的像素点
    //     cv::Point P = cv::Point(contours[i][j].x,contours[i][j].y);
    //     Contours.at<uchar>(P)=255;
    //   }
    //   //输出hierarchy向量内容
    //   char ch[256];
    //   sprintf(ch,"%d",i);
    //   string str=ch;
    //   cout<<"向量hierarchy的第" <<str<<" 个元素内容为："<<endl<<hierarchy[i]<<endl;
    //   //绘制轮廓
    //   drawContours(imageContours, contours, i, cv::Scalar(255), 1, 8, hierarchy);
    // }

    // cv::imshow("Contours Image",imageContours); //轮廓
    // cv::imshow("Point of Contours",Contours);   //向量contours内保存的所有轮廓点集
    // cv::waitKey(0);
    // std::cout<<"contours pixel index: "<<std::endl;
    // for (auto & contour : contours)
    // {
    //   for (auto & pixel_index : contour)
    //   {
    //     std::cout<<"pixel: "<<pixel_index.x<<" "<<pixel_index.y<<" ";
    //   }
    //   std::cout<<std::endl;
    // }
#endif
    vector<vector<cv::Point>> approx_contours(contours.size());
    std::cout<<"approx_contours: "<<approx_contours.size()<<std::endl;
    vector<vector<cv::Point>>::iterator approx_contour_iter = approx_contours.begin();
    for (auto & contour : contours)
    {
      cv::approxPolyDP(contour, *approx_contour_iter, 2, true);
      approx_contour_iter++;
    }
    std::cout<<"approx_contours: "<<approx_contours.size()<<std::endl;
#ifdef DEBUG
    // imageContours = cv::Mat::zeros(expand_grid.size(), CV_8UC1);
    // Contours = cv::Mat::zeros(expand_grid.size(), CV_8UC1);  
    // for(int i = 0;i < approx_contours.size();i++)
    // {
    //   //contours[i]代表的是第i个轮廓，contours[i].size()代表的是第i个轮廓上所有的像素点数
    //   for(int j = 0; j < approx_contours[i].size(); j++) 
    //   {
    //     //绘制出contours向量内所有的像素点
    //     cv::Point P = cv::Point(approx_contours[i][j].x, approx_contours[i][j].y);
    //     Contours.at<uchar>(P) = 255;
    //   }
    //   //输出hierarchy向量内容
    //   char ch[256];
    //   sprintf(ch,"%d",i);
    //   string str=ch;
    //   cout<<"向量hierarchy的第" <<str<<" 个元素内容为："<<endl<<hierarchy[i]<<endl<<endl;

    //   //绘制轮廓
    //   drawContours(imageContours, approx_contours, i, cv::Scalar(255), 1, 8,hierarchy);
    // }
    // cv::imshow("Contours Image", imageContours); //轮廓
    // cv::imshow("Point of Contours", Contours);   //向量contours内保存的所有轮廓点集
    // cv::waitKey(0);
#endif

    std::vector<std::vector<Eigen::Vector2f>> contours_2d;
    contours_2d.reserve(approx_contours.size());
    for (auto & contour : approx_contours)
    {
      vector<Eigen::Vector2f> contour_2d;
      contour_2d.reserve(contour.size());
      // vector<float> x_s, y_s;
      for (auto & point : contour)// 横向是x 纵向是y
      {
        float y = 1.5 - point.x * mcell_size;//横向 
        // float y = (mgrid_dimensions/2 - point.x)*mcell_size;
// #ifdef DEBUG
//         float x = point.x * mcell_size - 1.5;
// #else
//         float x = 3 - point.x * mcell_size;
// #endif
        float x = 3 - point.y * mcell_size;// 纵向
        // float x = (mgrid_dimensions - point.y) * mcell_size;
        contour_2d.emplace_back(Eigen::Vector2f(x, y));// 转到了机器人世界坐标系
        // x_s.emplace_back(x);
        // y_s.emplace_back(y);
      }
      // x_s.emplace_back(*x_s.begin());
      // y_s.emplace_back(*y_s.begin());
      // std::cout<<"obstacles: "<<std::endl;
      // plt::plot(x_s, y_s);
      // plt::show();
      contours_2d.emplace_back(contour_2d);
    }
    return contours_2d;
  }

  heightmap::~heightmap()
  {
    
  }
}