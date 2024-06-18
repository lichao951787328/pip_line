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
#include "plane_detection/quatree_node.h"
#include "plane_detection/type.h"
#include "plane_detection/load_parameter.h"
#include "plane_detection/quadtree_new.h"
#include "plane_detection/plane_new.h"
#include "plane_detection/plane_segmentation.h"
#include <glog/logging.h>
#include <fstream>
#include <chrono>
using namespace std;
orginazed_points raw_points;
parameter param;
index2D index_node_2d;
ros::Publisher pointcloudPub;
// parameter from camera
// [918.8240966796875, 0.0, 664.4627685546875, 0.0, 917.1801147460938, 375.1426696777344, 0.0, 0.0, 1.0]

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
  // 80 cols 40 rows
  cv::Mat clone_image = depth_image(cv::Rect(0, 600, 80, 80)).clone();
  
  raw_points.initial(depth_image);
  LOG(INFO)<<"raw points info: ";
  LOG(INFO)<<"width: "<<raw_points.width<<" height: "<<raw_points.height;
  
  // 测试getRectPoint
  // for (auto & iter_row : raw_points.points)
  // {
  //   for (auto & iter_row_col : iter_row)
  //   {
  //     cout<<iter_row_col.transpose()<<endl;
  //   }
  //   cout<<endl;
  // }
  // vector<Eigen::Vector3f> points = raw_points.getRectPoint(3, 4, 3, 2);
  // for (auto & iter_point : points)
  // {
  //   cout<<iter_point.transpose()<<endl;
  // }
  // pcl::PointCloud<pcl::PointXYZ> cloud;
  // for (auto & iter_row : raw_points.points)
  // {
  //   for (auto & iter_row_col : iter_row)
  //   {
  //     cloud.emplace_back(pcl::PointXYZ(iter_row_col.x(), iter_row_col.y(), iter_row_col.z()));
  //   }
  // }
  // pcl::visualization::CloudViewer viewer("cloud");
  // viewer.showCloud(cloud.makeShared());
  // system("read -p 'Press Enter to continue...' var");
  // while (!viewer.wasStopped())
	// {
	// }

  if (load_parameter(param) == 0)
  {
    param.patch_num_th = param.patch_num_percent_th * param.leafnode_width * param.leafnode_width;
    param.leafnode_depth = log2(raw_points.width/param.leafnode_width);
    param.showParameter();
  }
  else
    cout<<"load parameter error"<<endl;
  
  size_t index_x_rows = depth_image.rows/param.leafnode_width;
  size_t index_y_cols = depth_image.cols/param.leafnode_width;

  // quatree::node root(0, 0, raw_points.width, 0, 0, raw_points.width/p.leafnode_width, 0, nullptr);
  // root.showNodeFrame();
  // 显示点和normal，验证mse。点云法向量变化较大
  // pcl::PointCloud<pcl::PointXYZ> leafpoints;
  // pcl::PointCloud<pcl::Normal> normals;
  // std::list<quatree::node*> L;
  // L.push_back(&root);
  // while (!L.empty())
  // {
  //   quatree::node* tmpnode = L.front();
  //   L.pop_front();
  //   if (tmpnode->is_leafnode)
  //   {
  //     leafpoints.emplace_back(pcl::PointXYZ(tmpnode->center(0), tmpnode->center(1), tmpnode->center(2)));
  //     normals.emplace_back(pcl::Normal(tmpnode->normal(0), tmpnode->normal(1), tmpnode->normal(2)));
  //   }
  //   else
  //   {
  //     for (size_t i = 0; i < 4; i++)
  //     {
  //       if (tmpnode->children.at(i))
  //       {
  //         L.push_back(tmpnode->children.at(i));
  //       }
  //     }
  //   }
  // }
  // cout<<"get info finish"<<endl;
  // cout<<leafpoints.size()<<" "<<normals.size()<<endl;
  // boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer
	// (new pcl::visualization::PCLVisualizer("viewer name"));
	// viewer->setBackgroundColor(0, 0, 0);
	// pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> cloud_color(leafpoints.makeShared(), 255, 0, 0);
	// viewer->addPointCloud(leafpoints.makeShared(),cloud_color, "viewer pcd");
	// viewer->addPointCloudNormals<pcl::PointXYZ, pcl::Normal>(leafpoints.makeShared(), normals.makeShared(), 1, 0.5,"rabbit_normals");
	// viewer->addText("rabbit", 180, 180);
	// viewer->addCoordinateSystem();
	// viewer->spin();  
  
  auto start_quatree = std::chrono::high_resolution_clock::now();
  quatree::quatree qq;
  auto end_quatree = std::chrono::high_resolution_clock::now();
  std::chrono::duration<double> diff_quatree = end_quatree - start_quatree;
  cout<<"creat quatree cost time "<<diff_quatree.count()<<" s"<<endl;
  // qq.showFrame();
  qq.mergePatchsForSeeds();
  qq.showFrame();
  cout<<"....................."<<endl;
  plane_segmentation ps(index_x_rows, index_y_cols, &qq);
  vector<plane> planes = ps.getPlaneResult();
  std::cout<<"cand plane size "<<planes.size()<<endl;

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

  pcl::PointCloud<pcl::PointXYZRGB> cloud;
  size_t index = 0;
  for (auto & iter_plane : planes)
  {
    if (iter_plane.valid_points.size() > 5000)
    {
      for (auto & iter_point : iter_plane.valid_points)
      {
        pcl::PointXYZRGB point(iter_point(0), iter_point(1), iter_point(2), color.at(3 * (index%28)), color.at(3 * (index%28) + 1), color.at(3 * (index%28) + 2));
        cloud.emplace_back(point);
      }
      index ++;
    }
  }
  cout<<"show index "<<index<<endl;
  sensor_msgs::PointCloud2 output;
  pcl::toROSMsg(cloud, output);
  std::cout<<"publish cloud size "<<output.data.size()<<std::endl;
  output.header.frame_id = "camera";
  output.header.stamp = ros::Time::now();
  pointcloudPub.publish(output);
  // std::cout<<"points size "<<planes.front().valid_points.size()<<endl;
  // std::cout<<"points size "<<planes.front().valid_points_size<<endl;
  // list<quatree::node*> node_list = qq.getPatchsList();
  // int return_index = 0;
  // for (auto & iter_node : node_list)
  // {
  //   if (return_index <= 5)
  //   {
  //     LOG(INFO)<<"delete node address is "<<iter_node<<endl;
  //     for (size_t i = 0; i < 4; i++)
  //     {
  //       if (iter_node->parent->children.at(i))
  //       {
  //         if(iter_node->parent->children.at(i) == iter_node)
  //         {
  //           iter_node->parent->children.at(i) = nullptr;
  //           delete iter_node;
  //           iter_node = nullptr;
  //           break;
  //         }
  //       }
  //     }
  //   }
  //   return_index++;
  // }
  
  // quatree::node* seed = qq.getSeedNode();
  // cout<<seed<<endl;
  // for (size_t i = 0; i < 4; i++)
  // {
  //   if (seed->parent->children.at(i))
  //   {
  //     if(seed->parent->children.at(i) == seed)
  //     {
  //       seed->parent->children.at(i) = nullptr;
  //       delete seed;
  //       seed = nullptr;
  //       break;
  //     }
  //   }
  // }

  // qq.showFrame();
  // cout<<qq.getSeedNode()<<endl;
  // qq.checkTree();
  // LOG(INFO)<<"qq has checked."<<endl;
  // qq.showFrame();
  // index2D index_node_2d(index_x_rows, index_y_cols, qq.getRoot());
  // index_node_2d.showInfo();
  // plane tmpplane(index_x_rows, index_y_cols, index_node_2d);
  // quatree::node* seed = qq.getSeedNode();
  // LOG(INFO)<<"SEED: "<<seed<<endl;
  // if (tmpplane.checkInPlane(seed))
  // {
  //   tmpplane.addNode2Plane(seed);
  // }
  
  // qq.showFrame();
  // std::priority_queue<quatree::node*, std::vector<quatree::node*>, quatree::compnode >  neighbors;
  // tmpplane.getNeighborNode(neighbors);
  // std::priority_queue<quatree::node*, std::vector<quatree::node*>, quatree::compnode >  tmpneighbors = neighbors;
  // cout<<"now the neighbor is:"<<endl;
  // while (!tmpneighbors.empty())
  // {
  //   cout<<tmpneighbors.top()<<endl;
  //   tmpneighbors.pop();
  // }
  // cout<<"merge neighbor nodes."<<endl;
  // while (!neighbors.empty())
  // {
  //   quatree::node* tmpnode = neighbors.top();
  //   neighbors.pop();
  //   cout<<"now the node is: "<<tmpnode<<endl;
  //   if (tmpnode->is_plane)
  //   {
  //     if (tmpplane.checkInPlane(tmpnode))
  //     {
  //       cout<<"the node may be in plane"<<endl;
  //       if (tmpplane.addNode2Plane(tmpnode))// 每次加入之后需不需要checktree
  //       {
  //         cout<<"has add node "<<tmpnode<<" to plane."<<endl;
  //         qq.showFrame();
  //         for (auto & iter_x : index_node_2d.index2d )
  //         {
  //           for (auto & iter_x_y : iter_x)
  //           {
  //             cout<<iter_x_y<<" ";
  //           }
  //           cout<<endl;
  //         }
  //         while (!neighbors.empty())
  //         {
  //           neighbors.pop();
  //         }
  //         tmpplane.getNeighborNode(neighbors);
  //         std::priority_queue<quatree::node*, std::vector<quatree::node*>, quatree::compnode >  tmpneighbors = neighbors;
  //         while (!tmpneighbors.empty())
  //         {
  //           cout<<tmpneighbors.top()<<endl;
  //           tmpneighbors.pop();
  //         }
  //       }
  //       else
  //       {
  //         cout<<"has make sure the node is not in plane"<<endl;
  //         while (!neighbors.empty())
  //         {
  //           if (neighbors.top() == tmpnode)
  //           {
  //             neighbors.pop();
  //           }
  //           else
  //             break;
  //         }
  //       }
  //     }
  //     else
  //   {
  //     cout<<"the node is not in plane."<<endl;
  //     while (!neighbors.empty())
  //     {
  //       if (neighbors.top() == tmpnode)
  //       {
  //         neighbors.pop();
  //       }
  //       else
  //         break;
  //     }
  //   }
  //   }
  // }


  // pcl::PointCloud<pcl::PointXYZRGB> color_cloud;
  // std::vector<float> color = {
  //     51, 160, 44,  //0
  //     166, 206, 227 ,
  //     178 , 223 , 138 ,//6
  //     31, 120 , 180 ,
  //     251 , 154 , 153 ,// 12
  //     227 , 26 , 28 ,
  //     253 , 191 , 111 ,// 18
  //     106 , 61 , 154 ,
  //     255 , 127 , 0 , // 24
  //     202 , 178 , 214 ,
  //     255, 0.0, 0.0, // red // 30
  //     0.0, 255, 0.0, // green
  //     0.0, 0.0, 255, // blue// 36
  //     255, 255, 0.0,
  //     255, 0.0, 255, // 42
  //     0.0, 255, 255,
  //      177.5, 255, 0.0,
  //     255, 177.5, 0.0,
  //      177.5, 0.0, 255,
  //     255, 0.0,  177.5,
  //     0.0,  177.5, 255,
  //     0.0, 255,  177.5,
  //     255,  177.5,  177.5,
  //      177.5, 255,  177.5,
  //      177.5,  177.5, 255,
  //      177.5,  177.5, 255,
  //      177.5, 255,  177.5,
  //      177.5,  177.5, 255};
  // size_t color_index = 0;
  // std::list<quatree::node*> L;
  // L.push_back(qq.getRoot());
  // // pcl::PointCloud<pcl::PointXYZ> cloud;
  // while (!L.empty())
  // {
  //   quatree::node* tmpnode = L.front();
  //   L.pop_front();
  //   if (tmpnode->is_plane)
  //   {
  //     for (auto & iter_point : tmpnode->valid_points)
  //     {
  //       pcl::PointXYZRGB point(iter_point(0), iter_point(1), iter_point(2), color.at(3 * (color_index%28)), color.at(3 * (color_index%28) + 1), color.at(3 * (color_index%28) + 2));
  //       color_cloud.emplace_back(point);
  //     }
  //     color_index ++;
  //   }
  //   else
  //   {
  //     for (size_t i = 0; i < 4; i++)
  //     {
  //       if (tmpnode->children.at(i))
  //       {
  //         L.push_back(tmpnode->children.at(i));
  //       }
  //     }
  //   }
  // }
  // pcl::visualization::CloudViewer viewer("cloud");
  // viewer.showCloud(color_cloud.makeShared());
  // // system("read -p 'Press Enter to continue...' var");
  // while (!viewer.wasStopped())
	// {
	// }
  // qq.showFrame();
  // index_node_2d = quatree::index2D(index_x_rows, index_y_cols, qq.getRoot());
  // index_node_2d.showInfo();
  // cout<<endl;

  // auto start_merge = std::chrono::high_resolution_clock::now();
  // qq.mergePatchsForSeeds();
  // auto end_merge = std::chrono::high_resolution_clock::now();
  // std::chrono::duration<double> diff_merge = end_merge - start_merge;
  // cout<<"merge quatree cost time "<<diff_merge.count()<<" s"<<endl;
  // index_node_2d = quatree::index2D(index_x_rows, index_y_cols, qq.getRoot());
  // index_node_2d.showInfo();
  // qq.showFrame();

  // std::ofstream index_address("/home/humanoid/work/plane_detection/catkin_plane_detection_test_ws/index_address.dat");
  // for (auto & iter_x : index_node_2d.index2d )
  // {
  //   for (auto & iter_x_y : iter_x)
  //   {
  //     index_address<<iter_x_y<<" ";
  //   }
  //   index_address<<endl;
  // }
  // plane tmpplane(index_x_rows, index_y_cols);
  // quatree::node* seed = qq.getSeedNode();
  // LOG(INFO)<<"SEED: "<<seed<<endl;
  // if (tmpplane.checkInPlane(seed))
  // {
  //   tmpplane.addNode2Plane(seed);
  // }
  // qq.showFrame();
  // std::priority_queue<quatree::node*, std::vector<quatree::node*>, decltype(quatree::cmp)>  neighbors(quatree::cmp);
  // tmpplane.getNeighborNode(neighbors);
  // std::priority_queue<quatree::node*, std::vector<quatree::node*>, decltype(quatree::cmp)>  tmpneighbors = neighbors;
  // cout<<"now the neighbor is:"<<endl;
  // while (!tmpneighbors.empty())
  // {
  //   cout<<tmpneighbors.top()<<endl;
  //   tmpneighbors.pop();
  // }
  // cout<<"merge neighbor nodes."<<endl;
  // while (!neighbors.empty())
  // {
  //   quatree::node* tmpnode = neighbors.top();
  //   neighbors.pop();
  //   cout<<"now the node is: "<<tmpnode<<endl;
  //   if (tmpplane.checkInPlane(tmpnode))
  //   {
  //     cout<<"the node may be in plane"<<endl;
  //     if (tmpplane.addNode2Plane(tmpnode))
  //     {
  //       cout<<"has add node "<<tmpnode<<" to plane."<<endl;
  //       qq.showFrame();
  //       for (auto & iter_x : index_node_2d.index2d )
  //       {
  //         for (auto & iter_x_y : iter_x)
  //         {
  //           cout<<iter_x_y<<" ";
  //         }
  //         cout<<endl;
  //       }
  //       while (!neighbors.empty())
  //       {
  //         neighbors.pop();
  //       }
  //       tmpplane.getNeighborNode(neighbors);
  //       std::priority_queue<quatree::node*, std::vector<quatree::node*>, decltype(quatree::cmp)>  tmpneighbors = neighbors;
  //       while (!tmpneighbors.empty())
  //       {
  //         cout<<tmpneighbors.top()<<endl;
  //         tmpneighbors.pop();
  //       }
  //     }
  //     else
  //     {
  //       cout<<"has make sure the node is not in plane"<<endl;
  //       while (!neighbors.empty())
  //       {
  //         if (neighbors.top() == tmpnode)
  //         {
  //           neighbors.pop();
  //         }
  //         else
  //           break;
  //       }
  //     }
  //   }
  //   else
  //   {
  //     cout<<"the node is not in plane."<<endl;
  //     while (!neighbors.empty())
  //     {
  //       if (neighbors.top() == tmpnode)
  //       {
  //         neighbors.pop();
  //       }
  //       else
  //         break;
  //     }
  //   }
  // }

  // 针对连续提取的情况 
  // std::vector<plane> planes;
  // quatree::node* seed = qq.getSeedNode();
  // while (seed)
  // {
  //   LOG(INFO)<<"seed node is "<<seed<<endl;
  //   plane tmpplane(index_x_rows, index_y_cols);
  //   if (tmpplane.checkInPlane(seed))
  //   {
  //     tmpplane.addNode2Plane(seed);
  //   }
  //   cout<<"patchs size is "<<tmpplane.patchs.size()<<endl;
  //   std::priority_queue<quatree::node*, std::vector<quatree::node*>, decltype(quatree::cmp)>  neighbors(quatree::cmp);
  //   tmpplane.getNeighborNode(neighbors);
  //   while (!neighbors.empty())
  //   {
  //     quatree::node* tmpnode = neighbors.top();
  //     neighbors.pop();
  //     LOG(INFO)<<"now the node is: "<<tmpnode<<endl;
  //     if (tmpplane.checkInPlane(tmpnode))// node不是平面，不能添加
  //     {
  //       cout<<"patchs size is "<<tmpplane.patchs.size()<<endl;
  //       LOG(INFO)<<"the node may be in plane"<<endl;
  //       if (tmpplane.addNode2Plane(tmpnode))
  //       {
  //         LOG(INFO)<<"has add node "<<tmpnode<<" to plane."<<endl;
  //         qq.showFrame();
  //         for (auto & iter_x : index_node_2d.index2d )
  //         {
  //           for (auto & iter_x_y : iter_x)
  //           {
  //             cout<<iter_x_y<<" ";
  //           }
  //           cout<<endl;
  //         }
  //         while (!neighbors.empty())
  //         {
  //           neighbors.pop();
  //         }
  //         tmpplane.getNeighborNode(neighbors);
  //         std::priority_queue<quatree::node*, std::vector<quatree::node*>, decltype(quatree::cmp)>  tmpneighbors = neighbors;
  //         while (!tmpneighbors.empty())
  //         {
  //           LOG(INFO)<<tmpneighbors.top()<<endl;
  //           tmpneighbors.pop();
  //         }
  //       }
  //       else
  //       {
  //         LOG(INFO)<<"has make sure the node is not in plane"<<endl;
  //         while (!neighbors.empty())
  //         {
  //           if (neighbors.top() == tmpnode)
  //           {
  //             neighbors.pop();
  //           }
  //           else
  //             break;
  //         }
  //       }
  //     }
  //     else
  //     {
  //       // 对于不是平面的patch，将patch内的点加入
  //       // 虽然这种方式会导致很近的邻近平面上的点加入，但这样概率是极小的，一方面patch本身就很小，另一方面如果邻近属于一个平面，patch包含少许信息也不影响
  //       LOG(INFO)<<"put points of node in tmpplane"<<endl;
  //       vector<Eigen::Vector3f>::iterator iter_point = tmpnode->valid_points.begin();
  //       while (iter_point != tmpnode->valid_points.end())
  //       {
  //         if (tmpplane.checkPointInPlane(*iter_point))
  //         {
  //           if (tmpplane.addPoint2Plane(*iter_point))
  //           {
  //             tmpnode->valid_points.erase(iter_point);
  //           }
  //           else
  //             iter_point++;
  //         }
  //         else
  //           iter_point++;
  //       }  
  //       LOG(INFO)<<"the node is not in plane."<<endl;
  //       while (!neighbors.empty())
  //       {
  //         if (neighbors.top() == tmpnode)
  //         {
  //           neighbors.pop();
  //         }
  //         else
  //           break;
  //       }
  //     }
  //   }
  //   qq.showFrame();
  //   qq.checkTree();
  //   qq.showFrame();
  //   planes.emplace_back(tmpplane);
  //   LOG(INFO)<<"**************************"<<endl;
  //   seed = qq.getSeedNode();
  //   LOG(INFO)<<"now seed node is "<<seed<<endl;
  //   qq.showFrame();
  // }
  // std::cout<<planes.size()<<endl;
  // std::cout<<"patchs in plane: "<<endl;
  // for (auto & iter_patch : tmpplane.patchs)
  // {
  //   cout<<iter_patch<<endl;
  // }
  // for (auto & iter_rows : index_node_2d.index2d)
  // {
  //   for(auto & iter_rows_cols : iter_rows)
  //   {
  //     std::cout<<iter_rows_cols<<" ";
  //   }
  //   std::cout<<std::endl;
  // }
  // qq.showFrame();
  // LOG(INFO)<<"CHENK TREE"<<endl;
  // qq.checkTree();
  // LOG(INFO)<<"CHECK OVER"<<endl;
  // qq.showFrame();
  // cout<<"seed is "<<qq.getSeedNode()<<endl;
  // cv::Mat image = cv::imread("/home/humanoid/work/plane_detection/catkin_plane_detection_test_ws/src/plane_detection/bag/color/000000.jpg");
  // std::cout<<"load image"<<std::endl;
  // cv::imshow("window",image);
  // std::cout<<"show image"<<endl;
  // int index = 0;
  // std::cout<<"prepare enter while loop"<<endl;
  // while (!merge_patchs.empty())
  // {
  //   quatree::node* tmpnode = merge_patchs.top();
  //   merge_patchs.pop();
  //   cv::Point p1(tmpnode->start_cols, tmpnode->start_rows);
  //   cv::Point p2(tmpnode->start_cols+tmpnode->width, tmpnode->start_rows + tmpnode->width);
  //   cv::Scalar c(color.at((index%28)*3), color.at((index%28)*3 + 1), color.at((index%28)*3 + 2));
  //   cv::rectangle(image, p1, p2, c, -1);
  //   // cv::rectangle(image, (tmpnode->start_rows, tmpnode->start_cols), (tmpnode->start_rows + tmpnode->width, tmpnode->start_cols+tmpnode->width), (color.at(index*3), color.at(index*3 + 1), color.at(index*3 + 2)), cv::FILLED, 8);
  //   index ++;
  // }
  // cv::imwrite("/home/humanoid/work/plane_detection/catkin_plane_detection_test_ws/src/plane_detection/bag/color/output1.jpg", image);
  // std::cout<<"draw finish"<<endl;
  // cv::imshow("window",image);
  // cv::waitKey(0);
  // cv::imwrite("/home/humanoid/work/plane_detection/catkin_plane_detection_test_ws/src/plane_detection/bag/color/output.jpg", image);
  first = false;
  
  // pcl::PointCloud<pcl::PointXYZ> points;
  // for (size_t i = 0; i < height; i++)
  // {
  //   for (size_t j = 0; j < width; j++)
  //   {
  //     // 没有使用迭代器
  //     float z = (float)(depth_image.at<unsigned short>(i, j)) / kScaleFactor;
  //     if (std::isnan(z))
  //     {
  //       points.emplace_back(pcl::PointXYZ(0, 0, z));
  //       continue;
  //     }
  //     float x = ((float)j - kCx) * z / kFx;
  //     float y = ((float)i - kCy) * z / kFy;
  //     points.emplace_back(pcl::PointXYZ(x, y, z)); 
  //   }
  // }
  // pcl::visualization::CloudViewer viewer("points");
  // viewer.showCloud (points.makeShared());
  // while (!viewer.wasStopped ())
  // {
  // }
}


int main(int argc, char** argv)
{
  google::InitGoogleLogging(argv[0]); 
  google::InstallFailureSignalHandler();
  // google::SetCommandLineOptionWithMode("FLAGS_minloglevel", "2");
  FLAGS_minloglevel = 2;
  FLAGS_colorlogtostderr = true;
  // FLAGS_log_dir = "./log"; 
  FLAGS_alsologtostderr = true;
  LOG(INFO)<<"initial glog finish"<<endl;
  ros::init(argc, argv, "listener");
  ros::NodeHandle n;
  LOG(INFO)<<"camera infomation: "<<endl;
  LOG(INFO)<<"kFx: "<<kFx<<" kFy: "<<kFy<<" kCx: "<<kCx<<" kCy: "<<kCy<<endl;
  ros::Subscriber sub = n.subscribe("/camera/aligned_depth_to_color/image_raw", 1000, chatterCallback);
  pointcloudPub= n.advertise<sensor_msgs::PointCloud2>("plane_point", 1);
  sleep(1);
  ros::spin();
  google::ShutdownGoogleLogging();
  return 0;
}