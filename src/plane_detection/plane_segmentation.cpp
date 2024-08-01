/*
 * @description: 
 * @param : 
 * @return: 
 */
#include <plane_detection/plane_segmentation.h>
#include <plane_detection/quatree_node.h>
#include <plane_detection/type.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <ctime>
#include "opencv2/imgproc/imgproc.hpp"
// #include <pcl_conversions/pcl_conversions.h>
// #include <ros/ros.h>
// extern ros::Publisher pointcloudPub;
// #define DEBUG
// #define SHOW_PROCESSING
// #define SHOWIMAGE
// extern double refresh_quatree;
plane_segmentation::plane_segmentation(size_t rows, size_t cols, quatree::quatree * pqq_, orginazed_points & raw_points_, parameter & param_):raw_points(raw_points_),param(param_)
{
  pqq = pqq_;
  segmented_index = 0;
  segmented_result = cv::Mat(rows, cols, CV_8UC3, cv::Scalar(0, 0, 0));
  segmented_contours = cv::Mat(rows, cols, CV_8UC3, cv::Scalar(0, 0, 0));
  colors_ = {
      51, 160, 44,  //0
      166, 206, 227,
      178, 223, 138,//6
      31 , 120, 180,
       251, 154, 153,// 12
       227, 26 , 28 ,
      253, 191, 111,// 18
      106, 61 , 154,
       255, 127, 0  , // 24
      202, 178, 214};
  // std::cout<<"colors size "<<colors_.size()<<std::endl;
  std::cout<<"start detection"<<std::endl;
  while (1)
  {
    // index_node_2d = index2D(x, y, pqq->getRoot());// 用于使用index获取节点
    std::shared_ptr<quatree::node> seed = pqq->getSeedNode();
    // showNodeAndNeisImage(seed);
    // LOG(INFO)<<seed->start_rows<<" "<<seed->start_cols;
    // std::cout<<"seed: "<<seed<<std::endl;
    if (seed && seed->valid_points_size >= 8)
    {
      plane tmpplane(rows, cols, seed, raw_points, param);
      // cout<<"tmp plane"<<endl;
      // getPlane(tmpplane);
      tmpplane.regionGrowing();
      // LOG(INFO)<<"N: "<<tmpplane.stats.N;
      if (tmpplane.stats.N < 180)// 0.26/0.02 * 0.28/0.02
      {
        continue;
      }
      // cv::imshow("contour_image", tmpplane.contour_image);
      // cv::waitKey(0);
      // 遍历灰度图像的每个像素
      // 指定一种颜色 (例如红色)
      int color_index = segmented_index%(colors_.size()/3);
      cv::Vec3b color(colors_.at(color_index * 3), colors_.at(color_index * 3 + 1), colors_.at(color_index * 3 + 2)); // BGR格式，红色

      // 创建掩码 (只保留白色区域)
      cv::Mat mask;
      cv::inRange(tmpplane.contour_image, cv::Scalar(255), cv::Scalar(255), mask);
      // cv::imshow("mask", mask);
      // cv::waitKey(0);
      // 高斯也是一个比较耗时的操作，这里先不加了
      // 高斯模糊减少噪声
      // cv::Mat blurred;
      // cv::GaussianBlur(tmpplane.contour_image, blurred, cv::Size(71, 71), 0);

      // // 二值化
      // cv::Mat mask;
      // cv::threshold(blurred, mask, 180, 255, cv::THRESH_BINARY);
      // 创建一个与彩色图像相同大小的全色图像，填充为指定颜色
      cv::Mat colorMask(segmented_result.size(), segmented_result.type(), color);
      // cv::imshow("colorMask", colorMask);
      // cv::waitKey(0);
      // 使用掩码将白色区域赋值为指定颜色
      colorMask.copyTo(segmented_result, mask);
      // cv::imshow("segmented_result", segmented_result);
      // cv::waitKey(0);
      // cv::Mat single_result(segmented_result.size(), segmented_result.type(), cv::Vec3b(0, 0, 0));

      // colorMask.copyTo(single_result, mask);

      seg_images.emplace_back(tmpplane.contour_image);
      // cv::imshow("segmented image", tmpplane.contour_image);
      // cv::waitKey(0);
      // if (tmpplane.isEmpty())
      // {
      //   std::cout<<"plane is empty"<<endl;
      //   break;
      // }
      // std::cout<<"plane size is "<<tmpplane.valid_points_size<<std::endl;
      // tmpplane.testmedianBlur();
      // tmpplane.testGaussianBlur();
      // tmpplane.calculatePlaneParamNofilter();
      // tmpplane.calculatePlaneParammedianBlur();
      // plane_info tmppplane_info = tmpplane.calculatePlaneParamNofilter();
      // for (auto & contour : tmppplane_info.contours)
      // {
      //   vector<float> x, y;
      //   for (auto & point_3d : contour)
      //   {
      //     x.emplace_back(point_3d.x());
      //     y.emplace_back(point_3d.y());
      //   }
      // }
      

      // 返回平面参数
      planes.emplace_back(tmpplane.calculatePlaneParamtestGaussianBlur());
      // std::cout<<"contours information "<<endl;
      // for (auto & contour : tmpplane.calculatePlaneParamtestGaussianBlur().contours)
      // {
      //   std::cout<<"start:"<<endl;
      //   for (auto & point_3d : contour)
      //   {
      //     std::cout<<point_3d.transpose()<<endl;
      //   }
      //   std::cout<<endl;
      // }
      // std::cout<<"111"<<std::endl;
#ifdef SHOWIMAGE
      cv::Mat Gaussian_filter_image;
      cv::GaussianBlur(tmpplane.contour_image, Gaussian_filter_image, cv::Size(5, 5), 1, 1);
      tmpplane.contour_image = Gaussian_filter_image;
      std::vector<std::vector<cv::Point>> contours;
      std::vector<cv::Vec4i> hierarchy;
      cv::findContours(tmpplane.contour_image, contours, hierarchy, cv::RETR_CCOMP, cv::CHAIN_APPROX_NONE, cv::Point());
      // size_t r = colors_.at(segmented_index%(colors_.size()/3) * 3);
      // size_t g = colors_.at(segmented_index%(colors_.size()/3) * 3 + 1);
      // size_t b = colors_.at(segmented_index%(colors_.size()/3) * 3 + 2);
      // cv::drawContours(segmented_result, contours, -1, CV_RGB(r,g,b), -1);
      // // std::cout<<segmented_result.cols<<" "<<segmented_result.rows<<std::endl;
      // cv::imshow("segmented result",segmented_result);
      // cv::imwrite("/home/humanoid/work/plane_detection/catkin_plane_detection_test_ws/src/plane_detection/bag/segmented_result.png", segmented_result);
      // cv::waitKey(0);

      if (!contours.empty() && !hierarchy.empty()) 
      {
        std::vector<std::vector<cv::Point> >::const_iterator itc = contours.begin();
        // 遍历所有轮廓
        while (itc != contours.end()) 
        {
          // 定位当前轮廓所在位置
          cv::Rect rect = cv::boundingRect(cv::Mat(*itc));
          // contourArea函数计算连通区面积
          double area = contourArea(*itc);
          // 若面积小于设置的阈值
          if (area < 1000) 
          {
            // 遍历轮廓所在位置所有像素点
            for (int i = rect.y; i < rect.y + rect.height; i++) 
            {
              uchar *output_data = tmpplane.contour_image.ptr<uchar>(i);
              for (int j = rect.x; j < rect.x + rect.width; j++) 
              {
                output_data[j] = 255;
                // // 将连通区的值置0
                // if (output_data[j] == 0) 
                // {
                //   output_data[j] = 255;
                // }
              }
            }
          }
          itc++;
        }
      }

      cv::findContours(tmpplane.contour_image, contours, hierarchy, cv::RETR_CCOMP, cv::CHAIN_APPROX_NONE, cv::Point());

      size_t r = colors_.at(segmented_index%(colors_.size()/3) * 3);
      size_t g = colors_.at(segmented_index%(colors_.size()/3) * 3 + 1);
      size_t b = colors_.at(segmented_index%(colors_.size()/3) * 3 + 2);
      cv::drawContours(segmented_result, contours, -1, CV_RGB(r,g,b), -1);
      // std::cout<<segmented_result.cols<<" "<<segmented_result.rows<<std::endl;
      cv::imshow("segmented result",segmented_result);
      cv::imwrite("/home/humanoid/work/plane_detection/catkin_plane_detection_test_ws/src/plane_detection/bag/segmented_result_" + std::to_string(segmented_index) + ".png", segmented_result);
      cv::waitKey(0);

      vector<vector<cv::Point>> approx_contours(contours.size());
      vector<vector<cv::Point>>::iterator approx_contour_iter = approx_contours.begin();
      for (auto & coutour : contours)
      {
        cv::approxPolyDP(coutour, *approx_contour_iter, 9, true);
        approx_contour_iter++;
      }
      cv::drawContours(segmented_contours, approx_contours, -1, CV_RGB(r,g,b), -1);
      cv::drawContours(segmented_contours, approx_contours, -1, CV_RGB(255, 0, 0), 2);
      cv::imwrite("/home/humanoid/work/plane_detection/catkin_plane_detection_test_ws/src/plane_detection/bag/segmented_contours.png", segmented_contours);
      cv::waitKey(0);
#endif
      // clock_t start = clock();
      // 更新树及邻近关系
      pqq->refreshTree();
      if (!pqq->getRoot())
      {
          break;
      }
      pqq->PreRefreshIndex2D();
      pqq->getQuatreeNeighbors();
      // refresh_quatree += (double)(clock() - start)/CLOCKS_PER_SEC;
      // std::cout<<"refresh quatree costs "<<refresh_quatree <<" s."<<std::endl;

      segmented_index++;
      std::cout<<"segmented_index: "<<segmented_index<<std::endl;
    }
    else
      break;
      
    // cout<<"++++"<<endl;
    // std::cout<<"after check tree"<<std::endl;
    // pqq->showFrame();
  }
  cout<<"end detection"<<endl;
}
void plane_segmentation::getSinglePlaneResult()
{
  
}


void plane_segmentation::showNodeAndNeisImage(std::shared_ptr<quatree::node> n)
{
  // 如果有彩色图时
  cv::Mat color_image = cv::imread("/home/lichao/TCDS/src/pip_line/data/rotatedImage.png");
  // LOG(INFO)<<"color_image size: "<<color_image.size;
  // 如果没有彩色图时
  // cv::Mat color_image = cv::Mat::zeros(raw_points.height, raw_points.width, CV_8UC3);
  // 对于是平面节点的点画点，对于非平面节点画叉叉，节点边界用细线画
  int scale = 8;
  

  for (auto & no : n->neighbors)
  {
    cv::rectangle(color_image, cv::Rect(no->start_cols * scale, no->start_rows * scale, no->width * scale, no->width * scale), cv::Scalar(166, 206, 227), 2);
  }
  cv::rectangle(color_image, cv::Rect(n->start_cols * scale, n->start_rows * scale, n->width * scale, n->width * scale), cv::Scalar(20, 160, 10), 3);
  cv::imwrite("/home/lichao/TCDS/src/pip_line/data/seed_nei.png", color_image);
  cv::imshow("leaf node", color_image);
  cv::waitKey(0);
}

// void plane_segmentation::getPlane(plane & plane)
// {
//   std::shared_ptr<quatree::node> seed = pqq->getSeedNode();
//   LOG(INFO)<<"SEED: "<<seed<<endl;
//   if (seed == nullptr)
//   {
//     return;
//   }
//   if (plane.checkInPlane(seed))
//   {
//     plane.addNode2Plane(seed);
//   }
// #ifdef SHOW_PROCESSING
//   pcl::PointCloud<pcl::PointXYZ> cloud;
//   for (auto & iter_point : plane.valid_points)
//   {
//     cloud.emplace_back(pcl::PointXYZ(iter_point(0), iter_point(1), iter_point(2)));
//   }
//   sensor_msgs::PointCloud2 output;
//   pcl::toROSMsg(cloud, output);
//   output.header.frame_id = "camera";
//   pointcloudPub.publish(output);
//   usleep(10000);
// #endif
//   std::priority_queue<std::shared_ptr<quatree::node>, std::vector<std::shared_ptr<quatree::node>>, quatree::compnode>  neighbors;
//   plane.getNeighborNode(neighbors);
// #ifdef DEBUG
//   std::priority_queue<std::shared_ptr<quatree::node>, std::vector<std::shared_ptr<quatree::node>>, quatree::compnode >  tmpneighbors = neighbors;
//   cout<<"now the neighbor is:"<<endl;
//   while (!tmpneighbors.empty())
//   {
//     cout<<tmpneighbors.top()<<" ";
//     tmpneighbors.pop();
//   }
//   cout<<"merge neighbor nodes."<<endl;
// #endif
//   while (!neighbors.empty())
//   {
//     std::shared_ptr<quatree::node> tmpnode = neighbors.top();
//     neighbors.pop();
//     if(!tmpnode)
//       continue;
//     // cout<<"now the node is: "<<tmpnode<<endl;
//     // 先判断是否为平面
//     if (tmpnode->is_plane)
//     {
//       if (plane.checkInPlane(tmpnode))
//       {
//         // cout<<"the node may be in plane"<<endl;
//         if (plane.addNode2Plane(tmpnode))
//         {
// #ifdef DEBUG
//           cout<<"has add node "<<tmpnode<<" to plane."<<endl;
//           // pqq->showFrame();
//           for (auto & iter_x : index_node_2d.index2d )
//           {
//             for (auto & iter_x_y : iter_x)
//             {
//               cout<<iter_x_y<<" ";
//             }
//             cout<<endl;
//           }
// #endif
//           // 这个地方可以不重新计算邻近点，只需加入新加入节点的邻近点即可
//           // 先这么实现，后期再优化
//           while (!neighbors.empty())
//           {
//             neighbors.pop();
//           }
//           plane.getNeighborNode(neighbors);
// #ifdef DEBUG
//           std::priority_queue<std::shared_ptr<quatree::node>, std::vector<std::shared_ptr<quatree::node>>, quatree::compnode >  tmpneighbors = neighbors;
//           while (!tmpneighbors.empty())
//           {
//             cout<<tmpneighbors.top()<<" ";
//             tmpneighbors.pop();
//           }
// #endif
// #ifdef SHOW_PROCESSING
//           cloud.clear();
//           for (auto & iter_point : plane.valid_points)
//           {
//             cloud.emplace_back(pcl::PointXYZ(iter_point(0), iter_point(1), iter_point(2)));
//           }
//           output.data.clear();
//           pcl::toROSMsg(cloud, output);
//           std::cout<<"publish cloud size "<<output.data.size()<<std::endl;
//           output.header.frame_id = "camera";
//           output.header.stamp = ros::Time::now();
//           pointcloudPub.publish(output);
//           usleep(10000);
// #endif
//         }
//         else
//         {
//           // cout<<"has make sure the node is not in plane"<<endl;
//           while (!neighbors.empty())
//           {
//             if (neighbors.top() == tmpnode)
//             {
//               neighbors.pop();
//             }
//             else
//               break;
//           }
//         }
//       }
//       else
//       {
//         // cout<<"the node is not in plane."<<endl;
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
//       LOG(INFO)<<"put points of node in tmpplane"<<endl;
//       CHECK(tmpnode->valid_points.size() == tmpnode->valid_points_index.size());
//       vector<Eigen::Vector3f>::iterator iter_point = tmpnode->valid_points.begin();
//       vector<std::pair<size_t, size_t>>::iterator iter_index = tmpnode->valid_points_index.begin();
//       LOG(INFO)<<"patch valid points size "<<tmpnode->valid_points.size()<<endl;
//       LOG(INFO)<<"patch valid points size "<<tmpnode->valid_points_size<<endl;
//       // 如果一个patch内所有点加入了，第一是否需要考虑对patch属于平面的条件进行更改
//       // 如果不是patch的平面的所有点加入了，是不是需要把这个patch加入plane中寻找新的相邻节点
//       // 邻近节点的初始化,可以将参数设置为渐变型，根距离相关
//       size_t flag_index = 0;
//       while (iter_point != tmpnode->valid_points.end())
//       {
//         if (plane.checkPointInPlane(*iter_point))
//         {
//           if (plane.addPoint2Plane(*iter_point))
//           {
//             std::cout<<"put point in plane"<<std::endl;
//             LOG(INFO)<<"push point into plane"<<endl;
//             tmpnode->valid_points.erase(iter_point);
//             tmpnode->valid_points_index.erase(iter_index);
//             tmpnode->valid_points_size--;
//             flag_index++;
// #ifdef SHOW_PROCESSING
//             cloud.clear();
//             for (auto & iter_point : plane.valid_points)
//             {
//               cloud.emplace_back(pcl::PointXYZ(iter_point(0), iter_point(1), iter_point(2)));
//             }
//             output.data.clear();
//             pcl::toROSMsg(cloud, output);
//             std::cout<<"publish cloud size "<<output.data.size()<<std::endl;
//             output.header.frame_id = "camera";
//             output.header.stamp = ros::Time::now();
//             pointcloudPub.publish(output);
//             usleep(10000);
// #endif
//           }
//           else
//           {
//             iter_point++;
//             iter_index++;
//           }
//         }
//         else
//         {
//           iter_point++;
//           iter_index++;
//         }
//       }
//       LOG(INFO)<<"123456789"<<endl;
//       LOG(INFO)<<"patch valid points size "<<tmpnode->valid_points.size()<<endl;
//       LOG(INFO)<<"patch valid points size "<<tmpnode->valid_points_size<<endl;
//       LOG(INFO)<<"the points in node is put into plane finish."<<endl;
//       // 一个patch内的点全部加入到同一个平面中，可以是分多次，也需要将此patch的地址加入到该平面中
//       // 可以考虑先判断所有的点是否满足加入条件，再将所有的点一次性加入当前平面中，避免了每加入一个点就要计算一次，原因是当patch不是平面的情况已经非常靠后了，单个点的加入对平面信息影响很小   
//       // 对于点的加入还需要再考量，毕竟每次加入一个点再更新一次太浪费时间
//       if (flag_index == tmpnode->initial_size)
//       {
//         std::cout<<"node is not a plane, but its points are in candidate plane"<<std::endl;
//         plane.refreshIndexImage(tmpnode);
//         // 这个地方可以不重新计算邻近点，只需加入新加入节点的邻近点即可
//         // 先这么实现，后期再优化
//         while (!neighbors.empty())
//         {
//           neighbors.pop();
//         }
//         plane.getNeighborNode(neighbors);
//       }
//       else
//       {
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
//       tmpnode->valid_points_size = tmpnode->valid_points.size();
//       if (tmpnode->valid_points.size() == 0)
//       {
//         std::vector<std::vector<std::shared_ptr<quatree::node>>>::iterator iter_rows = index_node_2d.index2d.begin() + tmpnode->start_rows_2d;
//         for (size_t i = 0; i < tmpnode->width_2d; i++)
//         {
//           std::for_each(iter_rows->begin() + tmpnode->start_cols_2d, iter_rows->begin() + tmpnode->start_cols_2d + tmpnode->width_2d, [](std::shared_ptr<quatree::node> & p){ p = nullptr;});
//           iter_rows++;
//         }
//         quatree::deleteNodeInQuatree(tmpnode);
//         LOG(INFO)<<"the patch is empty"<<endl;
//         // if (tmpnode->parent)
//         // {
//         //   for (size_t i = 0; i < 4; i++)
//         //   {
//         //     if (tmpnode->parent->children.at(i))
//         //     {
//         //       if(tmpnode->parent->children.at(i) == tmpnode)
//         //       {
//         //         tmpnode->parent->children.at(i) = nullptr;
//         //         delete tmpnode;
//         //         tmpnode == nullptr;
//         //         break;
//         //       }
//         //     }
//         //   }
//         // }
//       }
//     }
//   }
// }

plane_segmentation::~plane_segmentation()
{
  planes.clear();
}