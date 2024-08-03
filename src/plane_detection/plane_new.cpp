/*
 * @description: 
 * @param : 
 * @return: 
 */
#include "plane_detection/plane_new.h"
#include <Eigen/Eigenvalues>
#include <Eigen/Core>
#include "opencv2/imgproc/imgproc.hpp"
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include "matplotlibcpp.h"
namespace plt = matplotlibcpp;
// #define DEBUG
// #define SHOWIMAGE
// extern parameter param;
// extern double test_double;
// extern ros::Publisher pointcloudPub;
// extern orginazed_points raw_points;
/**
 * @description: 平面初始化
 * @param {size_t} x 输入深度图的行
 * @param {size_t} y 输入深度图的列
 * @param {index2D &} index_node_2d_ 图像对应的节点地址编号
 * @return {*}
 */
plane::plane(size_t x, size_t y, /* index2D & index_node_2d_ */std::shared_ptr<quatree::node> seed, orginazed_points & raw_points_, parameter & param_):raw_points(raw_points_),param(param_)
{
  // 加上要
  // contour_image = cv::Mat(x, y, CV_8UC1, cv::Scalar(0));//
  contour_image = cv::Mat::zeros(x, y, CV_8UC1);
  // std::cout<<"initial: "<<contour_image.rows<<" "<<contour_image.cols<<endl;
  // index_node_2d = index_node_2d_;
  // LOG(INFO)<<"plane index image: "<<index_image.rows<<" "<<index_image.cols<<std::endl;
  addNode2Plane(seed);
  // nodesHadChecked.insert(seed);
}

void plane::regionGrowing()
{
  while (!p_queue.empty())
  {
    // std::cout<<"test double "<<test_double;
    // showInfo();
    // LOG(INFO)<<*(--neighbors.end());
    // pcl::PointCloud<pcl::PointXYZ> cloud;
    // for (auto & iter_point : valid_points)
    // {
    //   cloud.emplace_back(pcl::PointXYZ(iter_point.x(), iter_point.y(), iter_point.z()));
    // }
    // sensor_msgs::PointCloud2 output;
    // pcl::toROSMsg(cloud, output);
    // output.header.frame_id = "camera";
    // output.header.stamp = ros::Time::now();
    // pointcloudPub.publish(output);
    // 为什么不选择最大的节点，而是直接使用排序？
    // std::shared_ptr<quatree::node> tmpnode = *(--neighbors.end());

    std::shared_ptr<quatree::node> tmpnode = p_queue.top();
    p_queue.pop();
    if (close_set.find(node2string(tmpnode)) != close_set.end())// 表示之前添加过
    {
      continue;
    }
    

    // LOG(INFO)<<"node info: "<<tmpnode<<" "<<tmpnode->valid_points_size<<" "<<tmpnode->valid_points.size()<<" "<<tmpnode->mse;
    // neighbors.erase(tmpnode);

    // 已经加入了的，就不再考虑
    // 表示tmpnode已经检查过了，不论其是否在平面中，都已经检查过了
    // 那会不会随着节点的加入，导致平面信息改变，即使开始不满足，后来又满足了呢？？
    // if (nodesHadChecked.count(tmpnode) != 0)
    // {
    //   continue;
    // }
    // 这种情况只会把平面节点加入，其他情况下均把点加入
    if (tmpnode->is_plane && tmpnode->is_validnode)
    {
      // cout<<"is plane node"<<endl;
      if (checkInPlane(tmpnode))
      {
        // cout<<"in plane"<<endl;
        addNode2Plane(tmpnode);
      }
      // nodesHadChecked.insert(tmpnode);
      // cv::imshow("contour image", contour_image);
      // cv::waitKey(0);
      // 如果一个节点不满足checkInPlane，但是你并不能在neighbors内每个元素的邻近节点集中删除该节点，因为这样会破坏本身存在的邻近关系，影响下一个regiongrowing过程。
    }
    else //这就直接开始逐点添加了吗？不太合理
    {  
      IndexPoints points;
      IndexPoints::iterator iter_point = tmpnode->valid_points.begin();
      while (iter_point != tmpnode->valid_points.end())
      {
        if (checkPointInPlane(iter_point->second))
        {
          points.emplace_back(*iter_point);
          // erase后，后面的元素往前移一位
          tmpnode->valid_points.erase(iter_point);
        }
        else
        {
          iter_point++;
        }
      }
      // 把某个节点的一部分点加进取，并且节点作为边界点，其邻近节点不再加入
      // cout<<"points size: "<<points.size()<<endl;
      if (points.empty())
      {
        continue;
      }
      
      addPoints2Plane(points);

      // if (points.size() == tmpnode->initial_size)
      if (points.size() == tmpnode->width*tmpnode->width)
      {
        // 将邻近节点加入
        for (auto & n : tmpnode->neighbors)
        {
          if (close_set.find(node2string(n)) == close_set.end())
          {
            p_queue.push(n);
          }
        }
      }
      
      close_set.insert(node2string(tmpnode));
      // nodesHadChecked.insert(tmpnode);
      // for (auto & iter_node : tmpnode->neighbors)
      // {
      //   iter_node->neighbors.erase(tmpnode);
      // }
      tmpnode->valid_points_size = tmpnode->valid_points.size();
      if (tmpnode->valid_points_size == 0)
      {
        // cout<<"need delete"<<endl;
        quatree::deleteNodeInQuatree(tmpnode);
        tmpnode = nullptr;
      }

      // 这里的逻辑不对，因为即使每个点都在里面，但是node的点比较少此时不能以node加入
      // 
      // if (points.size() == tmpnode->initial_size)
      // {
      //   addNode2Plane(tmpnode);
      //   nodesHadChecked.insert(tmpnode);
      //   // cv::imshow("contour image", contour_image);
      //   // cv::waitKey(0);
      // }
      // else
      // {
      // }
    }
  }
}

bool plane::checkInPlane(std::shared_ptr<quatree::node> p)
{
  assert(p != nullptr && "node is nullptr");
  // assert(p->is_plane && "node is not a plane, please do not add it in a plane");
  if(!p->is_plane)
    return false;
  if (valid_points_size == 0)
  {
    // LOG(INFO)<<"now it is an empty plane."<<std::endl;
    return true;
  }
  else
  {
    // 本来是有三个条件，但是第三个条件（合并后的mse）在add时计算更好，这样可以减少计算
    // LOG(INFO)<<"condition 1: "<<(std::abs(normal.dot(p->normal)) > param.quatree_merge_normal_dot_th)<<" condition 2: "<<(std::abs((center - p->center).dot(normal)) < param.merge_normal_distance)<<std::endl;
    // std::cout<<(std::abs(normal.dot(p->normal)) > param.quatree_merge_normal_dot_th)<<" condition 2: "<<(std::abs((center - p->center).dot(normal)) < param.merge_normal_distance)<<std::endl;
    if (std::abs(normal.dot(p->normal)) > param.quatree_merge_normal_dot_th && std::abs((center - p->center).dot(normal)) < param.merge_normal_distance)//阈值三个
    {
      // LOG(INFO)<<"node may be in this plane."<<std::endl;
      return true;
    }
    // 如果为false，需不需要将这个点在index中的指针置null
  }
  return false;
}

string plane::node2string(std::shared_ptr<quatree::node> n)
{
  return (std::to_string(n->start_rows_2d) + "_" + std::to_string(n->start_cols_2d));
}

bool plane::addNode2Plane(std::shared_ptr<quatree::node> p)
{
  // cout<<"add node to plane"<<endl;
  assert(p != nullptr && "node is nullptr");
  // assert(p->is_plane && "node is not a plane, please do not add it in a plane");
  if (valid_points_size == 0)
  {
    // LOG(INFO)<<"now the plan is empty, refresh plane parameter."<<std::endl;
    stats = p->stats;
    center = p->center;
    // J = p->J;
    normal = p->normal;
    mse = p->mse;
    valid_points_size = p->stats.N;
    // valid_points_size = p->valid_points_size;
    // std::cout<<"valid_points_size: "<<valid_points_size<<std::endl;
    // valid_points.insert(valid_points.end(), p->valid_points.begin(), p->valid_points.end());
    // 为了得到轮廓
    // cv::Rect 内第一个参数是：列，第二个参数是：行
    // 加上要
    cv::rectangle(contour_image, cv::Rect(p->start_cols, p->start_rows, p->width, p->width), 255, -1, 1, 0);//255白色

    // cv::imshow("debug_image", contour_image);
    cv::imwrite("/home/lichao/TCDS/src/pip_line/data/debug_image.png", contour_image);
    // cv::waitKey(0);

    // cv::Mat enlarged_img;
    // int scale_factor = 50;  // 放大倍数
    // cv::resize(contour_image, enlarged_img, cv::Size(), scale_factor, scale_factor, cv::INTER_LINEAR);
    // cv::imshow("initial", enlarged_img);
    // cv::waitKey(0);

    // point_index_2d p1(p->start_rows_2d, p->start_cols_2d);
    // point_index_2d p2(p->start_rows_2d + p->width_2d - 1, p->start_cols_2d);
    // point_index_2d p3(p->start_rows_2d + p->width_2d - 1, p->start_cols_2d + p->width_2d - 1);
    // point_index_2d p4(p->start_rows_2d, p->start_cols_2d + p->width_2d - 1);
    // cv::Rect(100, 80, 20, 40) 宽度方向起点 高度方向的起点 宽度方向的块度， 高度方向的高度
    // LOG(INFO)<<"rect start cols "<<p->start_cols_2d<<" start rows "<<p->start_rows_2d<<" width2d "<<p->width_2d<<std::endl;
    // cv::rectangle(index_image, cv::Rect(p->start_cols_2d, p->start_rows_2d, p->width_2d, p->width_2d), 255, -1, 1, 0);//255白色
    // cv::namedWindow("index image", cv::WINDOW_FREERATIO);
    // cv::imshow("index image", index_image);
#ifdef DEBUG
    for (auto & iter_rows : index_node_2d.index2d)
    {
      for(auto & iter_rows_cols : iter_rows)
      {
        std::cout<<iter_rows_cols<<" ";
      }
      std::cout<<std::endl;
    }
#endif
    // std::vector<std::vector<std::shared_ptr<quatree::node>>>::iterator iter_rows = index_node_2d.index2d.begin() + p->start_rows_2d;
    // for (size_t i = 0; i < p->width_2d; i++)
    // {
    //   std::for_each(iter_rows->begin() + p->start_cols_2d, iter_rows->begin() + p->start_cols_2d + p->width_2d, [](std::shared_ptr<quatree::node> & p){ p = nullptr;});
    //   iter_rows++;
    // }
#ifdef DEBUG
    for (auto & iter_rows : index_node_2d.index2d)
    {
      for(auto & iter_rows_cols : iter_rows)
      {
        cout<<iter_rows_cols<<" ";
      }
      cout<<std::endl;
    }
#endif
    patchs.emplace_back(p);
    // std::cout<<"11111111111"<<std::endl;
    for (auto & p_node : p->neighbors)
    {
      p_queue.push(p_node);
    }
    close_set.insert(node2string(p));
    // std::cout<<"222222222222"<<std::endl;
    // std::cout<<"erase 1"<<std::endl;
    // 这是不是可以删除，初次时肯定没有
    // neighbors.erase(p);
    // for (auto & iter_node : neighbors)
    // {
    //   // std::cout<<"erase 2"<<std::endl;
    //   iter_node->neighbors.erase(p);
    // }
    // LOG(INFO)<<"delete some info"<<endl;
    // cout<<"need delteddddd"<<endl;
    quatree::deleteNodeInQuatree(p);
    // LOG(INFO)<<"return "<<endl;
    return true;
  }
  // std::cout<<"not empty"<<std::endl;
  // 到这一步表示满足了法向量和距离要求
  Stats origin_stats = stats;
  stats.push(p->stats);
  // Eigen::Vector3f M = center * valid_points_size + p->center * p->valid_points_size;
  // size_t sum_points = valid_points_size + p->valid_points_size;
  // Eigen::Vector3f center_cand = M/sum_points;
  // Eigen::Matrix3f J_cand = J + p->J;
  // Eigen::Matrix3f S_cand = J_cand - M*center_cand.transpose();
  // Eigen::EigenSolver<Eigen::Matrix3f> es(S_cand);
  // Eigen::Matrix3f::Index b;
  // auto minEigenValue = es.eigenvalues().real().minCoeff(&b);
  // double eigenValuesSum = es.eigenvalues().real().sum();
  // double mse_cand = minEigenValue/sum_points;
  Eigen::Vector3f cand_center, cand_normal;
  float cand_mse, cand_curvature;
  stats.compute(cand_center, cand_normal, cand_mse, cand_curvature);
  if (cand_curvature < param.eigen_value_th && cand_mse < param.patch_mse_th)
  {
    // 表示可能会作为加入的平面，但还是需要确定哪个平面合并后的mse最小
    // LOG(INFO)<<"ADD NODE TO PLANE"<<std::endl;
    // center = center_cand;
    // J = J_cand;
    // normal = es.eigenvectors().real().col(b);
    // mse = mse_cand;
    // valid_points_size = sum_points;
    center = cand_center;
    normal = cand_normal;
    mse = cand_mse;
    curvature = cand_curvature;
    valid_points_size = stats.N;
    // valid_points.insert(valid_points.end(), p->valid_points.begin(), p->valid_points.end());
    // 加上要
    // 求轮廓时用到
    cv::rectangle(contour_image, cv::Rect(p->start_cols, p->start_rows, p->width, p->width), 255, -1, 1, 0);//255白色
    cv::imwrite("/home/lichao/TCDS/src/pip_line/data/debug_image.png", contour_image);
    // std::cout<<"erase 3"<<std::endl;
    // std::cout<<"has erase"<<std::endl;
    // std::cout<<"erase node: "<<p<<std::endl;
    // std::cout<<neighbors.size()<<std::endl;
    // for (auto & iter_node : neighbors)
    // {
    //   std::cout<<iter_node<<" ";
    // }
    // std::cout<<std::endl;
    // LOG(INFO)<<"add node "<<p;
    // for (auto & iter_node : neighbors)
    // {
    //   LOG(INFO)<<iter_node;
    // }
    close_set.insert(node2string(p));
    for (auto & iter_node : p->neighbors)
    {
      if (close_set.find(node2string(iter_node)) == close_set.end())// 没有找到
      {
        p_queue.push(iter_node);
      }
      
      // LOG(INFO)<<"iter node "<<iter_node;
      // iter_node->neighbors.erase(p);
      // // 虽然nodesHadChecked已经包含了某节点，但是当对该节点进行count时，任然为0，说明这个红黑树查找存在问题
      // if (nodesHadChecked.count(iter_node) == 0)
      // {
      //   // LOG(INFO)<<"add nei "<<iter_node;
      //   neighbors.insert(iter_node);
      // }
    }
    
    // for (auto & iter_node : neighbors)
    // {
    //   LOG(INFO)<<iter_node;
    // }
    // std::cout<<"111112"<<std::endl;
    // neighbors.insert(p->neighbors.begin(), p->neighbors.end());
    // for (auto & iter_node : neighbors)
    // {
    //   // std::cout<<"erase node: "<<p<<std::endl;
    //   // std::cout<<iter_node->neighbors.size()<<std::endl;
    //   // for (auto & iter_node : iter_node->neighbors)
    //   // {
    //   //   std::cout<<iter_node<<" ";
    //   // }
    //   // std::cout<<std::endl;
    //   // std::cout<<"erase 4"<<std::endl;
    //   iter_node->neighbors.erase(p);
    // }
    // std::cout<<"cjjjj"<<std::endl;
    // cv::namedWindow("image before", cv::WINDOW_FREERATIO);
    // cv::imshow("image before", index_image);
    // cv::waitKey();
    // 用于求边缘，黑白图
    // cv::rectangle(index_image, cv::Rect(p->start_cols_2d, p->start_rows_2d, p->width_2d, p->width_2d), 255, -1, 1, 0);
    // cv::namedWindow("image after", cv::WINDOW_FREERATIO);
    // cv::imshow("image after", index_image);
    // cv::waitKey();
    // 用于求邻近节点，都是节点的地址
    // std::vector<std::vector<std::shared_ptr<quatree::node>>>::iterator iter_rows = index_node_2d.index2d.begin() + p->start_rows_2d;
    // for (size_t i = 0; i < p->width_2d; i++)
    // {
    //   std::for_each(iter_rows->begin() + p->start_cols_2d, iter_rows->begin() + p->start_cols_2d + p->width_2d, [](std::shared_ptr<quatree::node> & p){ p = nullptr;});
    //   iter_rows++;
    // }
    patchs.emplace_back(p);// 这个有没有用？？？
    // LOG(INFO)<<"add to patchs"<<endl;
    // cout<<"need delete****"<<endl;
    quatree::deleteNodeInQuatree(p);
    // cout<<"need delete======="<<endl;
    // LOG(INFO)<<"RETURN TRUE"<<endl;
    return true;
  }
  else
  {
    stats = origin_stats;
    return false;// 如果为false，需不需要将这个点在index中的指针置null
  }
}

// 对求边缘和邻近节点的相关操作，适用于节点内的所有点都加入了plane
// bool plane::refreshIndexImage(std::shared_ptr<quatree::node> p)
// {
//   cv::rectangle(index_image, cv::Rect(p->start_cols_2d, p->start_rows_2d, p->width_2d, p->width_2d), 255, -1, 1, 0);
//   // cv::namedWindow("image after", cv::WINDOW_FREERATIO);
//   // cv::imshow("image after", index_image);
//   // cv::waitKey();
//   // 用于求邻近节点，都是节点的地址
//   std::vector<std::vector<std::shared_ptr<quatree::node>>>::iterator iter_rows = index_node_2d.index2d.begin() + p->start_rows_2d;
//   for (size_t i = 0; i < p->width_2d; i++)
//   {
//     std::for_each(iter_rows->begin() + p->start_cols_2d, iter_rows->begin() + p->start_cols_2d + p->width_2d, [](std::shared_ptr<quatree::node> & p){ p = nullptr;});
//     iter_rows++;
//   }
//   patchs.emplace_back(p);
//   return  true;
// }
// 所有相邻的节点，包括不为平面的节点
// void plane::getNeighborNode(std::priority_queue<std::shared_ptr<quatree::node>, std::vector<std::shared_ptr<quatree::node>>, quatree::compnode > & neighbors)
// {
//   LOG(INFO)<<"index image rows "<<index_image.rows<<" cols "<<index_image.cols<<std::endl;
//   // cv::namedWindow("raw image", cv::WINDOW_FREERATIO);
//   // cv::imshow("raw image", index_image);
//   // cv::waitKey();
//   // cv::Mat element = cv::getStructuringElement(0, cv::Size(3,3), cv::Point(1,1));
//   // cv::Mat output/*  = index_image */;
//   // cv::dilate(index_image, output, element);
//   // cv::namedWindow("dilate image", cv::WINDOW_FREERATIO);
//   // cv::imshow("dilate image", output);
//   // cv::waitKey();
//   std::vector<std::vector<cv::Point>> contours;
//   std::vector<cv::Vec4i> hierarchy;
//   // 这个寻找边缘的像素点，包括内边缘和外边缘
//   cv::findContours(index_image, contours, hierarchy, cv::RETR_LIST, cv::CHAIN_APPROX_NONE, cv::Point());
//   for (autoneighbors
//     for (auto & iter_point : iter_contour)
//     {
//       LOG(INFO)<<"POINT ID: "<<iter_point.y<<" "<<iter_point.x<<std::endl;
//       // 找到周围8个点
//       for (int i = -1; i <= 1; i++)
//       {
//         for (int j = -1; j <= 1; j++)
//         {
//           if (i == 0 && j == 0)
//           {
//             continue;
//           }
//           if (iter_point.y + j >= 0 && iter_point.y + j < index_node_2d.rows)
//           {
//             if (iter_point.x + i >= 0 && iter_point.x + i < index_node_2d.cols)
//             {
//               LOG(INFO)<<"index: "<<iter_point.y + j<< " "<<iter_point.x + i<<std::endl;
//               std::shared_ptr<quatree::node> tmpnode = index_node_2d.getNodeByIndex(iter_point.y + j, iter_point.x + i);
//               if (tmpnode)// 可以每次将用过的patch置nullptr
//               {
//                 LOG(INFO)<<"PUSH NODE: "<<tmpnode<<std::endl;
//                 neighbors.push(tmpnode);
//               }
//             }
//           }
//         }
//       }
//     }
//   }
// }

bool plane::checkPointInPlane(Eigen::Vector3f & p)
{
  // std::cout<<std::abs(normal.dot(center - p))<<" "<<param.merge_normal_distance<<std::endl;
  return std::abs(normal.dot(center - p)) < param.merge_normal_distance;// canshu
}

/* bool plane::addPoint2Plane(Eigen::Vector3f & p)
{
  // Eigen::Vector3f M = center * valid_points_size + p;
  // Eigen::Vector3f center_cand = M/(valid_points_size + 1);
  // Eigen::Matrix3f J_cand = J + p * p.transpose();
  // Eigen::Matrix3f S_cand = J_cand - M * center_cand.transpose();
  // Eigen::EigenSolver<Eigen::Matrix3f> es(S_cand);
  // Eigen::Matrix3f::Index b;
  // auto minEigenValue = es.eigenvalues().real().minCoeff(&b);
  // double eigenValuesSum = es.eigenvalues().real().sum();
  // double mse_cand = minEigenValue/(valid_points_size + 1);
  Stats origin_stats = stats;
  stats.push(p);
  Eigen::Vector3f cand_center, cand_normal;
  float cand_mse, cand_curvature;
  stats.compute(cand_center, cand_normal, cand_mse, cand_curvature);
  if (cand_curvature >= param.patch_mse_th || cand_mse >= param.patch_mse_th)
  {
    stats = origin_stats;
    return false;
  }
  else
  {
    center = cand_center; normal = cand_normal; mse = cand_mse; curvature = cand_curvature;
    valid_points.emplace_back(p);
    valid_points_size = stats.N;
    return true;
  }
  // if (mse_cand <= param.patch_mse_th)
  // {
  //   center = center_cand;
  //   J = J_cand;
  //   normal = es.eigenvectors().real().col(b);
  //   mse = mse_cand;
  //   valid_points.emplace_back(p);
  //   valid_points_size ++;
  //   return true;
  // }
  // return false;
} */
// 遇到问题，一次性加入多个点会不会不满足mse条件
bool plane::addPoints2Plane(IndexPoints& ps)
{
  Stats origin_stats = stats;
  stats.push(ps);
  Eigen::Vector3f cand_center, cand_normal;
  float cand_mse, cand_curvature;
  stats.compute(cand_center, cand_normal, cand_mse, cand_curvature);
  
  // size_t ps_num = ps.size();
  // Eigen::Vector3f ps_center = Eigen::Vector3f::Zero();
  // for (auto & iter_point : ps)
  // {
  //   ps_center += iter_point;
  // }
  // // ps_center /= ps_num;
  // Eigen::Vector3f M = center * valid_points_size + ps_center;
  // Eigen::Vector3f center_cand = M/(valid_points_size + ps_num);
  // ps_center /= ps_num;
  // Eigen::Matrix3f ps_J = Eigen::Matrix3f::Zero();
  // for (auto & iter_point : ps)
  // {
  //   ps_J += (iter_point) * (iter_point).transpose();
  // }
  // Eigen::Matrix3f J_cand = J + ps_J;
  // Eigen::Matrix3f S_cand = J_cand - M * center_cand.transpose();
  // Eigen::EigenSolver<Eigen::Matrix3f> es(S_cand);
  // Eigen::Matrix3f::Index b;
  // auto minEigenValue = es.eigenvalues().real().minCoeff(&b);
  // double eigenValuesSum = es.eigenvalues().real().sum();
  // double mse_cand = minEigenValue/(valid_points_size + ps_num);

  if (cand_curvature >= param.patch_mse_th || cand_mse >= param.patch_mse_th)
  {
    // center = center_cand;
    // J = J_cand;
    // normal = es.eigenvectors().real().col(b);
    // mse = mse_cand;
    // valid_points_size += ps_num;
    stats = origin_stats;
    return false;
  }
  else
  {
    center = cand_center; normal = cand_normal; mse = cand_mse; curvature = cand_curvature;
    // 加上要
    for (auto & indexpoint : ps)
    {
      // inline uchar &cv::Mat::at<uchar>(int row, int col)
      contour_image.at<uchar>(indexpoint.first.first, indexpoint.first.second) = 255;
    }
    cv::imwrite("/home/lichao/TCDS/src/pip_line/data/debug_image.png", contour_image);
    // cv::imshow("debug_image", contour_image);
    // cv::waitKey(0);

    // cv::Mat enlarged_img;
    // int scale_factor = 50;  // 放大倍数
    // cv::resize(contour_image, enlarged_img, cv::Size(), scale_factor, scale_factor, cv::INTER_LINEAR);
    // cv::imshow("result", enlarged_img);
    // cv::waitKey(0);

    // valid_points.insert(valid_points.end(), ps.begin(), ps.end());
    valid_points_size = stats.N;
    return true;
  }
}

// void plane::setIndexImage(std::shared_ptr<quatree::node> p)
// {
//   cv::rectangle(index_image, cv::Rect(p->start_cols_2d, p->start_rows_2d, p->width_2d, p->width_2d), 255, -1, 1, 0);//255白色
//   std::vector<std::vector<std::shared_ptr<quatree::node>>>::iterator iter_rows = index_node_2d.index2d.begin() + p->start_rows_2d;
//   for (size_t i = 0; i < p->width_2d; i++)
//   {
//     std::for_each(iter_rows->begin() + p->start_cols_2d, iter_rows->begin() + p->start_cols_2d + p->width_2d, [](std::shared_ptr<quatree::node> & p){ p = nullptr;});
//     iter_rows++;
//   }
// }
// void plane::showImage()
// {
//   for (size_t i = 0; i < index_image.rows; i++)
//   {
//     for (size_t j = 0; j < index_image.cols; j++)
//     {
//       std::cout<<index_image.at<uchar>(j, i)<<std::endl;
//     }
//   }
// }

void plane::showInfo()
{
  LOG(INFO)<<"patchs: ";
  for (auto & iter_node : patchs)
  {
    LOG(INFO)<<iter_node;
  }

  LOG(INFO)<<"nodes had checked: ";
  // for (auto & iter_node : nodesHadChecked)
  // {
  //   LOG(INFO)<<iter_node;
  // }
  // LOG(INFO)<<"neighbors: ";
  // for (auto & iter_node : neighbors)
  // {
  //   LOG(INFO)<<iter_node;
  // }
}


bool plane::isEmpty()
{
  return valid_points_size == 0;
}

// bool plane::nodeHasInPlane(std::shared_ptr<quatree::node> p)
// {
//   for (auto & iter_node : patchs)
//   {
//     if (iter_node == p)
//     {
//       return true;
//     }
//   }
//   return false;
// }


plane_info plane::calculatePlaneParamNofilter()
{
  std::vector<std::vector<cv::Point>> contours;
  std::vector<cv::Vec4i> hierarchy;
  cv::findContours(contour_image, contours, hierarchy, cv::RETR_CCOMP, cv::CHAIN_APPROX_NONE, cv::Point());
  

  // 这里需要对检测到的轮廓进行填充，因为可能会出现小空洞的情况，这是由于点云采集质量决定的，如黑色阴影部分，其反光效果差
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
          uchar *output_data = contour_image.ptr<uchar>(i);
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
  cv::findContours(contour_image, contours, hierarchy, cv::RETR_CCOMP, cv::CHAIN_APPROX_NONE, cv::Point());

#ifdef SHOWIMAGE
	cv::Mat imageContours = cv::Mat::zeros(contour_image.size(),CV_8UC1);
	cv::Mat Contours = cv::Mat::zeros(contour_image.size(),CV_8UC1);  //绘制
	for(int i = 0;i < contours.size();i++)
	{
		//contours[i]代表的是第i个轮廓，contours[i].size()代表的是第i个轮廓上所有的像素点数
		for(int j = 0;j < contours[i].size();j++) 
		{
			//绘制出contours向量内所有的像素点
			cv::Point P = cv::Point(contours[i][j].x,contours[i][j].y);
			Contours.at<uchar>(P)=255;
		}
 
		//输出hierarchy向量内容
		char ch[256];
		sprintf(ch,"%d",i);
		string str=ch;
		cout<<"向量hierarchy的第" <<str<<" 个元素内容为："<<endl<<hierarchy[i]<<endl<<endl;
 
		//绘制轮廓
		drawContours(imageContours,contours,i,cv::Scalar(255),1,8,hierarchy);
	}
	cv::imshow("Contours Image",imageContours); //轮廓
	cv::imshow("Point of Contours",Contours);   //向量contours内保存的所有轮廓点集
	cv::waitKey(0);
#endif
  vector<vector<cv::Point>> approx_contours(contours.size());
  vector<vector<cv::Point>>::iterator approx_contour_iter = approx_contours.begin();
  for (auto & coutour : contours)
  {
    cv::approxPolyDP(coutour, *approx_contour_iter, 5, true);
    approx_contour_iter++;
  }
#ifdef SHOWIMAGE
  imageContours = cv::Mat::zeros(contour_image.size(),CV_8UC1);
	Contours = cv::Mat::zeros(contour_image.size(),CV_8UC1);  
  for(int i = 0;i < approx_contours.size();i++)
	{
		//contours[i]代表的是第i个轮廓，contours[i].size()代表的是第i个轮廓上所有的像素点数
		for(int j = 0; j < approx_contours[i].size(); j++) 
		{
			//绘制出contours向量内所有的像素点
			cv::Point P = cv::Point(approx_contours[i][j].x, approx_contours[i][j].y);
			Contours.at<uchar>(P) = 255;
		}
		//输出hierarchy向量内容
		char ch[256];
		sprintf(ch,"%d",i);
		string str=ch;
		cout<<"向量hierarchy的第" <<str<<" 个元素内容为："<<endl<<hierarchy[i]<<endl<<endl;
 
		//绘制轮廓
		drawContours(imageContours, approx_contours, i, cv::Scalar(255), 1, 8,hierarchy);
	}
	cv::imshow("approx Contours Image", imageContours); //轮廓
	cv::imshow("approx Point of Contours", Contours);   //向量contours内保存的所有轮廓点集
	cv::waitKey(0);
#endif
  // 至此，得到了轮廓
  // 投影到三维平面上
  // vector<vector<Eigen::Vector3f>> ;
  // std::vector< std::vector < Eigen::Vector3f > > Plane_coutours(contours.size());
  // std::cout<<"12345"<<std::endl;
  for (auto & contour : approx_contours)
  {
    vector<Eigen::Vector3f> contour_points;
    contour_points.reserve(contour.size());
    for (auto & point_pixel_index : contour)
    {
      // std::cout<<point_pixel_index.y<<" "<<point_pixel_index.x<<std::endl;
      Eigen::Vector3f tmppoint = raw_points.getIndexPoint(point_pixel_index.y, point_pixel_index.x);
      float d = (tmppoint - center).dot(normal);
      contour_points.emplace_back(tmppoint - d * normal);
    }
    Plane_coutours.emplace_back(contour_points);
  }
  // std::cout<<"okkk"<<std::endl;
  plane_info return_plane;
  return_plane.center = center;
  return_plane.contours = Plane_coutours;
  return_plane.hierarchy.reserve(hierarchy.size());
  for (auto & hierarchy_elem : hierarchy)
  {
    std::array<int, 4> element;
    element.at(0) = hierarchy_elem(0);
    element.at(1) = hierarchy_elem(1);
    element.at(2) = hierarchy_elem(2);
    element.at(3) = hierarchy_elem(3);
    return_plane.hierarchy.emplace_back(element);
  }
  return_plane.normal = normal;
  return return_plane;
  // 分解成二维平面
  // Eigen::Vector3d z_axid = normal;
  // Eigen::Vector3d x_point;
  // for (auto & point : 3DPlane_coutours.begin())
  // {
  //   if ((point - center).norm() > 0.2)
  //   {
  //     x_point = point;
  //     break;
  //   }
  // }
  // Eigen::Vector3d x_axid = (x_point - center).normalized();
  // Eigen::Vector3d y_axid = (normal.cross(x_axid)).normalized();
  // Eigen::Matrix3d rotation2W;
  // rotation2W<<x_axid.dot(Eigen::Vector3d::UnitX()), y_axid.dot(Eigen::Vector3d::UnitX()), 
  //             z_axid.dot(Eigen::Vector3d::UnitX()), x_axid.dot(Eigen::Vector3d::UnitY()),
  //             y_axid.dot(Eigen::Vector3d::UnitY()), z_axid.dot(Eigen::Vector3d::UnitY()),
  //             x_axid.dot(Eigen::Vector3d::UnitZ()), y_axid.dot(Eigen::Vector3d::UnitZ()),
  //             z_axid.dot(Eigen::Vector3d::UnitZ());
  // Eigen::Isometry3d T1=Eigen::Isometry3d::Identity();
  // T1.rotate (rotation2W);
  // T1.pretranslate (center);
  // std::vector<dt::Vector2<double>> points2D;
  // 将多边形用多个凸多边形表示
  // 投影到平面上
  // std::vector<Eigen::Vector3f> plane_points;
  // for (auto & point : valid_points)
  // {
  //   float d = (point - center).dot(normal);
  //   plane_points.emplace_back(point - d * (normal));
  // }
  // 获取平面的轮廓点
  // Eigen::Vector3d z_axid = normal;
  // Eigen::Vector3d x_point;
  // for (auto & iter_plane_point : plane_points)
  // {
  //   if ((iter_plane_point - center).norm() > 0.2)
  //   {
  //     x_point = iter_plane_point;
  //     break;
  //   }
  // }
  // Eigen::Vector3d x_axid = (x_point - center).normalized();
  // Eigen::Vector3d y_axid = (normal.cross(x_axid)).normalized();
  // // 从定义的平面坐标系到世界坐标系
  // Eigen::Matrix3d rotation2W;
  // rotation2W<<x_axid.dot(Eigen::Vector3d::UnitX()), y_axid.dot(Eigen::Vector3d::UnitX()), 
  //             z_axid.dot(Eigen::Vector3d::UnitX()), x_axid.dot(Eigen::Vector3d::UnitY()),
  //             y_axid.dot(Eigen::Vector3d::UnitY()), z_axid.dot(Eigen::Vector3d::UnitY()),
  //             x_axid.dot(Eigen::Vector3d::UnitZ()), y_axid.dot(Eigen::Vector3d::UnitZ()),
  //             z_axid.dot(Eigen::Vector3d::UnitZ());
  // Eigen::Isometry3d T1=Eigen::Isometry3d::Identity();
  // T1.rotate (rotation2W);
  // T1.pretranslate (center);
  // std::vector<dt::Vector2<double>> points2D;
	// for (auto & iter : output_vector)
	// {
	// 	points.emplace_back(dt::Vector2<double>{iter(0), iter(1)});
	// }
  // std::vector<cv::Point2f> hull;
  // for (auto & iter:(cloud_hull))
  // {
  //     Eigen::Vector3d new_p = T1.inverse()*Eigen::Vector3d(iter.x, iter.y, iter.z);
  //     hull.emplace_back(cv::Point2f(new_p(0), new_p(1)));
  // }
  // LShapedFIT lshaped;
  // cv::RotatedRect rr = lshaped.FitBox(&hull);
  // std::vector<cv::Point2f> vertices = lshaped.getRectVertex();
  // vector<Eigen::Vector3d> edgePoints;
  // for (auto & iter: vertices)
  // {
  //   Eigen::Vector3d point(iter.x, iter.y, 0.0);
  //   edgePoints.emplace_back(T1*point);
  // }
  // return edgePoints;
  // 矩形拟合（看需要不）
}

plane_info plane::calculatePlaneParammedianBlur()
{
  cv::Mat median_filter_image;
  cv::medianBlur(contour_image, median_filter_image, 5);
  std::vector<std::vector<cv::Point>> contours;
  std::vector<cv::Vec4i> hierarchy;
  cv::findContours(median_filter_image, contours, hierarchy, cv::RETR_CCOMP, cv::CHAIN_APPROX_NONE, cv::Point());
  
  // 这里需要对检测到的轮廓进行填充，因为可能会出现小空洞的情况，这是由于点云采集质量决定的，如黑色阴影部分，其反光效果差
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
          uchar *output_data = median_filter_image.ptr<uchar>(i);
          for (int j = rect.x; j < rect.x + rect.width; j++) 
          {
            output_data[j] = 255;
          }
        }
      }
      itc++;
    }
  }
  cv::findContours(median_filter_image, contours, hierarchy, cv::RETR_CCOMP, cv::CHAIN_APPROX_NONE, cv::Point());

#ifdef SHOWIMAGE
	cv::Mat imageContours = cv::Mat::zeros(median_filter_image.size(),CV_8UC1);
	cv::Mat Contours = cv::Mat::zeros(median_filter_image.size(),CV_8UC1);  //绘制
	for(int i = 0;i < contours.size();i++)
	{
		//contours[i]代表的是第i个轮廓，contours[i].size()代表的是第i个轮廓上所有的像素点数
		for(int j = 0;j < contours[i].size();j++) 
		{
			//绘制出contours向量内所有的像素点
			cv::Point P = cv::Point(contours[i][j].x,contours[i][j].y);
			Contours.at<uchar>(P)=255;
		}
 
		//输出hierarchy向量内容
		char ch[256];
		sprintf(ch,"%d",i);
		string str=ch;
		cout<<"向量hierarchy的第" <<str<<" 个元素内容为："<<endl<<hierarchy[i]<<endl<<endl;
 
		//绘制轮廓
		drawContours(imageContours,contours,i,cv::Scalar(255),1,8,hierarchy);
	}
	cv::imshow("median Contours Image",imageContours); //轮廓
	cv::imshow("median Point of Contours",Contours);   //向量contours内保存的所有轮廓点集
	cv::waitKey(0);
#endif
  vector<vector<cv::Point>> approx_contours(contours.size());
  vector<vector<cv::Point>>::iterator approx_contour_iter = approx_contours.begin();
  for (auto & coutour : contours)
  {
    cv::approxPolyDP(coutour, *approx_contour_iter, 5, true);
    approx_contour_iter++;
  }
#ifdef SHOWIMAGE
  imageContours = cv::Mat::zeros(median_filter_image.size(),CV_8UC1);
	Contours = cv::Mat::zeros(median_filter_image.size(),CV_8UC1);  
  for(int i = 0;i < approx_contours.size();i++)
	{
		//contours[i]代表的是第i个轮廓，contours[i].size()代表的是第i个轮廓上所有的像素点数
		for(int j = 0; j < approx_contours[i].size(); j++) 
		{
			//绘制出contours向量内所有的像素点
			cv::Point P = cv::Point(approx_contours[i][j].x, approx_contours[i][j].y);
			Contours.at<uchar>(P) = 255;
		}
		//输出hierarchy向量内容
		char ch[256];
		sprintf(ch,"%d",i);
		string str=ch;
		cout<<"向量hierarchy的第" <<str<<" 个元素内容为："<<endl<<hierarchy[i]<<endl<<endl;
 
		//绘制轮廓
		drawContours(imageContours, approx_contours, i, cv::Scalar(255), 1, 8,hierarchy);
	}
	cv::imshow("median approx Contours Image", imageContours); //轮廓
	cv::imshow("median approx Point of Contours", Contours);   //向量contours内保存的所有轮廓点集
	cv::waitKey(0);
#endif
  for (auto & contour : approx_contours)
  {
    vector<Eigen::Vector3f> contour_points;
    contour_points.reserve(contour.size());
    for (auto & point_pixel_index : contour)
    {
      // std::cout<<point_pixel_index.y<<" "<<point_pixel_index.x<<std::endl;
      Eigen::Vector3f tmppoint = raw_points.getIndexPoint(point_pixel_index.y, point_pixel_index.x);
      float d = (tmppoint - center).dot(normal);
      contour_points.emplace_back(tmppoint - d * normal);
    }
    Plane_coutours.emplace_back(contour_points);
  }
  // std::cout<<"okkk"<<std::endl;
  plane_info return_plane;
  return_plane.center = center;
  return_plane.contours = Plane_coutours;
  return_plane.hierarchy.reserve(hierarchy.size());
  for (auto & hierarchy_elem : hierarchy)
  {
    std::array<int, 4> element;
    element.at(0) = hierarchy_elem(0);
    element.at(1) = hierarchy_elem(1);
    element.at(2) = hierarchy_elem(2);
    element.at(3) = hierarchy_elem(3);
    return_plane.hierarchy.emplace_back(element);
  }
  return_plane.normal = normal;
  return return_plane;
}

plane_info plane::calculatePlaneParamtestGaussianBlur()
{
  cv::Mat Gaussian_filter_image;
  cv::GaussianBlur(contour_image, Gaussian_filter_image, cv::Size(5, 5), 1, 1);
  std::vector<std::vector<cv::Point>> contours;
  std::vector<cv::Vec4i> hierarchy;
  cv::findContours(Gaussian_filter_image, contours, hierarchy, cv::RETR_CCOMP, cv::CHAIN_APPROX_NONE, cv::Point());
  
  // 这里需要对检测到的轮廓进行填充，因为可能会出现小空洞的情况，这是由于点云采集质量决定的，如黑色阴影部分，其反光效果差
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
          uchar *output_data = Gaussian_filter_image.ptr<uchar>(i);
          for (int j = rect.x; j < rect.x + rect.width; j++) 
          {
            output_data[j] = 255;
          }
        }
      }
      itc++;
    }
  }
  cv::findContours(Gaussian_filter_image, contours, hierarchy, cv::RETR_CCOMP, cv::CHAIN_APPROX_NONE, cv::Point());
  // std::cout<<"contours size "<<contours.size()<<endl;
#ifdef SHOWIMAGE
	// cv::Mat imageContours = cv::Mat::zeros(Gaussian_filter_image.size(),CV_8UC1);
	// cv::Mat Contours = cv::Mat::zeros(Gaussian_filter_image.size(),CV_8UC1);  //绘制
	// for(int i = 0;i < contours.size();i++)
	// {
	// 	//contours[i]代表的是第i个轮廓，contours[i].size()代表的是第i个轮廓上所有的像素点数
	// 	for(int j = 0;j < contours[i].size();j++) 
	// 	{
	// 		//绘制出contours向量内所有的像素点
	// 		cv::Point P = cv::Point(contours[i][j].x,contours[i][j].y);
	// 		Contours.at<uchar>(P)=255;
	// 	}
 
	// 	//输出hierarchy向量内容
	// 	char ch[256];
	// 	sprintf(ch,"%d",i);
	// 	string str=ch;
	// 	cout<<"向量hierarchy的第" <<str<<" 个元素内容为："<<endl<<hierarchy[i]<<endl<<endl;

	// 	//绘制轮廓
	// 	drawContours(imageContours,contours,i,cv::Scalar(255),1,8,hierarchy);
	// }
	// cv::imshow("Gaussian Contours Image",imageContours); //轮廓
	// cv::imshow("Gaussian Point of Contours",Contours);   //向量contours内保存的所有轮廓点集
	// cv::waitKey(0);
#endif
  vector<vector<cv::Point>> approx_contours(contours.size());
  vector<vector<cv::Point>>::iterator approx_contour_iter = approx_contours.begin();
  for (auto & coutour : contours)
  {
    cv::approxPolyDP(coutour, *approx_contour_iter, 5, true);
    approx_contour_iter++;
  }
  // 显示近似后的图像
  // cv::drawContours(segmented_result, contours, -1, CV_RGB(r,g,b), -1);
#ifdef SHOWIMAGE
  // imageContours = cv::Mat::zeros(Gaussian_filter_image.size(),CV_8UC1);
	// Contours = cv::Mat::zeros(Gaussian_filter_image.size(),CV_8UC1);  
  // for(int i = 0;i < approx_contours.size();i++)
	// {
	// 	//contours[i]代表的是第i个轮廓，contours[i].size()代表的是第i个轮廓上所有的像素点数
	// 	for(int j = 0; j < approx_contours[i].size(); j++) 
	// 	{
	// 		//绘制出contours向量内所有的像素点
	// 		cv::Point P = cv::Point(approx_contours[i][j].x, approx_contours[i][j].y);
	// 		Contours.at<uchar>(P) = 255;
	// 	}
	// 	//输出hierarchy向量内容
	// 	char ch[256];
	// 	sprintf(ch,"%d",i);
	// 	string str=ch;
	// 	cout<<"向量hierarchy的第" <<str<<" 个元素内容为："<<endl<<hierarchy[i]<<endl<<endl;
 
	// 	//绘制轮廓
	// 	drawContours(imageContours, approx_contours, i, cv::Scalar(255), 1, 8,hierarchy);
	// }
	// cv::imshow("Gaussian approx Contours Image", imageContours); //轮廓
	// cv::imshow("Gaussian approx Point of Contours", Contours);   //向量contours内保存的所有轮廓点集
	// cv::waitKey(0);
#endif

  // 记录一个这些轮廓点组成的点云，检查点云质量，因为上述都是基于像素的，不是基于实际点云坐标的，光线或者成像的影响，可能导致点云坐标噪声很大
  // 发现实际点云中有很多坐标为（0，0，0）的点，确实是由于深度图成像质量引起的

  /*
  **************************这个地方需要根据实际参数更改*******************************
  */
  for (auto & contour : approx_contours)
  {
    vector<Eigen::Vector3f> contour_points;
    contour_points.reserve(contour.size());
    for (auto & point_pixel_index : contour)
    {
      // std::cout<<point_pixel_index.y<<" "<<point_pixel_index.x<<std::endl;
      const double kFx = 456.67578125;
      const double kFy = 457.3671875;
      const double kCx = 310.76171875;
      const double kCy = 246.896484375;


      // 注意这个地方的参数需要根据你截取的上下左右来更新，非常重要，否则得到的多边形会不正常
      float div_z = normal(0) * (point_pixel_index.x + 50 - kCx) / kFx + normal(1) * (point_pixel_index.y + 100 - kCy) / kFy + normal(2);
      float z = (normal.dot(center)) / div_z;      
      float x = ((float)point_pixel_index.x + 50 - kCx) * z / kFx;
      float y = ((float)point_pixel_index.y + 100- kCy) * z / kFy;
      // Eigen::Vector3f tmppoint = raw_points.getIndexPoint(point_pixel_index.y, point_pixel_index.x);
      Eigen::Vector3f tmppoint(x, y, z);
      // std::cout<<"raw point data "<<tmppoint.transpose()<<std::endl;
      // std::cout<<"raw point data pixel "<<raw_points.getIndexPoint(point_pixel_index.y, point_pixel_index.x).transpose()<<std::endl;
      contour_points.emplace_back(tmppoint);
      // float d = (tmppoint - center).dot(normal);
      // contour_points.emplace_back(tmppoint - d * normal);
    }
    // 画出轮廓图，检查一下
    // std::cout<<"contour data: "<<std::endl;
    // std::cout<<contour_points.size()<<std::endl;
    // vector<float> x, y;
    // for (auto & point_3d : contour_points)
    // {
    //   std::cout<<point_3d.transpose()<<std::endl;
    //   x.emplace_back(point_3d.x());
    //   y.emplace_back(point_3d.y());
    //   // plt::plot(x, y);
    //   // plt::show();
    // }
    // x.emplace_back(*x.begin());
    // y.emplace_back(*y.begin());

    // plt::plot(x, y);
    // plt::show();

    Plane_coutours.emplace_back(contour_points);
  }
  // std::cout<<"okkk"<<std::endl;
  plane_info return_plane;
  return_plane.center = center;
  return_plane.contours = Plane_coutours;
  return_plane.hierarchy.reserve(hierarchy.size());
  for (auto & hierarchy_elem : hierarchy)
  {
    std::array<int, 4> element;
    element.at(0) = hierarchy_elem(0);
    element.at(1) = hierarchy_elem(1);
    element.at(2) = hierarchy_elem(2);
    element.at(3) = hierarchy_elem(3);
    return_plane.hierarchy.emplace_back(element);
  }
  return_plane.normal = normal;
  return return_plane;
}
