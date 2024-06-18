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
#ifndef _PLANE_H_
#define _PLANE_H_
#include <Eigen/Core>
#include <vector>
#include <limits.h>
#include <glog/logging.h>
#include <assert.h>
#include <cfloat>
#include <plane_detection/quadtree_new.h>
#include <opencv2/highgui/highgui.hpp>
#include "plane_detection/type.h"
#include <queue>

#define GRAPH


class plane
{
public:
  Stats stats;
  orginazed_points raw_points;
  parameter param;
  float curvature;
  Eigen::Vector3f center = Eigen::Vector3f::Zero();
  Eigen::Matrix3f J = Eigen::Matrix3f::Zero();
  Eigen::Vector3f normal = Eigen::Vector3f::Zero();
  float mse = FLT_MAX;
  size_t valid_points_size = 0;
  std::vector<quatree::node*> patchs;
  // mat 第一个参数是宽度， 第二个参数是高度
  cv::Mat contour_image;
  std::vector< std::vector < Eigen::Vector3f > > Plane_coutours;
  // IndexPoints valid_points;
  // index2D index_node_2d;
  std::pair<std::vector<Eigen::Vector3f>, Eigen::Vector3f> plane_params;
  std::set<quatree::node*, quatree::compnode> neighbors;
  std::set<quatree::node*, quatree::compnode> nodesHadChecked;
  // void deleteNodeInQuatree(quatree::node* p);
public:
  plane(size_t x, size_t y, /* index2D & index_node_2d_ */quatree::node* seed, orginazed_points & raw_points_, parameter & param_);
  void regionGrowing();
  // void getNeighborNode(std::priority_queue<quatree::node*, std::vector<quatree::node*>, quatree::compnode > & neighbors);
  bool checkInPlane(quatree::node* p);
  bool addNode2Plane(quatree::node* p);
  // bool refreshIndexImage(quatree::node* p);
  bool checkPointInPlane(Eigen::Vector3f & p);
  // bool addPoint2Plane(Eigen::Vector3f & p);
  bool addPoints2Plane(IndexPoints& ps);
  // void setIndexImage(quatree::node* p);
  // void showImage();
  void showInfo();
  bool isEmpty();
  plane_info calculatePlaneParamNofilter();
  plane_info calculatePlaneParammedianBlur();
  plane_info calculatePlaneParamtestGaussianBlur();// 发现高斯平滑的效果更好
  // bool nodeHasInPlane(quatree::node* p);
};


#endif