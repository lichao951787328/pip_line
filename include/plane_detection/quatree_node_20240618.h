/*
 * @description: 
 * @param : 
 * @return: 
 */

#ifndef _QUATREE_NODE_H_
#define _QUATREE_NODE_H_
#include <Eigen/Core>
#include <float.h>
#include <array>
#include <set>
#include <plane_detection/node_stats.h>
// #include <plane_detection/type.h>
#include <plane_detection/static_type.h>
namespace quatree
{
  class node;

// 这是在创建邻近队列时使用，保证邻近队列的优先元素为大平面元素
struct compnode
{
  bool operator() (std::shared_ptr<node> a, std::shared_ptr<node> b) const;
};

// 这是在区域增长阶段使用，保证在区域增长的优先队列中大平面在前面
struct reverseComnode
{
  compnode baseCom;
  bool operator()(std::shared_ptr<node> a, std::shared_ptr<node> b) const;
};

// typedef std::shared_ptr<node> NodePtr;
class node : public std::enable_shared_from_this<node>
{
public:
  static size_t data_width;
  static size_t data_height;
  static size_t quatree_width;
  static Eigen::Vector3f check_normal;

  static orginazed_points raw_points;
  static parameter param;
  // 加一个彩色图，根据高度值来赋色，从而看看哪个地方会出现缺口的问题
  // for debug
  static cv::Mat debug_color_image;

  static std::shared_ptr<node> create(size_t start_rows_, size_t start_cols_, size_t width_, size_t start_rows_2d_, size_t start_cols_2d_, size_t width_2d_, size_t depth_, std::shared_ptr<node> parent_);
public:
  size_t start_rows, start_cols, width, depth;
  size_t start_rows_2d, start_cols_2d, width_2d;
  Stats stats;
  float curvature = 0;
  bool is_plane = false;
  bool is_validnode = false;
  bool is_leafnode = false;
  float mse = FLT_MAX;
  size_t initial_size = 0;

  // 改成vector
  std::set<std::shared_ptr<node>, compnode> neighbors;
  // std::set<node*, compnode> neighbors;
  // uint32_t binary_code = 0;

  // 需要加一个点的mse的最大值，以确定是否满足条件，可能有几个点不是属于平面，但平均值很小。这个方法可不是用特征值间的大小来进行确定，不然遍历计算会增加计算量

  Eigen::Vector3f center = Eigen::Vector3f::Zero();
  Eigen::Vector3f normal = Eigen::Vector3f::Zero();
  Eigen::Matrix3f J = Eigen::Matrix3f::Zero();
  size_t valid_points_size = 0;
  // std::vector<Eigen::Vector3f> valid_points;
  IndexPoints valid_points;
  // row_index, col_index
  std::vector<std::pair<size_t, size_t>> valid_points_index;
public:
  /**
   * @description: initial node
   * @param {size_t} start_rows_
   * @param {size_t} start_cols_
   * @param {size_t} width_
   * @param {size_t} depth_
   * @param {node*} parent_
   * @return {*}
   */
  node(size_t start_rows_, size_t start_cols_, size_t width_, size_t start_rows_2d_, size_t start_cols_2d_, size_t width_2d_, size_t depth_, /* uint32_t binary_code_,  */std::shared_ptr<node> parent_);

  void initialize();
  void resetNode();
  void createChildren();
  static void setStaticMember(size_t width_, size_t height_, size_t quatree_width_, Eigen::Matrix4f & ROBOTWORLD_T_CAMERA, orginazed_points & raw_points_, parameter & param_);
  ~node();
  std::array<std::shared_ptr<node>, 4> children = {nullptr, nullptr, nullptr, nullptr};
  std::shared_ptr<node> parent = nullptr;
  // void showNodeFrame();
  // std::shared_ptr<node> create(size_t start_rows_, size_t start_cols_, size_t width_, size_t start_rows_2d_, size_t start_cols_2d_, size_t width_2d_, size_t depth_, std::shared_ptr<node> parent_);
};

// typedef std::shared_ptr<node> NodePtr;

void deleteNodeInQuatree(std::shared_ptr<node> p);


}

#endif