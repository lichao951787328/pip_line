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

#ifndef _QUATREE_NODE_H_
#define _QUATREE_NODE_H_
#include <Eigen/Core>
#include <float.h>
#include <array>
#include <set>
#include <plane_detection/node_stats.h>
namespace quatree
{
  class node;
struct compnode
{
  bool operator() (node const * const &  a, node const * const & b) const;
};

class node
{
public:
  static size_t data_width;
  static size_t data_height;
  static size_t quatree_width;
  static Eigen::Vector3f check_normal;
public:
  size_t start_rows, start_cols, width, depth;
  size_t start_rows_2d, start_cols_2d, width_2d;
  Stats stats;
  float curvature = 0;
  bool is_plane = false;
  bool is_validnode = false;
  bool is_leafnode = false;
  float mse = FLT_MAX;
  size_t initial_size;
  std::set<node*, compnode> neighbors;
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
  node(size_t start_rows_, size_t start_cols_, size_t width_, size_t start_rows_2d_, size_t start_cols_2d_, size_t width_2d_, size_t depth_, /* uint32_t binary_code_,  */node* parent_);
  static void setStaticMember(size_t width_, size_t height_, size_t quatree_width_, Eigen::Matrix4f & ROBOTWORLD_T_CAMERA);
  ~node();
  std::array<node*, 4> children = {nullptr};
  node* parent = nullptr;
  void showNodeFrame();
};

// inline auto cmp = [](const node* a,const node* b)
// {
//   if (a->valid_points_size < b->valid_points_size)
//   {
//     return true;
//   }
//   else if (a->valid_points_size == b->valid_points_size)
//   {
//     if (a->mse > b->mse)
//     {
//       return true;
//     }
//     else
//       return false;
//   }
//   else
//     return false;
// };


void deleteNodeInQuatree(node* p);


}

#endif