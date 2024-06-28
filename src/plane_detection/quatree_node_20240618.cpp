
#include "plane_detection/quatree_node_20240618.h"
#include <assert.h>
#include <plane_detection/type.h>
#include <Eigen/Eigenvalues>
// extern orginazed_points raw_points;
// extern parameter param;
// 这个地方根据使用哪种类型的点云在变
// extern orginazed_points raw_points;
// extern Eigen::Matrix4f ROBOTWORLD_T_CAMERA;
namespace quatree
{
  size_t node::data_width = 0;
  size_t node::data_height = 0;
  size_t node::quatree_width = 0;
  Eigen::Vector3f node::check_normal = Eigen::Vector3f::Zero();
  orginazed_points node::raw_points;
  parameter node::param;

  // for debug
  cv::Mat debug_color_image;

bool compnode::operator() (std::shared_ptr<node> a, std::shared_ptr<node> b) const
{
  if (a == b)
  {
    return false;
  }
  else
  {
    if (a->is_plane && b->is_plane)
    {
      // valid_points_size 会改变
      if (a->valid_points_size > b->valid_points_size)
      {
        return true;
      }
      else if (a->valid_points_size == b->valid_points_size)
      {
        // return a->mse > b->mse;

        if (a->mse != b->mse)
        {
          if (a->mse < b->mse)
          {
            return true;
          }
          else
          {
            return false;
          }
        }
        else
        {
          return a > b;
        }
      }
      else
        return false;
    }
    else if (!a->is_plane && !b->is_plane)
    {
      return a > b;
    }
    else // 保证为平面的排在队列的前端
    {
      if (a->is_plane)
      {
        return true;
      }
      else
      {
        return false;
      }
    }
  }
}


bool reverseComnode::operator()(std::shared_ptr<node> a, std::shared_ptr<node> b) const
{
  return baseCom(b, a);
}

node::node(size_t start_rows_, size_t start_cols_, size_t width_, size_t start_rows_2d_, size_t start_cols_2d_, size_t width_2d_, size_t depth_, std::shared_ptr<node> parent_): start_rows(start_rows_), start_cols(start_cols_), width(width_),start_rows_2d(start_rows_2d_), start_cols_2d(start_cols_2d_), width_2d(width_2d_), depth(depth_), parent(parent_) 
{
  assert(start_rows_ >= 0);
  assert(start_rows_ < data_height);
  assert(start_cols_ >= 0);
  assert(start_cols_ < data_width);// width 计数方式与序号方式不一样
  assert(start_rows_ + width_ <= quatree_width);
  assert(start_cols_ + width_ <= quatree_width);
  
  neighbors.clear();
}

void node::resetNode()
{
  center = Eigen::Vector3f::Zero();
  normal = Eigen::Vector3f::Zero();
  mse = FLT_MAX;
  curvature = 0.0;
}

void node::initialize()
{
  if (depth == param.leafnode_depth) 
  {
    // Leaf node specific initialization
    is_leafnode = true;
    valid_points = raw_points.getRectPoint(start_rows, start_cols, width, width);
    valid_points_size = valid_points.size();
    initial_size = valid_points_size;

    if (valid_points_size >= param.patch_num_th) 
    {
      is_validnode = true;
      stats.push(valid_points);
      stats.compute(center, normal, mse, curvature);

      if (curvature < param.eigen_value_th && mse < param.patch_mse_th) 
      {
        if (abs(check_normal.dot(normal)) > 0.866) 
        {
          is_plane = true;
        }
      } 
      else 
      {
        resetNode();
      }
    }
    else 
    {
        is_validnode = false;
    }
  } 
  else 
  {
      is_leafnode = false;
      is_plane = false;
      createChildren();
  }
}

std::shared_ptr<node> node::create(size_t start_rows_, size_t start_cols_, size_t width_, size_t start_rows_2d_, size_t start_cols_2d_, size_t width_2d_, size_t depth_, std::shared_ptr<node> parent_)
{
  std::shared_ptr<node> node_p = std::make_shared<node>(start_rows_, start_cols_, width_, start_rows_2d_, start_cols_2d_, width_2d_, depth_, parent_);
  node_p->initialize();  // 创建对象后立即调用初始化函数
  return node_p;
}

void node::createChildren()
{
  size_t child_width = width >> 1;
  size_t child_width_2d = width_2d >> 1;

  for (size_t i = 0; i < 4; i++) 
  {
    size_t child_start_rows = start_rows + child_width * (i >> 1 & 1);
    size_t child_start_cols = start_cols + child_width * (i & 1);
    size_t child_start_rows_2d = start_rows_2d + child_width_2d * (i >> 1 & 1);
    size_t child_start_cols_2d = start_cols_2d + child_width_2d * (i & 1);

    if (child_start_rows >= data_height || child_start_cols >= data_width) 
    {
      children.at(i) = nullptr;
    } 
    else 
    {
      std::shared_ptr<node> this_ptr = shared_from_this();
      children.at(i) = node::create(child_start_rows, child_start_cols, child_width, child_start_rows_2d, child_start_cols_2d, child_width_2d, depth + 1, this_ptr);
      if (children.at(i)->valid_points_size == 0)
      {
        for (size_t j = 0; j < 4; j++)
        {
          if (children.at(i)->parent->children.at(j) == children.at(i))
          {
            // cout<<"ghjkl"<<endl;
            // LOG(INFO)<<"at "<<i<<"child, need delete"<<endl;
            // p->parent->children.at(i) = nullptr;
            // delete p;
            // p = nullptr;
            children.at(i)->parent->children.at(j);
            break;
          }
        }
      }
      
    }
  }
}

node::~node()
{
  neighbors.clear();
  valid_points.clear();
  valid_points_index.clear();
  stats.clear();
}

void deleteNodeInQuatree(std::shared_ptr<node> p)
{
  // LOG(INFO)<<p;
  // cout<<"p: "<<p<<endl;
  // cout<<"p->parent: "<<p->parent<<endl;
  if (p->parent)
  {
    // cout<<"yyy"<<endl;
    // LOG(INFO)<<"CHECK CHILDREN"<<endl;
    // LOG(INFO)<<"node parent is: "<<p->parent<<endl;
    for (size_t i = 0; i < 4; i++)
    {
      // cout<<"p->parent->children.at "<<i<<" "<<p->parent->children.at(i)<<endl;
      // LOG(INFO)<<p->parent->children.at(i);
      // LOG(INFO)<<"at "<<i<<" child"<<endl;
      if (p->parent->children.at(i) == p)
      {
        // cout<<"ghjkl"<<endl;
        // LOG(INFO)<<"at "<<i<<"child, need delete"<<endl;
        // p->parent->children.at(i) = nullptr;
        // delete p;
        // p = nullptr;
        p->parent->children.at(i).reset();
        break;
      }
    }
  }
  else// 表明p是根节点
  {
    // cout<<"nnnn"<<endl;
    // LOG(INFO)<<"the delete node is root";
    // delete p;
    // p = nullptr;
    p.reset();
  }
}

void node::setStaticMember(size_t width_, size_t height_, size_t quatree_width_, Eigen::Matrix4f & ROBOTWORLD_T_CAMERA, orginazed_points & raw_points_, parameter & param_)
{
  data_width = width_; 
  data_height = height_; 
  quatree_width = quatree_width_;
  // 如果不考虑一定要有与平面夹角的约束，这一项是不是可以不考虑
  check_normal = ROBOTWORLD_T_CAMERA.block<1,3>(2,0).transpose();
  raw_points = raw_points_;
  param = param_;
}

}
