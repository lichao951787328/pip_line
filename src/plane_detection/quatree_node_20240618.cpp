
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
  // int node::xxx = 0;
  // test_xxx node::xxx = test_xxx();
  orginazed_points node::raw_points;
  parameter node::param;

bool compnode::operator() (node const * const &  a, node const * const & b) const
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
      if (a->valid_points_size < b->valid_points_size)
      {
        return true;
      }
      else if (a->valid_points_size == b->valid_points_size)
      {
        return a->mse > b->mse;
        // return a < b;
        // if (a->start_cols_2d > b->start_cols_2d)
        // {
        //   return true;
        // }
        // else if (a->start_cols_2d == b->start_cols_2d)
        // {
        //   if (a->start_rows_2d > b->start_rows_2d)
        //   {
        //     return true;
        //   }
        //   else if (a->start_rows_2d < b->start_rows_2d)
        //   {
        //     return false;
        //   }
        //   else
        //   {
        //     return a < b;
        //   }
        // }
        // else
        //   return false;
      }
      else
        return false;
    }
    else if (!a->is_plane && !b->is_plane)
    {
      return a > b;
      // if (a->valid_points_size < b->valid_points_size)
      // {
      //   return true;
      // }
      // else if (a->valid_points_size == b->valid_points_size)
      // {
      //   // return a < b;
      //   if (a->start_cols_2d > b->start_cols_2d)
      //   {
      //     return true;
      //   }
      //   else if (a->start_cols_2d == b->start_cols_2d)
      //   {
      //     if (a->start_rows_2d > b->start_rows_2d)
      //     {
      //       return true;
      //     }
      //     else if (a->start_rows_2d < b->start_rows_2d)
      //     {
      //       return false;
      //     }
      //     else
      //     {
      //       return a < b;
      //     }
      //   }
      //   else
      //     return false;
      // }
      // else
      // {
      //   return false;
      // }
    }
    else
    {
      return !a->is_plane; 
    }
  }
}


node::node(size_t start_rows_, size_t start_cols_, size_t width_, size_t start_rows_2d_, size_t start_cols_2d_, size_t width_2d_, size_t depth_,/*  uint32_t binary_code_, */ node* parent_)
{
  assert(start_rows_ >= 0);
  assert(start_rows_ < data_height);
  assert(start_cols_ >= 0);
  assert(start_cols_ < data_width);// width 计数方式与序号方式不一样
  assert(start_rows_ + width_ <= quatree_width);
  assert(start_cols_ + width_ <= quatree_width);
  start_rows = start_rows_; start_cols = start_cols_; width = width_; 
  start_rows_2d = start_rows_2d_; start_cols_2d = start_cols_2d_; width_2d = width_2d_;
  depth = depth_; /* binary_code = binary_code_;  */parent = parent_;
  neighbors.clear();
  if (depth == param.leafnode_depth)
  {
    // LOG(INFO)<<"it is leaf node."<<std::endl;
    is_leafnode = true;
    // 2014年算法中是将图像中附上代号
    // 改进点1
    // 可以是一个改进点，如果不是平面可以放到refine环节，使用像素来腐蚀来获取点的信息
    // 如果是平面，则直接记录，不需要把点都记录下来
    valid_points = raw_points.getRectPoint(start_rows, start_cols, width, width);

    // raw_points.getRectPointAndIndex(start_rows, start_cols, width, width, valid_points, valid_points_index);
    // valid_points = raw_points.getRectPoint(start_rows, start_cols, width, width);// 需要再次确定
    valid_points_size = valid_points.size();
    initial_size = valid_points_size;
    // LOG(INFO)<<"valid_points_size: "<<valid_points_size<<std::endl;
    // LOG(INFO)<<"param.patch_num_th: "<<param.patch_num_th<<std::endl;
    if (valid_points_size >= param.patch_num_th)
    {
      // LOG(INFO)<<"the valid points is more than th"<<std::endl;
      is_validnode = true;
      // center = Eigen::Vector3f::Zero();
      // std::for_each(valid_points.begin(), valid_points.end(), [](Eigen::Vector3f & point){stats.push(point)});
      
      stats.push(valid_points);
      // center /= valid_points_size;
      // Eigen::Matrix3f S = Eigen::Matrix3f::Zero();
      // for (auto & iter_point : valid_points)
      // {
      //   Eigen::Vector3f v = iter_point - center;
      //   S += v * v.transpose();
      //   J += iter_point * iter_point.transpose();
      // }
      // // 通过特征值大小及mse共同判断是否为平面
      // Eigen::EigenSolver<Eigen::Matrix3f> es(S);
      // Eigen::Matrix3f::Index b;
      // auto minEigenValue = es.eigenvalues().real().minCoeff(&b);
      // auto eigenValuesSum = es.eigenvalues().real().sum();
      // // 需要修正叶子节点的方向吗？
      // normal = es.eigenvectors().real().col(b);
      // float d = normal.dot(center);
      // float mse_check = 0.0;
      // for (auto & iter_point : valid_points)
      // {
      //   mse_check += std::pow(normal.dot(iter_point) - d, 2); 
      // }
      // mse_check /= valid_points_size;
      // if (d < 0)
      // {
      //   normal = - normal;
      // }
      // mse = minEigenValue/valid_points_size;// 验证计算方式
      stats.compute(center, normal, mse, curvature);

      // std::cout<<"mse_check: "<<mse_check<<std::endl;
      // std::cout<<"mse: "<<mse<<std::endl;

      if (curvature < param.eigen_value_th && mse < param.patch_mse_th)
      {
        // 后续考虑把这部分去掉，检测所有能踩的。
        // 也可以不去，毕竟是针对双足机器人的
        if (check_normal.dot(normal) > 0.866)// 这里去除了法向量太小的点，即如果面与水平面夹角较大，机器人不能踩上去，则不考虑
        {
          is_plane = true;
        }
      }
      else
      {
        center = Eigen::Vector3f::Zero();
        normal = Eigen::Vector3f::Zero();
        // J = Eigen::Matrix3f::Zero();
        mse = FLT_MAX;
        curvature = 0.0;
      }
    }
    else
    {
      // 证明此区块没有有效点
      is_validnode = false;
    }
  }
  else
  {
    // LOG(INFO)<<"it is not leaf node."<<std::endl;
    is_leafnode = false; 
    is_plane = false;
    // 也可以通过有效点的数量来进行分割判断，但是似乎会增加计算量
    size_t child_width = width >>1;
    size_t child_width_2d = width_2d >>1;
    // 对node里超出高度范围，有效点不足，不是平面分开进行标志？
    for (size_t i = 0; i < 4; i++)
    {
      // 这个地方可以增加节点编码，方便求邻近节点
      size_t child_start_rows = start_rows + child_width * (i >>1 & 1);//求交
      size_t child_start_cols = start_cols + child_width * (i&1);
      size_t child_start_rows_2d = start_rows_2d + child_width_2d *(i>>1 & 1);
      size_t child_start_cols_2d = start_cols_2d + child_width_2d *(i & 1);
      // std::cout<<(i&1)<<1<<" "<<(i>>1 & 1)<<std::endl;
      // std::cout<<(uint32_t)((i&1)<<1 || (i>>1 & 1))<<std::endl;
      // uint32_t child_binary_code = (binary_code<<2) | (uint32_t)((i>>1 & 1)<<1 | (i&1));
      // LOG(INFO)<<"current node information: "<<endl;
      // LOG(INFO)<<"start rows: "<<child_start_rows<<" start cols: "<<child_start_cols<<" width: "<<child_width<<endl;
      // 由于对原先的node进行角和，看看在不在depth image里
      if (child_start_rows >= data_height || child_start_cols >= data_width)
      {
        // LOG(WARNING)<<"current node point is nullptr."<<endl;
        children.at(i) = nullptr;
      }
      else
      {
        // 行起始，列起始，宽度，深度
        children.at(i) = new node(child_start_rows, child_start_cols, child_width, child_start_rows_2d, child_start_cols_2d, child_width_2d, /* child_binary_code, */ depth + 1, this);
        // 此情况下也应设为nullptr
        if (children.at(i)->is_leafnode)
        {
          if (children.at(i)->valid_points_size == 0)
          {
            for (int j = 0; j < 4; j++)
            {
              if (children.at(i)->parent->children.at(j) == children.at(i))
              {
                children.at(i)->parent->children.at(j) = nullptr;
                delete children.at(i);
                children.at(i) = nullptr;
                break;
              }
            }
          }
        }
      }
    }

    // cout<<"button up construct quadtree"<<endl;
    // LOG(INFO)<<"TRY TO BUTTON UP CONSTRUCT QUADTREE"<<endl;
    // // 在这个地方进行合并即可
    // bool parentNodeIsPlane = true;
    // Eigen::Vector3f mean_normal = Eigen::Vector3f::Zero();
    // size_t sum_points = 0;
    // for (auto & child : children)
    // {
    //   // 如果子结点为空或者不是平面
    //   if (child || !child->is_plane)
    //   {
    //     parentNodeIsPlane = false;
    //     break;
    //   }
    //   else
    //   {
    //     mean_normal += child->valid_points_size * child->normal;
    //     sum_points += child->valid_points_size;
    //   }
    // }
    // if (parentNodeIsPlane)
    // {
    //   mean_normal /= sum_points;
    //   bool merge_flag = true;
    //   for (auto & iter_child : children)
    //   {
    //     if (mean_normal.dot(iter_child->normal) < param.quatree_merge_normal_dot_th)
    //     {
    //       merge_flag = false;
    //       break;
    //     }
    //   }
    //   if (merge_flag)
    //   {
    //     stats = Stats(children.at(0)->stats, children.at(1)->stats, children.at(2)->stats, children.at(3)->stats);
    //     stats.compute(center, normal, mse, curvature);
    //     if (curvature < param.eigen_value_th && mse < param.patch_mse_th)
    //     {
    //       // p->mse = minEigenValue/sum_nums;// 验证计算方式
    //       is_plane = true;
    //       is_validnode = true;
    //       valid_points_size = stats.N;  
    //       // clear memory of children 
    //       for (auto & iter_node : children)
    //       {
    //         valid_points.insert(valid_points.end(), iter_node->valid_points.begin(), iter_node->valid_points.end());
    //         delete iter_node;
    //       }
    //       children = {nullptr, nullptr, nullptr, nullptr};
    //     }
    //     else
    //     {
    //       stats.clear();
    //       normal = Eigen::Vector3f::Zero();
    //       center = Eigen::Vector3f::Zero();
    //       // p->J = Eigen::Matrix3f::Zero();
    //       is_plane = false;
    //     }
    //   }
    // }
  
  }
}

node::~node()
{
  neighbors.clear();
  valid_points.clear();
  valid_points_index.clear();
  stats.clear();
}

void node::showNodeFrame()
{
  std::list<node*> L;
  L.push_back(this);
  while (!L.empty())
  {
    node* tmpnode = L.front();
    LOG(INFO)<<tmpnode<<" parent is "<<tmpnode->parent<<std::endl;
    LOG(INFO)<<tmpnode->start_rows<<" "<<tmpnode->start_cols<<" "<<tmpnode->width<<std::endl;
    LOG(INFO)<<tmpnode->start_rows_2d<<" "<<tmpnode->start_cols_2d<<" "<<tmpnode->width_2d<<std::endl;
    LOG(INFO)<<"is_plane "<<tmpnode->is_plane;
    // LOG(INFO)<<"is_validnode "<<tmpnode->is_validnode;
    LOG(INFO)<<"is_leafnode "<<tmpnode->is_leafnode;
    LOG(INFO)<<"neighbors: ";
    for (auto & iter_node : tmpnode->neighbors)
    {
      LOG(INFO)<<iter_node<<" ";
    }
    
    // LOG(INFO)<<"mse "<<tmpnode->mse;
    // LOG(INFO)<<"center "<<tmpnode->center.transpose();
    LOG(INFO)<<"normal "<<tmpnode->normal.transpose();
    // LOG(INFO)<<"J "<<tmpnode->J;
    LOG(INFO)<<"valid_points_size "<<tmpnode->valid_points_size;
    
    L.pop_front();
    for (size_t i = 0; i < 4; i++)
    {
      if (tmpnode->children.at(i) != nullptr)
      {
        L.push_back(tmpnode->children.at(i));
      }
      LOG(INFO)<<"child "<<i<<": "<<tmpnode->children.at(i)<<std::endl;
    }
  }
}

void deleteNodeInQuatree(node* p)
{
  // LOG(INFO)<<p;
  cout<<"p: "<<p<<endl;
  cout<<"p->parent: "<<p->parent<<endl;
  if (p->parent)
  {
    cout<<"yyy"<<endl;
    // LOG(INFO)<<"CHECK CHILDREN"<<endl;
    // LOG(INFO)<<"node parent is: "<<p->parent<<endl;
    for (size_t i = 0; i < 4; i++)
    {
      cout<<"p->parent->children.at "<<i<<" "<<p->parent->children.at(i)<<endl;
      // LOG(INFO)<<p->parent->children.at(i);
      // LOG(INFO)<<"at "<<i<<" child"<<endl;
      if (p->parent->children.at(i) == p)
      {
        cout<<"ghjkl"<<endl;
        // LOG(INFO)<<"at "<<i<<"child, need delete"<<endl;
        p->parent->children.at(i) = nullptr;
        delete p;
        p = nullptr;
        break;
      }
    }
  }
  else// 表明p是根节点
  {
    cout<<"nnnn"<<endl;
    // LOG(INFO)<<"the delete node is root";
    delete p;
    p = nullptr;
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
