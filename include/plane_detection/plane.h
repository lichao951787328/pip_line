#ifndef _PLANE_H_
#define _PLANE_H_
#include <Eigen/Core>
#include <vector>
#include <limits.h>
#include <glog/logging.h>
#include <assert.h>
#include <cfloat>
#include <plane_detction/quadtree_new.h>
#include <opencv2\highgui\highgui.hpp>
struct plane
{
  Eigen::Vector3f center = Eigen::Vector3f::Zero();
  Eigen::Matrix3f J = Eigen::Matrix3f::Zero();
  Eigen::Vector3f normal = Eigen::Vector3f::Zero();
  float mse = FLT_MAX;
  size_t valid_points_size = 0;
  cv::Mat index_image = cv::Mat(index_x_rows, index_y_cols, CV_8UC1);
  enum direct {down, right, up, left};
  struct point_index_2d
  {
    size_t x, y;
    point_index_2d()
    {

    }
    point_index_2d(size_t x_, size_t y_)
    {
      x = x_; y = y_;
    }
    point_index_2d& operator=(const point_index_2d& other)
    {
      if (this == &other) return *this;	
      this->x = other.x;
      this->y = other.y;
      return *this;
    }
  };
  
  // 暂时先不用看看效果
  std::vector<Eigen::Vector3f> valid_points;
  bool checkInPlane(quatree::node* p)
  {
    assert(p != nullptr && "node is nullptr");
    assert(p->is_plane && "node is not a plane, please do not add it in a plane");
    if (valid_points_size == 0)
    {
      return true;
    }
    else
    {
      // 本来是有三个条件，但是第三个条件（合并后的mse）在add时计算更好，这样可以减少以
      if (std::abs(normal.dot(p->normal)) > 0.8 && std::abs((center - p->center).dot(normal)) > 0.003)//阈值三个
      {
        return true;
      }
    }
    return false;
  }
  // 在加入校验条件三是否满足要求, 会计算出该平面合并后会变成的新的平面，但是暂时还不改变原平面的参数，必须确定这个合并后的平面的mse是最小的，才来更新
  // 所有的平面都不满足增加条件，则返回false
  // 创建一个当前轮廓点到下一个的索引，方便合并
  bool addNode2Plane(quatree::node* p)
  {
    assert(p != nullptr && "node is nullptr");
    assert(p->is_plane && "node is not a plane, please do not add it in a plane");
    if (valid_points_size == 0)
    {
      center = p->center;
      J = p->J;
      normal = p->normal;
      mse = p->mse;
      valid_points_size = p->valid_points_size;
      // point_index_2d p1(p->start_rows_2d, p->start_cols_2d);
      // point_index_2d p2(p->start_rows_2d + p->width_2d - 1, p->start_cols_2d);
      // point_index_2d p3(p->start_rows_2d + p->width_2d - 1, p->start_cols_2d + p->width_2d - 1);
      // point_index_2d p4(p->start_rows_2d, p->start_cols_2d + p->width_2d - 1);
      
      cv::rectangle(index_image, cv::Rect(p->start_rows_2d, p->start_cols_2d, p->width_2d, p->width_2d), 255, -1, 1, 0);

      return true;
    }
    // 到这一步表示满足了法向量和距离要求
    Eigen::Vector3f M = center * valid_points_size + p->center * p->valid_points_size;
    size_t sum_points = valid_points_size + p->valid_points_size;
    Eigen::Vector3f center_cand = M/sum_points;
    Eigen::Matrix3f J_cand = J + p->J;
    Eigen::Matrix3f S_cand = J_cand - M*center_cand.transpose();
    Eigen::EigenSolver<Eigen::Matrix3f> es(S_cand);
    Eigen::Matrix3f::Index b;
    auto minEigenValue = es.eigenvalues().real().minCoeff(&b);
    double eigenValuesSum = es.eigenvalues().real().sum();
    double mse_cand = minEigenValue/sum_points;
    if (mse_cand < mse_plane_th)
    {
      // 表示可能会作为加入的平面，但还是需要确定哪个平面合并后的mse最小
      center = center_cand;
      J = J_cand;
      normal = es.eigenvectors().real().col(b);
      mse = mse_cand;
      valid_points_size = sum_points;
      cv::rectangle(index_image, cv::Rect(p->start_rows_2d, p->start_cols_2d, p->width_2d, p->width_2d), 255, -1, 1, 0);
      // 对于插入的节点与平面内已有的轮廓线，左对右，上对下
      uint8_t add_edge = 0;
      bool has_overlap_edge = false;

      std::vector<direct_line>::iterator iter_edge;
      for (iter_edge = down_edges.begin(); iter_edge != down_edges.end(); iter_edge++)
      {
        if (iter_edge->start_point.y == p->start_cols_2d + p->width_2d)// 表明这个边与node相邻
        {
          has_overlap_edge == true;
          size_t overlap_start, overlap_end;
          bool change_flag = false;
          for (size_t i = iter_edge->start_point.x; i <= iter_edge->end_point.x; i++)
          {
            assert(i < p->start_rows_2d + p->width_2d);
            if(i < p->start_rows_2d)
              continue;
            bool end_flag = false;
            for(size_t j = p->start_rows_2d; j < p->start_rows_2d + p->width_2d; j++)
            {
              if (!change_flag && i == j)
              {
                overlap_start = i;
                change_flag = true;
              }
              if (change_flag && i != j)
              {
                overlap_end = i - 1;
                end_flag = true;
              }
            }
            if (!end_flag && change_flag)
            {
              overlap_end = iter_edge->end_point.x;
            }
            if (end_flag)
            {
              break;
            }
          }

          if (overlap_start == iter_edge->start_point.x)
          {
            if (p->start_rows_2d > overlap_start)
            {
              size_t tmp_edge_y = p->start_cols_2d + p->width_2d - 1;
              point_index_2d p1(overlap_start - 1, tmp_edge_y);
              point_index_2d p2(p->start_rows, tmp_edge_y);
              up_edges.emplace_back(direct_line(p1, p2, up));
            }

            if (overlap_end < p->start_rows_2d + p->width_2d - 1)
            {
              size_t tmp_edge_y = p->start_cols_2d + p->width_2d - 1;
              point_index_2d p1(p->start_rows_2d + p->width_2d - 1, tmp_edge_y);
              point_index_2d p2(overlap_end + 1, tmp_edge_y);
              up_edges.emplace_back(direct_line(p1 , p2, up));
              down_edges.erase(iter_edge);
            }
            else if (overlap_end < iter_edge->end_point.x)
            {
              iter_edge->start_point.x = overlap_end + 1;
              size_t tmp_edge_y = p->start_cols_2d + p->width_2d - 1;
              point_index_2d p1(overlap_start - 1, tmp_edge_y);
              point_index_2d p2(p->start_rows_2d, tmp_edge_y);
              up_edges.emplace_back(direct_line(p1, p2 ,up));
            }
          }
          else if(overlap_end == iter_edge->end_point.x)
          {
            if (overlap_end < p->start_rows_2d + p->width_2d - 1)
            {
              size_t tmp_edge_y = p->start_cols_2d + p->width_2d - 1;
              point_index_2d p1(p->start_cols_2d + p->width_2d - 1, tmp_edge_y);
              point_index_2d p2(overlap_end + 1, tmp_edge_y);
              up_edges.emplace_back(direct_line(p1, p2, up));
            }
            iter_edge->end_point.x = overlap_end;
          }
          else if (overlap_start == p->start_rows_2d && overlap_end == p->start_rows_2d + p->width_2d - 1)
          {
            point_index_2d p1(overlap_end, iter_edge->start_point.y);
            point_index_2d p2 = iter_edge->end_point;
            down_edges.emplace_back(direct_line(p1, p2, down));
            iter_edge->end_point.x = overlap_start;
          }
        }
        
      }
      if (!has_overlap_edge)
      {
        point_index_2d p1(p->start_rows_2d + p->width_2d - 1, p->start_cols_2d + p->width_2d -1);
        point_index_2d p2(p->start_rows_2d, p->start_cols_2d + p->width_2d - 1);
        up_edges.emplace_back(direct_line(p1, p2, up));
      }
      
      has_overlap_edge = false;
      for (iter_edge = right_edges.begin(); iter_edge != right_edges.end(); iter_edge++)
      {
        if (iter_edge->start_point.x + 1 == p->start_rows_2d)
        {
          size_t overlap_start, overlap_end;
          bool change_flag = false;
          for (size_t i = iter_edge->start_point.y; i <= iter_edge->end_point.y; i++)
          {
            assert(i < p->start_cols_2d + p->width_2d);
            if (i < p->start_cols_2d)
            {
              continue;
            }
            bool end_flag = false;
            for (size_t j = p->start_cols_2d; j < p->start_cols_2d + p->width_2d; j++)
            {
              if (!change_flag && i == j)
              {
                overlap_start = i;
                change_flag = true;
              }
              if (change_flag && i != j)
              {
                overlap_end = i - 1;
                end_flag = true;
              }
            }
            if (end_flag)
            {
              break;
            }
          }
          if (iter_edge->end_point.y > overlap_end)
          {
            point_index_2d p1(iter_edge->end_point.x, overlap_end + 1);
            down_edges.emplace_back((p1, iter_edge->end_point, iter_edge->d));
          }
          iter_edge->end_point.y = overlap_start - 1;
          has_overlap_edge = true;
        }
        if (has_overlap_edge)
        {
          add_edge = add_edge | (1<<1);
        }
      }
      

      
      
      for (auto & iter_edge : left_edges)
      {
        assert(iter_edge.start_point.x != p->start_rows_2d + p->width_2d - 1);
        if (iter_edge.start_point.x == p->start_rows_2d + p->width_2d)
        {
          
        }
        
      }
      
      
      return true;
    }
    else
      return false;
  }

  void checkNodeEdges(quatree::node* p, uint8_t & add_edge)
  {
    point_index_2d p1(p->start_rows_2d, p->start_cols_2d);
    point_index_2d p2(p->start_rows_2d + p->width_2d - 1, p->start_cols_2d);
    point_index_2d p3(p->start_rows_2d + p->width_2d - 1, p->start_cols_2d + p->width_2d - 1);
    point_index_2d p4(p->start_rows_2d, p->start_cols_2d + p->width_2d - 1);

    direct_line dl1(p1, p2, down);
    checkNodeEdge(dl1, add_edge);

    direct_line dl2(p2, p3, right);
    checkNodeEdge(dl2, add_edge);

    direct_line dl3(p3, p4, up);
    checkNodeEdge(dl3, add_edge);

    direct_line dl4(p4, p1, left);
    checkNodeEdge(dl4, add_edge);
  }

  void checkNodeEdge(direct_line & dl, uint8_t & add_edge)
  {
    switch (dl.d)
    {
    case down: // 从上到下的插入边与从下到上平面边集合
      size_t line_y_index = dl.start_point.y;
      for (size_t i = 0; i < count; i++)
      {
        /* code */
      }
      
      break;
    case right:// 从左到右的插入边与从右到左平面边集合
      /* code */
      break;

    case up:
      /* code */
      break;
    case left:
      /* code */
      break;
    default:
      break;
    }
  }

  
};

#endif