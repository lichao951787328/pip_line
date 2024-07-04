/*
 * @description: 
 * @param : 
 * @return: 
 */
#ifndef _TYPE_H_
#define _TYPE_H_
#include <Eigen/Core>
#include <vector>
#include <assert.h>
#include <opencv2/highgui.hpp>
#include <list>
#include <set>
#include <plane_detection/quatree_node_20240618.h>
#include <iostream>
#include <glog/logging.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <plane_detection/point_type.h>
using namespace std;

struct index2D
{
  size_t rows;
  size_t cols;
  std::vector<std::vector<std::shared_ptr<quatree::node>>> index2d;
  index2D()
  {

  }
  index2D(size_t rows_, size_t cols_, std::shared_ptr<quatree::node> p)
  {
    rows = rows_; cols = cols_;
    
    assert(p->start_rows_2d + p->width_2d >= rows);
    assert(p->start_cols_2d + p->width_2d >= cols);

    initial(p);
  }

  index2D & operator=(const index2D & other)
  {
    if (this == &other)
    {
      return *this;
    }
    this->rows = other.rows;
    this->cols = other.cols;
    this->index2d = other.index2d;
    return *this;
  }

  void initial(std::shared_ptr<quatree::node> root)
  {
    index2d.resize(rows);
    for (auto & iter : index2d)
    {
      iter.resize(cols);
    }
    std::list<std::shared_ptr<quatree::node>> Q;
    Q.emplace_back(root);
    while (!Q.empty())
    {
      std::shared_ptr<quatree::node> tmpnode = Q.front();
      Q.pop_front();
      if (tmpnode->is_plane || (!tmpnode->is_plane && tmpnode->is_leafnode))// 混合区域增长算法需要考虑所有不为nullptr的点
      {
        std::vector<std::vector<std::shared_ptr<quatree::node>>>::iterator rows_iter = index2d.begin() + tmpnode->start_rows_2d;
        // tmpnode->width_2d tmpnode->height_2d 是一样大的
        for (size_t i = 0; i < tmpnode->width_2d; i++)
        {
          std::vector<std::shared_ptr<quatree::node>>::iterator rows_cols_iter = rows_iter->begin() + tmpnode->start_cols_2d;
          for (size_t j = 0; j < tmpnode->width_2d; j++)
          {
            *rows_cols_iter = tmpnode;
            rows_cols_iter++; 
          }
          rows_iter++;
        }
      }
      else
      {
        for (auto & iter_node : tmpnode->children)
        {
          if (iter_node)
          {
            Q.emplace_back(iter_node);
          }
        }
      }
    }
  }
  // x是行 y是列
  std::shared_ptr<quatree::node> getNodeByIndex(size_t x, size_t y)
  {
    return index2d.at(x).at(y);
  }

  void setNullptr()
  {
    for (auto & iter : index2d)
    {
      std::for_each(iter.begin(), iter.end(), [](std::shared_ptr<quatree::node> & p){ p = nullptr;});
    }
  }

  void showInfo()
  {
    for (auto & iter_rows : index2d)
    {
      for (auto & iter_rows_cols : iter_rows)
      {
        std::cout<<iter_rows_cols<<" ";
      }
      std::cout<<std::endl;
    }
  }

  // 在LOG(INFO)中，即使默认不打印出其内容，也会耗时
  void getNeighbors(std::shared_ptr<quatree::node> p, std::set<std::shared_ptr<quatree::node>, quatree::compnode> & neighbors)
  {
    neighbors.clear();
    cv::Mat image = cv::Mat::zeros(rows, cols, CV_8UC1);
    // std::cout<<image.rows<<" "<<image.cols<<endl;
    cv::rectangle(image, cv::Rect(p->start_cols_2d, p->start_rows_2d, p->width_2d, p->width_2d), 255, -1, 1, 0);

    // int scale_factor = 20;  // 放大倍数
    // cv::Mat enlarged_img;
    // cv::resize(image, enlarged_img, cv::Size(), scale_factor, scale_factor, cv::INTER_LINEAR);
    // cv::imshow("enlarged_img", enlarged_img);
    // cv::waitKey(0);

    int kernel_size = 3;
    cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(kernel_size, kernel_size));
    cv::Mat dilated_image;
    cv::dilate(image, dilated_image, kernel);

    // cv::Mat enlarged_img2;
    // cv::resize(dilated_image, enlarged_img2, cv::Size(), scale_factor, scale_factor, cv::INTER_LINEAR);
    // cv::imshow("enlarged_img2", enlarged_img2);
    // cv::waitKey(0);

    cv::Mat nei_pixels = dilated_image - image;

    // cv::Mat enlarged_img3;
    // cv::resize(nei_pixels, enlarged_img3, cv::Size(), scale_factor, scale_factor, cv::INTER_LINEAR);
    // cv::imshow("enlarged_img3", enlarged_img3);
    // cv::waitKey(0);
    // cout<<nei_pixels.rows<<" "<<nei_pixels.cols<<endl;

    std::vector<cv::Point> white_points;
    cv::findNonZero(nei_pixels, white_points);
    for (auto & cv_p : white_points)
    {
      // cout<<"nei: "<<index2d.at(cv_p.y).at(cv_p.x)<<endl;
      if (index2d.at(cv_p.y).at(cv_p.x))
      {
        neighbors.insert(index2d.at(cv_p.y).at(cv_p.x));
      }
    }
    
    // cout<<"nei size: "<<neighbors.size();
    // for (int i = 0; i < nei_pixels.rows; i++)
    // {
    //   for (int j = 0; j < nei_pixels.cols; j++)
    //   {
        
    //     if (nei_pixels.at<uchar>(i, j) == 255)
    //     {
    //       neighbors.insert(index2d.at(i).at(j));
    //     }
    //   }
    // }
    
    // // 分三块，上面一排， 中间n排，虽然只有两个数，下面一排
    // size_t start_col_index = (p->start_cols_2d > 0) ? p->start_cols_2d - 1 : p->start_cols_2d;
    // size_t end_col_index = (p->start_cols_2d + p->width_2d < cols) ? (p->start_cols_2d + p->width_2d) : (p->start_cols_2d + p->width_2d - 1);
    // // LOG(INFO)<<"start_col_index: "<<start_col_index<<" end_col_index: "<<end_col_index;
    // if (p->start_rows_2d > 0)
    // {
    //   vector<std::shared_ptr<quatree::node>>::iterator iter_node;
    //   for (iter_node = index2d.at(p->start_rows_2d - 1).begin() + start_col_index; iter_node <= index2d.at(p->start_rows_2d - 1).begin() + end_col_index; iter_node++)
    //   {
    //     if (*iter_node)
    //     {
    //       neighbors.insert(*iter_node);
    //     }
    //   }
    // }
    // // LOG(INFO)<<"FIRST: ";
    // // for (auto & iter_neighbor : neighbors)
    // // {
    // //   // LOG(INFO)<<iter_neighbor;
    // // }
    // vector<size_t> col_indexs;
    // if (start_col_index == p->start_cols_2d - 1)
    // {
    //   col_indexs.emplace_back(start_col_index);
    // }
    // if (end_col_index == p->start_cols_2d + p->width_2d)
    // {
    //   col_indexs.emplace_back(end_col_index);
    // }
    // // LOG(INFO)<<"WIDTH: "<<p->width_2d;
    // for (size_t i = 0; i < p->width_2d; i++)
    // {      
    //   for (auto & iter_col_index : col_indexs)
    //   {
    //     // LOG(INFO)<<iter_col_index;
    //     // LOG(INFO)<<*(index2d.at(p->start_rows_2d + i).begin() + iter_col_index);
    //     std::shared_ptr<quatree::node> tmpnode = *(index2d.at(p->start_rows_2d + i).begin() + iter_col_index);
    //     if (tmpnode)
    //     {
    //       neighbors.insert(tmpnode);
    //     }
    //   }
    //   // LOG(INFO)<<neighbors.size();
    // }
    // // LOG(INFO)<<"second: ";
    // // for (auto & iter_neighbor : neighbors)
    // // {
    // //   // LOG(INFO)<<iter_neighbor;
    // // }
    // size_t row_index = p->start_rows_2d + p->width_2d;
    // if (row_index < rows)
    // {
    //   vector<std::shared_ptr<quatree::node>>::iterator iter_node;
    //   for (iter_node = index2d.at(row_index).begin() + start_col_index; iter_node <= index2d.at(row_index).begin() + end_col_index; iter_node++)
    //   {
    //     if (*iter_node)
    //     {
    //       neighbors.insert(*iter_node);
    //     }
    //   }
    // }
    // LOG(INFO)<<"tmpnode: "<<p;
    // for (auto & iter_neighbor : neighbors)
    // {
    //   // LOG(INFO)<<iter_neighbor;
    // }
  }

  void pritfIndex()
  {
    for (int i = 0; i < rows; i++)
    {
      for (int j = 0; j < cols; j++)
      {
        cout<<index2d.at(i).at(j)<<" ";
      }
      cout<<endl;
    }
  }
};

struct levelsIndex2d
{
  size_t rows;
  size_t cols;

  typedef std::vector<std::vector<std::shared_ptr<quatree::node>>> levelIndex2d;

  std::vector<levelIndex2d> levelsIndex;

  levelsIndex2d()
  {

  }
  // 必须长宽相等，因为会显示根节点的地址
  levelsIndex2d(std::shared_ptr<quatree::node> p)
  {
    rows = p->start_rows_2d + p->width_2d;
    cols = p->start_cols_2d + p->width_2d;
    initial(p);
  }

  levelsIndex2d & operator=(const levelsIndex2d & other)
  {
    if (this == &other)
    {
      return *this;
    }
    this->rows = other.rows;
    this->cols = other.cols;
    this->levelsIndex = other.levelsIndex;
    return *this;
  }

  void initial(std::shared_ptr<quatree::node> root)
  {
    size_t level = 0;
    std::list<std::shared_ptr<quatree::node>> l;
    l.emplace_back(root);
    levelIndex2d tmpIndex2d;
    tmpIndex2d.resize(rows);
    for (auto & iter_row : tmpIndex2d)
    {
      iter_row.resize(cols);
      std::for_each(iter_row.begin(), iter_row.end(), [](std::shared_ptr<quatree::node> & p){ p = nullptr;});
    }
    
    while (!l.empty())
    {
      std::shared_ptr<quatree::node> tmpnode = l.front();
      
      if (tmpnode->depth == level)
      {
        std::vector<std::vector<std::shared_ptr<quatree::node>>>::iterator rows_iter = tmpIndex2d.begin() + tmpnode->start_rows_2d;
        for (size_t i = 0; i < tmpnode->width_2d; i++)
        {
          std::vector<std::shared_ptr<quatree::node>>::iterator rows_cols_iter = rows_iter->begin() + tmpnode->start_cols_2d;
          for (size_t j = 0; j < tmpnode->width_2d; j++)
          {
            *rows_cols_iter = tmpnode;
            rows_cols_iter++;
          }
          rows_iter++;
        }
        for (auto & iter_node : tmpnode->children)
        {
          if (iter_node)
          {
            l.emplace_back(iter_node);
          }
        }
        l.pop_front();
      }
      else
      {
        levelsIndex.emplace_back(tmpIndex2d);
        for (auto & iter_row : tmpIndex2d)
        {
          std::for_each(iter_row.begin(), iter_row.end(), [](std::shared_ptr<quatree::node> & p){ p = nullptr;});
        }
        level++;
      }
    }
  }

  void showInfo()
  {
    for (auto & iter_index2d : levelsIndex)
    {
      for (auto & iter_rows : iter_index2d)
      {
        for (auto & iter_rows_cols : iter_rows)
        {
          std::cout<<iter_rows_cols<<" ";
        }
        std::cout<<std::endl;
      }
      std::cout<<std::endl;
    }
  }
};


struct index2dBinaryCode
{
  size_t row_cor, col_cor;
  size_t rows, cols;
  std::vector<std::vector<std::shared_ptr<quatree::node>>> index2ds;
  index2dBinaryCode(size_t rows_, size_t cols_, std::shared_ptr<quatree::node> p)
  {
    rows = rows_; cols = cols;
    size_t depth = std::log2(p->width_2d);
    row_cor = 0; col_cor = 0;
    for (size_t i = 0; i < depth; i++)
    {
      row_cor = (row_cor<<2)|(1<<2);
      col_cor = (col_cor<<2)|(1);
    }
    index2ds.resize(rows);
    for (auto & iter_row : index2ds)
    {
      iter_row.resize(cols);
    }
    initial(p);
  }
  void initial(std::shared_ptr<quatree::node> p)
  {
    // std::list<quatree::node*> l;
    // l.emplace_back(p);
    // while (!l.empty())
    // {
    //   quatree::node* tmpnode = l.front();
    //   l.pop_front();
    //   if (tmpnode->is_leafnode)
    //   {
    //     size_t row_index = tmpnode->binary_code & row_cor;
    //     size_t col_index = tmpnode->binary_code & col_cor;
    //     index2ds.at(row_index).at(col_index) = tmpnode;
    //   }
    //   else
    //   {
    //     for (auto & iter_child : tmpnode->children)
    //     {
    //       if (iter_child)
    //       {
    //         l.emplace_back(iter_child);
    //       }
    //     }
    //   }
    // }
  }
};


Eigen::Matrix4f getT(const float &px, const float &py, const float &pz, const float &rx, const float &ry, const float &rz);


double _deg2rad(double degree);

float correct2threeDecimal(float x);

#endif