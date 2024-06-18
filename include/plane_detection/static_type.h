#pragma once
#include <Eigen/Core>
#include <vector>
#include <opencv2/opencv.hpp>
#include <iostream>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <plane_detection/point_type.h>
#include <glog/logging.h>
const double kFx = 456.67578125;
const double kFy = 457.3671875;
const double kCx = 310.76171875;
const double kCy = 246.896484375;
const int kDepthWidth = 640;
const int kDepthHeight = 480;

const int kNeighborRange = 5; // boundary pixels' neighbor range
const int kScaleFactor = 1000; // scale coordinate unit in mm
const float kInfVal = 1000000; // an infinite large value used in MRF optimization
struct orginazed_points
{
  public:
  size_t width, height;
  // 写成一个vector会不会效率更高
  std::vector<std::vector<Eigen::Vector3f>> points;
  
  orginazed_points() : width(0), height(0) {}

  orginazed_points(const orginazed_points &rhs)
  {
    width = rhs.width;
    height = rhs.height;
    points = rhs.points;
  }

//赋值函数
  orginazed_points& operator=(const orginazed_points &rhs)
  {
    if (this == &rhs)
      return *this;

    width = rhs.width;
    height = rhs.height;
    points = rhs.points;
    return *this;
  }

  void initial(cv::Mat & depth_image)
  {
    width = depth_image.cols; height = depth_image.rows;
    // 先将长方形的点转成正方形
    points.reserve(height);
    for (size_t i = 0; i < height; i++)
    {
      std::vector<Eigen::Vector3f> i_row;
      i_row.reserve(width);
      for (size_t j = 0; j < width; j++)
      {
        // 没有使用迭代器
        float z = (float)(depth_image.at<unsigned short>(i, j)) / kScaleFactor;
        if (std::isnan(z))
        {
          i_row.emplace_back(Eigen::Vector3f(0, 0, z));
          continue;
        }
        float x = ((float)j - kCx) * z / kFx;
        float y = ((float)i - kCy) * z / kFy;
        i_row.emplace_back(Eigen::Vector3f(x, y, z)); 
      }
      points.emplace_back(i_row);
    }
  }
  
  void initialByPCL(pcl::PointCloud<pcl::PointXYZ> & pc)
  {
    width = pc.width; height = pc.height;
    std::cout<<"width: "<<width<<" height: "<<height<<std::endl;
    points.reserve(height);
    for (size_t i = 0; i < height; i++)
    {
      std::vector<Eigen::Vector3f> i_row;
      i_row.reserve(width);
      pcl::PointCloud<pcl::PointXYZ>::iterator start_iter = pc.begin() + i * width;
      pcl::PointCloud<pcl::PointXYZ>::iterator end_iter = pc.begin() + (i+1) * width;
      while (start_iter != end_iter)
      {
        i_row.emplace_back(Eigen::Vector3f(start_iter->x, start_iter->y, start_iter->z));
        start_iter++;
      }
      // {}只表示了其内部的两个数字，并没有表示中间所有的
      // for (auto & iter_point:std::range{pc.begin() + i * width, pc.begin() + (i + 1) * width})
      // {
      //   i_row.emplace_back(Eigen::Vector3f(iter_point->x, iter_point->y, iter_point->z));
      // }
      // std::cout<<i_row.size()<<std::endl;
      assert(i_row.size() == width);
      points.emplace_back(i_row);
    }
  }
  
  //先写在这，看后续是不是需要换成更高效的方式, 需要记录点的像素坐标
  /**
   * @description: get points from raw points
   * @param {size_t} start_rows: left and top point at a rect, the point start row index
   * @param {size_t} start_cols: left and top point at a rect, the point start col index
   * @param {size_t} width: rect width, it is on col direct
   * @param {size_t} height: rect height, it is on row direct
   * @return {*} valid points in rect
   */  
  IndexPoints getRectPoint(size_t start_rows, size_t start_cols, size_t width_, size_t height_)
  {
    
    assert(start_rows < height);
    assert(start_cols < width);
    if (start_rows + height_ > height)
    {
      height_ = height - start_rows;
    }
    if (start_cols + width_ > width)
    {
      width_ = width - start_cols;
    }
    // LOG(INFO)<<"start_cols: "<<start_cols<<" width_: "<<width_<<" start_rows: "<<start_rows<<" height_: "<<height_<<endl;
    // LOG(INFO)<<start_rows<<" "<<height_<<" "<<height<<endl;
    assert(start_rows + height_ <= height);
    // LOG(INFO)<<start_cols<<" "<<width_<<" "<<width<<endl;
    assert(start_cols + width_ <= width);

    // std::cout<<"start_rows: "<<start_rows<<endl;
    // std::cout<<"start_cols: "<<start_cols<<endl;
    // std::cout<<"width_: "<<width_<<endl;
    // std::cout<<"height_: "<<height_<<endl;

    IndexPoints return_points;
    return_points.reserve(width_ * height_);
    int i = 0;
    // std::cout<<"111111111"<<std::endl;
    std::vector<std::vector<Eigen::Vector3f>>::iterator get_row;
    for (get_row = points.begin() + start_rows; get_row != points.begin() + start_rows + height_; get_row++)
    {
      // std::cout<<"i = "<<i<<std::endl;
      std::vector<Eigen::Vector3f>::iterator get_point;
      int j = 0;
      for (get_point = get_row->begin() + start_cols; get_point != get_row->begin() + start_cols + width_; get_point++)
      {
        // std::cout<<"j = "<< j <<std::endl;
        if (!std::isnan(get_point->z()))
        {
          // 行 列
          IndexPoint tmppoint = std::make_pair(std::make_pair(start_rows + i, start_cols + j), *get_point);
          return_points.emplace_back(tmppoint);
        }
        j++;
      }
      i++;
    }
    return return_points;
    // std::vector<Eigen::Vector3f> rectPoints;
    // rectPoints.reserve(width_ * height_);
    // for (size_t i = 0; i < height_; i++)
    // {
    //   std::vector<Eigen::Vector3f> tmp(points.at(i + start_rows).begin() + start_cols, points.at(i + start_rows).begin() + start_cols + width_);
    //   rectPoints.insert(rectPoints.end(), tmp.begin(), tmp.end());
    // }
    // assert(rectPoints.size() == width_ * height_);
    // // std::for_each 可以替换
    // std::vector<Eigen::Vector3f> Points;// 只取有效点
    // for (auto & iter_point : rectPoints)
    // {
    //   if (!std::isnan(iter_point(2)))
    //   {
    //     Points.emplace_back(iter_point);
    //   }
    // }
    // return Points;
  }

  void getRectPointAndIndex(size_t start_rows, size_t start_cols, size_t width, size_t height, std::vector<Eigen::Vector3f> & getpoints, std::vector<std::pair<size_t, size_t>> & getindexs)
  {
    std::vector<std::vector<Eigen::Vector3f>>::iterator get_row;
    int i = 0;
    for (get_row = points.begin() + start_rows; get_row != points.begin() + start_rows + height; get_row++)
    {
      std::vector<Eigen::Vector3f>::iterator get_point;
      int j = 0;
      for (get_point = get_row->begin() + start_cols; get_point != get_row->begin() + start_cols + width; get_point++)
      {
        if (!std::isnan(get_point->z()))
        {
          getpoints.emplace_back(*get_point);
          getindexs.emplace_back(std::make_pair(start_rows + i, start_cols + j));
        }
        j++;
      }
      i++;
    }
  }

  orginazed_points Rect(size_t start_col, size_t start_row, size_t width_, size_t height_)
  {
    std::cout<<"Rect: "<<width_<<" "<<height_<<std::endl;
    orginazed_points return_points;
    return_points.width = width_;
    return_points.height = height_;
    return_points.points.reserve(height_);
    for (size_t i = 0; i < height_; i++)
    {
      std::vector<Eigen::Vector3f> i_row;
      i_row.reserve(width_);
      i_row.insert(i_row.end(), points.at(i + start_row).begin() + start_col, points.at(i + start_row).begin() + start_col + width_);
      assert(i_row.size() == width_);
      return_points.points.emplace_back(i_row);
    }
    return return_points;
  }

  void clear()
  {
    points.clear();
  }

// row_index: index in image
  Eigen::Vector3f getIndexPoint(size_t row_index, size_t col_index)
  {
    return points.at(row_index).at(col_index);
  }

  
};

struct orginazed_points_pcl
{
  pcl::PointCloud<pcl::PointXYZ> pointcloud;
  size_t width, height;
  orginazed_points_pcl()
  {
    
  }
  orginazed_points_pcl(pcl::PointCloud<pcl::PointXYZ> & pc)
  {
    pointcloud = pc;
    width = pc.width;
    height = pc.height;
  }

  orginazed_points_pcl(const orginazed_points_pcl &rhs)
  {
    width = rhs.width;
    height = rhs.height;
    pointcloud = rhs.pointcloud;
  }

//赋值函数
  orginazed_points_pcl& operator=(const orginazed_points_pcl &rhs)
  {
    if (this == &rhs)
      return *this;

    width = rhs.width;
    height = rhs.height;
    pointcloud = rhs.pointcloud;
    return *this;
  }

  /**
   * @description: 取矩形区域内的点云，存储到rectPoints
   * @param {size_t} start_xindex 列开始
   * @param {size_t} start_yindex 行开始
   * @param {size_t} width 列宽
   * @param {size_t} height 行宽
   * @param {PointCloud<pcl::PointXYZ>} rectPoints
   * @return {*}
   */  
  void getRectPoints(size_t start_xindex, size_t start_yindex, size_t width_, size_t height_, pcl::PointCloud<pcl::PointXYZ> &rectPoints)
  {
    std::cout<<"width_pc: "<<rectPoints.width<<"height_pc: "<<rectPoints.height<<std::endl;
    for (size_t i = 0; i < height_; i++)
    {
      size_t start_index = (start_yindex + i) * width + start_xindex;
      size_t end_index = (start_yindex + i) * width + start_xindex + width_;
      assert(end_index < pointcloud.size());
      rectPoints.points.insert(rectPoints.points.end(), pointcloud.begin() + start_index, pointcloud.begin() + end_index);
    }
    rectPoints.width = width; rectPoints.height = height;
  }

  void clear()
  {
    pointcloud.clear();
  }

  IndexPoints getRectPoint(size_t start_rows, size_t start_cols, size_t width_, size_t height_)
  {
    assert(start_rows < height);
    assert(start_cols < width);
    if (start_rows + height_ > height)
    {
      height_ = height - start_rows;
    }
    if (start_cols + width_ > width)
    {
      width_ = width - start_cols;
    }
    assert(start_rows + height_ <= height);
    assert(start_cols + width_ <= width);

    IndexPoints return_points;
    for (int row_index = start_rows; row_index < start_rows + height_; row_index++)
    {
      pcl::PointCloud<pcl::PointXYZ>::iterator iter_begin = pointcloud.begin() + (row_index * width + start_cols);
      pcl::PointCloud<pcl::PointXYZ>::iterator iter_end = pointcloud.begin() + (row_index * width + start_cols + width_);
      int col_index = start_cols;
      for (auto iter = iter_begin; iter < iter_end; iter++)
      {
        Eigen::Vector3f point(iter->x, iter->y, iter->z);
        IndexPoint tmppoint = std::make_pair(std::make_pair(row_index, col_index), point);
        return_points.emplace_back(tmppoint);
        col_index++;
      }
    }
    return return_points;
  }

  void getRectPointAndIndex(size_t start_rows, size_t start_cols, size_t width_, size_t height_, std::vector<Eigen::Vector3f> & getpoints, std::vector<std::pair<size_t, size_t>> & getindexs)
  {
    assert(start_rows < height);
    assert(start_cols < width);
    if (start_rows + height_ > height)
    {
      height_ = height - start_rows;
    }
    if (start_cols + width_ > width)
    {
      width_ = width - start_cols;
    }
    assert(start_rows + height_ <= height);
    assert(start_cols + width_ <= width);

    IndexPoints return_points;
    for (int row_index = start_rows; row_index < start_rows + height_; row_index++)
    {
      pcl::PointCloud<pcl::PointXYZ>::iterator iter_begin = pointcloud.begin() + (row_index * width + start_cols);
      pcl::PointCloud<pcl::PointXYZ>::iterator iter_end = pointcloud.begin() + (row_index * width + start_cols + width_);
      int col_index = start_cols;
      for (auto iter = iter_begin; iter < iter_end; iter++)
      {
        Eigen::Vector3f point(iter->x, iter->y, iter->z);
        getpoints.emplace_back(point);
        getindexs.emplace_back(std::make_pair(row_index, col_index));
        // IndexPoint tmppoint = std::make_pair(std::make_pair(row_index, col_index), point);
        // return_points.emplace_back(tmppoint);
        col_index++;
      }
    }
  }

  orginazed_points_pcl Rect(size_t start_cols, size_t start_rows, size_t width_, size_t height_)
  {
    assert(start_rows < height);
    assert(start_cols < width);
    if (start_rows + height_ > height)
    {
      height_ = height - start_rows;
    }
    if (start_cols + width_ > width)
    {
      width_ = width - start_cols;
    }
    assert(start_rows + height_ <= height);
    assert(start_cols + width_ <= width);

    // IndexPoints return_points;
    pcl::PointCloud<pcl::PointXYZ> return_points;
    for (int row_index = start_rows; row_index < start_rows + height_; row_index++)
    {
      pcl::PointCloud<pcl::PointXYZ>::iterator iter_begin = pointcloud.begin() + (row_index * width + start_cols);
      pcl::PointCloud<pcl::PointXYZ>::iterator iter_end = pointcloud.begin() + (row_index * width + start_cols + width_);
      // int col_index = start_cols;
      for (auto iter = iter_begin; iter < iter_end; iter++)
      {
        return_points.emplace_back(*iter);
      }
    }
    return_points.width = width_;
    return_points.height = height_;
    return return_points;
  }

  Eigen::Vector3f getIndexPoint(size_t row_index, size_t col_index)
  {
    // return points.at(row_index).at(col_index);
    // CHECK()
    pcl::PointXYZ point = pointcloud.at(row_index * width + col_index);
    return Eigen::Vector3f(point.x, point.y, point.z);
  }

};

struct parameter
{
  size_t leafnode_depth;
  size_t leafnode_width;
  float patch_num_percent_th;
  size_t patch_num_th;
  float patch_mse_th;
  float eigen_value_th;
  float merge_normal_dot_th;
  float merge_normal_distance;
  float quatree_merge_normal_dot_th;
  size_t quatree_width;

  parameter()
        : leafnode_depth(0), leafnode_width(0), patch_num_percent_th(0.0f), patch_num_th(0),
          patch_mse_th(0.0f), eigen_value_th(0.0f), merge_normal_dot_th(0.0f),
          merge_normal_distance(0.0f), quatree_merge_normal_dot_th(0.0f), quatree_width(0) {}

  parameter(const parameter &rhs)
  {
    leafnode_depth = rhs.leafnode_depth;
    leafnode_width = rhs.leafnode_width;
    patch_num_percent_th = rhs.patch_num_percent_th;
    patch_num_th = rhs.patch_num_th;
    patch_mse_th = rhs.patch_mse_th;
    eigen_value_th = rhs.eigen_value_th;
    merge_normal_dot_th = rhs.merge_normal_dot_th;
    merge_normal_distance = rhs.merge_normal_distance;
    quatree_merge_normal_dot_th = rhs.quatree_merge_normal_dot_th;
    quatree_width = rhs.quatree_width;
  }

//赋值函数
  parameter& operator=(const parameter &rhs)
  {
    if (this == &rhs)
      return *this;

    leafnode_depth = rhs.leafnode_depth;
    leafnode_width = rhs.leafnode_width;
    patch_num_percent_th = rhs.patch_num_percent_th;
    patch_num_th = rhs.patch_num_th;
    patch_mse_th = rhs.patch_mse_th;
    eigen_value_th = rhs.eigen_value_th;
    merge_normal_dot_th = rhs.merge_normal_dot_th;
    merge_normal_distance = rhs.merge_normal_distance;
    quatree_merge_normal_dot_th = rhs.quatree_merge_normal_dot_th;
    quatree_width = rhs.quatree_width;
    return *this;
  }


  void initial(size_t raw_points_width)
  {
    LOG(INFO)<<"raw_points_width: "<<raw_points_width<<std::endl;
    std::cout<<"raw_points_width: "<<raw_points_width<<std::endl;
    patch_num_th = patch_num_percent_th * leafnode_width * leafnode_width;
    // std::cout<<leafnode_width<<std::endl;
    // std::cout<<(double)raw_points_width/(double)leafnode_width<<std::endl;
    // std::cout<<std::log2((double)raw_points_width/(double)leafnode_width)<<std::endl;
    // std::cout<<std::ceil(log2(raw_points_width/leafnode_width))<<std::endl;
    leafnode_depth = std::ceil(log2((double)raw_points_width/(double)leafnode_width));
    std::cout<<"leafnode_depth: "<<leafnode_depth<<std::endl;
    quatree_width = std::pow(2, leafnode_depth) * leafnode_width;
  }

  void showParameter()
  {
    LOG(INFO)<<"leafnode_depth: "<<leafnode_depth;
    LOG(INFO)<<"leafnode_width: "<<leafnode_width;
    LOG(INFO)<<"patch_num_th: "<<patch_num_th;
    LOG(INFO)<<"patch_mse_th: "<<patch_mse_th;
    LOG(INFO)<<"eigen_value_th: "<<eigen_value_th;
    LOG(INFO)<<"merge_normal_dot_th: "<<merge_normal_dot_th;
    LOG(INFO)<<"merge_normal_distance: "<<merge_normal_distance;
    LOG(INFO)<<"quatree_merge_normal_dot_th: "<<quatree_merge_normal_dot_th;
    LOG(INFO)<<"quatree_width: "<<quatree_width;
  }
};
