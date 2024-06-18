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
#ifndef _HEIGHTMAP_H_
#define _HEIGHTMAP_H_
#include <iostream>
#include <vector>
#include <Eigen/Core>
#include "opencv2/imgproc/imgproc.hpp"
#include <opencv2/highgui/highgui.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <Eigen/Core>
using namespace std;
// 设置heightmap的面积为3*3米，左右各1.5m，前方3m
// 分层，上一层表达出机器人上肢避障，第二层表达下层肯定要实行绕行的，这位部分其实并没有实际意义，最户一层表达可能可以跨过的障碍
// 对于洞穴检测，检测可通行平面区域上方0.7m内是否有障碍，有则该区域为爬行通过区域
namespace heightmap
{
  class heightmap
  {
  private:
    float mcell_size;
    size_t mgrid_dimensions;
    float mupper_body;
    float mround_walk;
    vector< vector<float> > heightMap;//确实需要的
    // vector< vector<bool> > round_obstacle_grid;
    Eigen::Affine3f transform2 = Eigen::Affine3f::Identity();
    cv::Mat upper_obstacle_grid;
    // cv::Mat round_obstacle_grid;
    // cv::Mat general_grid;
  public:
    heightmap(float cell_size, float upper_body, float round_walk, Eigen::Matrix4f & ROBOTWORLD_T_CAMERA);
    void initial(vector<Eigen::Vector3f> & ps);
    void initial_pcl(pcl::PointCloud<pcl::PointXYZ> & pc);
    vector<vector<Eigen::Vector2f>> get2Dcontours();
    inline vector< vector<float> > getHeightMap()
    {
      return heightMap;
    }
    ~heightmap();
  };
}

#endif