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
#ifndef _MAP_FOR_BIPEDROBOT_H_
#define _MAP_FOR_BIPEDROBOT_H_
#include <plane_detection/plane_segmentation.h>
#include <plane_detection/heightmap.h>
#include <vector>
#include <boost/geometry.hpp>
#include <boost/geometry/geometries/point_xy.hpp>
#include <boost/geometry/geometries/polygon.hpp>
#include <Eigen/Core>
using namespace std;
// 将平面检测的平面和上肢防撞平面进行切割
typedef boost::geometry::model::d2::point_xy<float> point_t;
// polygon 可以指定外轮廓的旋转方向，同时根据外轮廓方向可以得到内轮廓方向
// 由于opencv得到的外轮廓是逆时针，内轮廓是顺时针的
typedef boost::geometry::model::polygon<point_t, false> polygon;
// 是不是可以先进行凹多边形凸化，再投影回原斜面上
vector<plane_info> getLandedArea(vector<plane_info>& planes, vector<vector<Eigen::Vector2f>> obstacles, Eigen::Matrix4f & ROBOTWORLD_T_CAMERA);


#endif
