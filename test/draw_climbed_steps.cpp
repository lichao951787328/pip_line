/*
 * @description: 
 * @param : 
 * @return: 
 */
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <iostream>
#include <vector>
#include <Eigen/Core>
#include <Eigen/Geometry>
using namespace std;
struct footstep
{
  bool is_left;
  double x;
  double y;
  double z;
  double theta;
  footstep() {}
  footstep(bool is_left_, double x_, double y_, double z_, double theta_)
  {
      is_left = is_left_; x = x_; y = y_; theta = theta_; z = z_;
  }
  footstep& operator =(const footstep &a)
	{
		is_left = a.is_left;
    x = a.x;
    y = a.y;
    z = a.z;
    theta = a.theta;
		return *this;
	}
};

Eigen::Matrix4f getT(const float &px, const float &py, const float &pz, const float &rx, const float &ry, const float &rz)
{
  using namespace Eigen;
  Matrix4f res;
  res.setIdentity();
  res.block<3,3>(0,0) = (AngleAxisf(rz, Vector3f::UnitZ())*AngleAxisf(ry, Vector3f::UnitY())*AngleAxisf(rx, Vector3f::UnitX())).matrix();
  res(0,3) = px;
  res(1,3) = py;
  res(2,3) = pz;
  return res;
}

double _deg2rad(double degree)
{
  double rad = degree/57.3;
  return rad;
}
Eigen::Matrix4f ROBOTWORLD_T_CAMERA;
cv::Mat draw_image;
void initial_matrix()
{
  float BaseVisionZ = +1.1258e+00-+7.1365e-01+0.18938;
  BaseVisionZ -= (77.51 - 12.81)/1000;
  float BaseVisionX = 0.15995;
  BaseVisionX -= (82.4 - 65.17)/1000;
  float BaseVisionY = 0.0;
  float BaseVisionPitchDeg = 27.5;
  Eigen::Matrix4f Base_T_Vision, Vision_T_Tar, World_T_Base, World_T_Tar;
  Base_T_Vision = getT(BaseVisionX, BaseVisionY, BaseVisionZ, 0, 0, 0);
  Eigen::Matrix3f Base_R_VisionTemp;
  Base_R_VisionTemp << 0,-1,0, -1,0,0, 0,0,-1;
  Base_T_Vision.block<3,3>(0,0) = Base_R_VisionTemp*(Eigen::AngleAxisf(_deg2rad(BaseVisionPitchDeg),Eigen::Vector3f::UnitX())).matrix();
  // World_T_Base = getT(-0.015, 0, 0.65, 0.0, 0.0, 0.0);
  World_T_Base = getT(-0.0, 0, 0.8, 0.0, 0.0, 0.0);
  Vision_T_Tar.setIdentity();
  World_T_Tar = World_T_Base*Base_T_Vision*Vision_T_Tar;
  ROBOTWORLD_T_CAMERA = World_T_Tar;

  // Eigen::Matrix4f MARKER_T_INITWORLD;
  // // marker相对于初始世界坐标系的位姿 = 实时marker位姿、初始检测时刻marker相对于世界坐标系的位姿、marker相对于机器人初始世界坐标系的位姿
  // MARKER_T_INITWORLD = ARTWORLD_T_MARKER.inverse() * initial_ARTWORLD_T_MARKER * ROBOTWORLD_T_MARKER.inverse();
  
  // initROBOTWORLD_T_CAMERA = MARKER_T_INITWORLD.inverse() * MARKER_T_CAMERA;
  // // draw_test();
  // draw_steps(steps);
  // draw_rect();
}

std::vector<float> colors_ = {
      51, 160, 44,  //0
      166, 206, 227 ,
      178 , 223 , 138 ,//6
      31, 120 , 180 ,
      251 , 154 , 153 ,// 12
      227 , 26 , 28 ,
      253 , 191 , 111 ,// 18
      106 , 61 , 154 ,
      255 , 127 , 0 , // 24
      202 , 178 , 214 ,
      255, 0.0, 0.0, // red // 30
      0.0, 255, 0.0, // green
      0.0, 0.0, 255, // blue// 36
      255, 255, 0.0,
      255, 0.0, 255, // 42
      0.0, 255, 255,
       177.5, 255, 0.0,
      255, 177.5, 0.0,
       177.5, 0.0, 255,
      255, 0.0,  177.5,
      0.0,  177.5, 255,
      0.0, 255,  177.5,
      255,  177.5,  177.5,
       177.5, 255,  177.5,
       177.5,  177.5, 255,
       177.5,  177.5, 255,
       177.5, 255,  177.5,
       177.5,  177.5, 255};

vector<Eigen::Vector3f> computeStepMarker(footstep& f)
{
  vector<Eigen::Vector3f> return_v;
  vector<Eigen::Vector3f> v;
  v.reserve(4); return_v.reserve(4);
  Eigen::Vector3f v1(0.15, 0.05, 0.0);
  Eigen::Vector3f v2(0.15,  - 0.09, 0.0);
  Eigen::Vector3f v3(- 0.09, - 0.09, 0.0);
  Eigen::Vector3f v4(-0.09, 0.05, 0.0);
  v.emplace_back(v1);
  v.emplace_back(v2);
  v.emplace_back(v3);
  v.emplace_back(v4);
  Eigen::AngleAxisf r_v(f.theta, Eigen::Vector3f(0,0,1));
  for (auto & iter : v)
  {
    return_v.emplace_back(r_v.matrix()*iter + Eigen::Vector3f(f.x, f.y, f.z));
  }
  return return_v;
}

std::pair<size_t, size_t> Word2Pixel(Eigen::Vector3f & p)
{
  Eigen::Vector3f point_vision = p;
  // const double kFx = 456.67578125;
  // const double kFy = 457.3671875;
  // const double kCx = 310.76171875;
  // const double kCy = 246.896484375;

  const double kFx = 896.8710327148438;
  const double kFy = 896.7228393554688;
  const double kCx = 660.7276000976562;
  const double kCy = 362.8913879394531;
// const int kDepthWidth = 1280;
// const int kDepthHeight = 720;
  // 像素
  size_t j = point_vision.x() * kFx/point_vision.z() + kCx;
  size_t i = point_vision.y() * kFy/point_vision.z() + kCy;
  return std::make_pair(i, j);
}

std::vector<std::pair<size_t, size_t>> PointsWord2Pixel(std::vector<Eigen::Vector3f> ps)
{
  std::vector<std::pair<size_t, size_t>> return_pixels;
  for (auto & iter_point : ps)
  {
    return_pixels.emplace_back(Word2Pixel(iter_point));
  }
  return return_pixels;
}

void draw_steps(vector<footstep>& steps)
{
  // 点坐标转换
  // 这个点是初始世界坐标系下的点
  // 初始世界坐标系转为相机坐标系
  // maker相对与相机的位姿、当前marker相对于初始世界坐标系下的位姿
  // maker相对与相机的位姿 marker相对于当前世界坐标系的位姿 相机相对于当前世界坐标系的位姿，检测时可以计算得到
  // 当前marker相对于初始世界坐标系下的位姿 当前marker相对于art的位姿 初始marker相对于art的位姿 marker相对于初始世界坐标系下的位
  std::cout<<"enter draw steps"<<std::endl;
  size_t index = 1;
  for (auto & iter_step : steps)
  {

    cout<<"step: "<<iter_step.x<<" "<<iter_step.y<<" "<<iter_step.z<<" "<<iter_step.theta<<endl;
    vector<Eigen::Vector3f> step = computeStepMarker(iter_step);

    vector<Eigen::Vector3f> step_inVersion;
    for (auto & iter_point : step)
    {
      Eigen::Vector4f point; 
      point.head(3) = iter_point;
      point(3) = 1;
      step_inVersion.emplace_back((ROBOTWORLD_T_CAMERA.inverse()*point).head(3));
    }

    vector<std::pair<size_t, size_t>> pixels = PointsWord2Pixel(step_inVersion);
    std::vector<cv::Point> fillContSingle;
    //add all points of the contour to the vector
    for (auto & iter_pixel : pixels)
    {
      std::cout<<iter_pixel.first<<" "<<iter_pixel.second<<std::endl;
      // fillContSingle.push_back(cv::Point(iter_pixel.first, iter_pixel.second));
      fillContSingle.push_back(cv::Point(iter_pixel.second, iter_pixel.first));
    }
    std::vector<std::vector<cv::Point> > fillContAll;
    //fill the single contour 
    //(one could add multiple other similar contours to the vector)
    fillContAll.push_back(fillContSingle);
    // std::cout<<"at draw steps: "<<draw_image.cols<<" "<<draw_image.rows<<std::endl;
    if (iter_step.is_left)
    {
      cv::fillPoly(draw_image, fillContAll, cv::Scalar(255, 0, 0));

      // cv::polylines(draw_image, fillContAll, true, cv::Scalar(51, 160, 44), -1); 
    }
    else
    {
      cv::fillPoly(draw_image, fillContAll, cv::Scalar(0, 0, 255));
      // cv::polylines(draw_image, fillContAll, true, cv::Scalar(251, 154, 153), -1); 
    }
    
    // cv::polylines(draw_image, fillContAll, true, cv::Scalar(colors_.at(15), colors_.at(16), colors_.at(17)), -1);
    // cv::polylines(draw_image, fillContAll, true, cv::Scalar(colors_.at(15), colors_.at(16), colors_.at(17)), 5);
    // Eigen::Vector3f center_w(iter_step.x, iter_step.y, iter_step.z);
    // Eigen::Vector4f center_txt(iter_step.x, iter_step.y, iter_step.z, 1);
    // Eigen::Vector3f center_txt_pixel = (ROBOTWORLD_T_CAMERA.inverse() * center_txt).head(3);
    // std::pair<size_t, size_t> center = Word2Pixel(center_txt_pixel);
    // cv::putText(draw_image, std::to_string(index), cv::Point(center.second, center.first), cv::FONT_HERSHEY_SIMPLEX, 1.0, cv::Scalar(colors_.at(9), colors_.at(10), colors_.at(11)), 2);
    // index++;
  }
}

int main(int argc, char** argv)
{
  draw_image = cv::imread("/home/humanoid/work/plane_detection/catkin_plane_detection_test_ws/src/plane_detection/bag/climbed_steps_new/segmented_contours copy.png");
  cv::imshow("read_image", draw_image);
  cv::waitKey(0);
  // cv::Mat Gaussian_filter_image;
  // cv::GaussianBlur(draw_image, Gaussian_filter_image, cv::Size(15, 15), 1, 1);
  // cv::imshow("Gaussian_filter_image", Gaussian_filter_image);
  // cv::waitKey(0);
  initial_matrix();
  // // 通过调整脚的位置，并将脚投影到像素坐标系
  vector<footstep> steps;
  // footstep tmpstep1(false, 0.52, -0.18, 0, 0);
  // footstep tmpstep2(true, 0.52, 0.12, 0, 0);

  footstep tmpstep1(false, 0.21, -0.14, 0, 0);
  footstep tmpstep2(true, 0.21, 0.08, 0, 0);

  footstep tmpstep3(false, 0.49, -0.14, 0.1, 0);
  footstep tmpstep4(true, 0.75, 0.08, 0.2, 0);

  footstep tmpstep5(false, 1.01, -0.14, 0.3, 0);
  footstep tmpstep6(true, 1.01, 0.08, 0.3, 0);


  // footstep tmpstep3(false, 0.8, 0.0, 0.1, 0);
  // footstep tmpstep4(true, 1.1, 0.3, 0.2, 0);
  // footstep tmpstep5(false, 1.4, 0.0, 0.3, 0);
  // footstep tmpstep6(true, 1.4, 0.3, 0.3, 0);
  steps.emplace_back(tmpstep1);
  steps.emplace_back(tmpstep2);
  steps.emplace_back(tmpstep3);
  steps.emplace_back(tmpstep4);
  steps.emplace_back(tmpstep5);
  steps.emplace_back(tmpstep6);

  draw_steps(steps);
  cv::imshow("image", draw_image);
  
  cv::waitKey(0);
  cv::imwrite("/home/humanoid/work/plane_detection/catkin_plane_detection_test_ws/src/plane_detection/bag/climbed_steps_new/planed_steps_with_contours.png", draw_image);
  return 0;
}