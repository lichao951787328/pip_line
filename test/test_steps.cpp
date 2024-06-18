/*
 * @description: 
 * @param : 
 * @return: 
 */
#include <iostream>
#include "matplotlibcpp.h"
#include <fstream>
#include <vector>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <array>
namespace plt = matplotlibcpp;
using namespace std;
#define PI 3.1415926
array<Eigen::Vector3f, 4> computeStepMarker(Eigen::Vector4f step)
{
  array<Eigen::Vector3f, 4> Zero_step;
  Zero_step.at(0) = Eigen::Vector3f(0.15, 0.05, 0.0);
  Zero_step.at(1) = Eigen::Vector3f(0.15,  - 0.09, 0.0);
  Zero_step.at(2) = Eigen::Vector3f(- 0.09, - 0.09, 0.0);
  Zero_step.at(3) = Eigen::Vector3f(-0.09, 0.05, 0.0);

  Eigen::AngleAxisf r_v(step(3), Eigen::Vector3f(0,0,1));
  array<Eigen::Vector3f, 4> step_real;
  step_real.at(0) = r_v.matrix() * Zero_step.at(0) + step.head(3);
  step_real.at(1) = r_v.matrix() * Zero_step.at(1) + step.head(3);
  step_real.at(2) = r_v.matrix() * Zero_step.at(2) + step.head(3);
  step_real.at(3) = r_v.matrix() * Zero_step.at(3) + step.head(3);
  return step_real;
}

size_t step_index = 1;
void showSteps(vector<Eigen::Vector4f> steps)
{
  step_index = 1;
  for (auto & tmpstep : steps)
  {
    vector<float> x_, y_;
    cout<<tmpstep.transpose()<<endl;
    array<Eigen::Vector3f, 4> footCor = computeStepMarker(tmpstep);
    for (auto & point2d : footCor)
    {
      x_.emplace_back(point2d.x());
      y_.emplace_back(point2d.y());
      // cout<<"in for "<<point2d.transpose()<<endl;
    }
    x_.emplace_back(*x_.begin());
    y_.emplace_back(*y_.begin());
    plt::plot(x_, y_);
    plt::text(tmpstep.x(), tmpstep.y(), std::to_string(step_index));
    step_index++;
  }
  plt::set_aspect(1);
  plt::show();
}

int main(int argc, char** argv)
{
  ifstream inFile;
    
  inFile.open("/home/humanoid/work/plane_detection/catkin_plane_detection_test_ws/src/plane_detection/config/steps.txt");
  if (!inFile) {
    cout << "Unable to open file";
    exit(1); // terminate with error
  }
  else
  {
    cout<<"open files success"<<endl;
  }
  vector<Eigen::Vector4f> steps;
  Eigen::Vector4f step;
  float x;
  int index = 1;
  while (inFile >> x)
  {
    switch (index % 4)
    {
      case 1:
        step(0)= x;
        index ++;
        break;
      case 2:
        step(1) = x;
        index ++;
        break;
      case 3:
        step(2) = x;
        index ++;
        break;
      case 0:
        step(3) = x;
        index ++;
        steps.emplace_back(step);
        step.setZero();
        // cout<<"step: "<<step.transpose()<<endl;
        break;
      default:
        break;
    }
  }

  steps.pop_back();
  steps.pop_back();
  steps.pop_back();
  steps.pop_back();
  steps.pop_back();
  steps.pop_back();


  steps.pop_back();
  steps.pop_back();
  steps.pop_back();
  // steps.pop_back();
  // steps.pop_back();
  // steps.pop_back();

  // steps.pop_back();
  // steps.pop_back();
  // steps.pop_back();

  cout<<"add befor: "<<steps.size()<<endl;
  showSteps(steps);
  if (steps.size() % 2 == 0)// 补左脚
  {
    cout<<"left: "<<endl;
    cout<<"step angle "<<steps.back().w()<<endl;
    
    float angle = PI/2 + steps.back().w();
    cout<<"angle = "<<angle<<endl;
    Eigen::AngleAxisf r_v(angle, Eigen::Vector3f::UnitZ());
    Eigen::Vector3f normal = r_v.matrix() * Eigen::Vector3f::UnitX();
    Eigen::Vector3f left_cor = steps.back().head(3) + 0.16 * normal;
    steps.emplace_back(Eigen::Vector4f(left_cor(0), left_cor(1), left_cor(2), steps.back().w()));
  }
  else// 补右脚
  {
    cout<<"step angle "<<steps.back().w()<<endl;
    float angle = - PI/2 + steps.back().w();
    Eigen::AngleAxisf r_v(angle, Eigen::Vector3f::UnitZ());
    Eigen::Vector3f normal = r_v.matrix() * Eigen::Vector3f::UnitX();
    Eigen::Vector3f right_cor = steps.back().head(3) + 0.16 * normal;
    steps.emplace_back(Eigen::Vector4f(right_cor(0), right_cor(1), right_cor(2), steps.back().w()));
  }
  cout<<"add after: "<<steps.size()<<endl;
  showSteps(steps);  
  return 0;
}
