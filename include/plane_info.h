#pragma once
#include <Eigen/Core>
#include <vector>
using namespace std;

struct plane_info
{
  Eigen::Vector3f center;
  Eigen::Vector3f normal;
  std::vector<std::vector<Eigen::Vector3f>> contours;
  std::vector< std::array<int, 4> > hierarchy;
  plane_info transform(Eigen::Matrix4f & T)
  {
    plane_info tmpplane;
    tmpplane.hierarchy = hierarchy;
    Eigen::Vector4f center_H;
    center_H.head(3) = center; center_H(3) = 1;
    tmpplane.center = (T * center_H).head(3);

    tmpplane.normal = T.block<3,3>(0,0) * normal;

    for (auto & contour : contours)
    {
      vector<Eigen::Vector3f> contour_T;
      contour_T.reserve(contour.size());
      for (auto & point : contour)
      {
        // std::cout<<"befor transform: "<<point.transpose()<<endl;
        Eigen::Vector4f point_H;
        point_H.head(3) = point; point_H(3) = 1;
        // std::cout<<"after transfrom: "<<(T*point_H).head(3).transpose()<<endl;
        contour_T.emplace_back((T * point_H).head(3));
      }
      tmpplane.contours.emplace_back(contour_T);
    }
    return tmpplane;
  }
  double getZ(Eigen::Vector2d xy)
  {
    Eigen::Vector2f xy_(xy.x(), xy.y());
    double d = -center.dot(normal);
    return (-d - xy_.dot(normal.head(2)))/normal(2);
  }
  double getZ(double x, double y)
  {
    return getZ(Eigen::Vector2d(x, y));
  }
};