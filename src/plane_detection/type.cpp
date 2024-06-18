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
#include<plane_detection/type.h>

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

float correct2threeDecimal(float x)
{
  return (int(std::round(x*100)))/((float)100);
}