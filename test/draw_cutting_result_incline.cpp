/*
 * @description: 
 * @param : 
 * @return: 
 */
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/visualization/cloud_viewer.h>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <vector>
#include <iostream>

using namespace std;

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
  World_T_Base = getT(-0.015, 0, 0.65, 0.0, 0.0, 0.0);
  // World_T_Base = getT(-0.0, 0, 0.8, 0.0, 0.0, 0.0);
  Vision_T_Tar.setIdentity();
  World_T_Tar = World_T_Base*Base_T_Vision*Vision_T_Tar;
  ROBOTWORLD_T_CAMERA = World_T_Tar;
}

// void FitCenterByLeastSquares(std::map<int, std::vector<double>> mapPoint, std::vector<double> &centerP, double &radius)
// {
// 	double sumX = 0, sumY = 0;
// 	double sumXX = 0, sumYY = 0, sumXY = 0;
// 	double sumXXX = 0, sumXXY = 0, sumXYY = 0, sumYYY = 0;
// 	for (std::map<int, std::vector<double>>::iterator it = mapPoint.begin(); it != mapPoint.end(); ++it)
// 	{
// 		std::vector<double> p = it->second;
// 		sumX += p[0];
// 		sumY += p[1];
// 		sumXX += p[0] * p[0];
// 		sumYY += p[1] * p[1];
// 		sumXY += p[0] * p[1];
// 		sumXXX += p[0] * p[0] * p[0];
// 		sumXXY += p[0] * p[0] * p[1];
// 		sumXYY += p[0] * p[1] * p[1];
// 		sumYYY += p[1] * p[1] * p[1];
// 	}
// 	int pCount = mapPoint.size();
// 	double M1 = pCount * sumXY - sumX * sumY;
// 	double M2 = pCount * sumXX - sumX * sumX;
// 	double M3 = pCount * (sumXXX + sumXYY) - sumX * (sumXX + sumYY);
// 	double M4 = pCount * sumYY - sumY * sumY;
// 	double M5 = pCount * (sumYYY + sumXXY) - sumY * (sumXX + sumYY);
// 	double a = (M1 * M5 - M3 * M4) / (M2*M4 - M1 * M1);
// 	double b = (M1 * M3 - M2 * M5) / (M2*M4 - M1 * M1);
// 	double c = -(a * sumX + b * sumY + sumXX + sumYY) / pCount;
// 	//圆心XY 半径
// 	double xCenter = -0.5*a;
// 	double yCenter = -0.5*b;
// 	radius = 0.5 * sqrt(a * a + b * b - 4 * c);
// 	centerP[0] = xCenter;
// 	centerP[1] = yCenter;
// }

void FitCenterByLeastSquares(vector<Eigen::Vector2f> & points2d, Eigen::Vector2f & center, float & radius)
{
  float sumX = 0, sumY = 0;
	float sumXX = 0, sumYY = 0, sumXY = 0;
	float sumXXX = 0, sumXXY = 0, sumXYY = 0, sumYYY = 0;
  for (auto & point2d : points2d)
  {
    sumX += point2d.x();
		sumY += point2d.y();
		sumXX += point2d.x() * point2d.x();
		sumYY += point2d.y() * point2d.y();
		sumXY += point2d.x() * point2d.y();
		sumXXX += point2d.x() * point2d.x() * point2d.x();
		sumXXY += point2d.x() * point2d.x() * point2d.y();
		sumXYY += point2d.x() * point2d.y() * point2d.y();
		sumYYY += point2d.y() * point2d.y() * point2d.y();
  }
  int pCount = points2d.size();
  float M1 = pCount * sumXY - sumX * sumY;
	float M2 = pCount * sumXX - sumX * sumX;
	float M3 = pCount * (sumXXX + sumXYY) - sumX * (sumXX + sumYY);
	float M4 = pCount * sumYY - sumY * sumY;
	float M5 = pCount * (sumYYY + sumXXY) - sumY * (sumXX + sumYY);
	float a = (M1 * M5 - M3 * M4) / (M2*M4 - M1 * M1);
	float b = (M1 * M3 - M2 * M5) / (M2*M4 - M1 * M1);
	float c = -(a * sumX + b * sumY + sumXX + sumYY) / pCount;

  float xCenter = -0.5*a;
	float yCenter = -0.5*b;
	radius = 0.5 * sqrt(a * a + b * b - 4 * c);
	center.x() = xCenter;
	center.y() = yCenter;
}

bool pointInConvexPolygon(Eigen::Vector2f & p, vector<Eigen::Vector2f> & polygon)
{
	int i, iNext, i2Next;
	float preCross, nextCross;
	Eigen::Vector2f v1, v2, v3;
	int polySize= polygon.size();
	
	if(polySize < 3)
	{
		return false;
	}
	for(i = 0; i < polySize; i++)
	{
		iNext = (i + 1) %  polySize;
		i2Next = (iNext + 1) % polySize;
	    
	  //注意v1, v2, v3最好归一化一下，防止图像坐标过大，导致叉乘结果溢出
    v1 = polygon.at(i) - p;
    v2 = polygon.at(iNext) - p;
    Eigen::Vector3f v1_3d(v1.x(), v1.y(), 0);
    Eigen::Vector3f v2_3d(v2.x(), v2.y(), 0);
		preCross = v1_3d.cross(v2_3d).z();
		
    v3 = polygon.at(i2Next) - p;
		Eigen::Vector3f v3_3d(v3.x(), v3.y(), 0);
		nextCross = v2_3d.cross(v3_3d).z();
		
		if(preCross * nextCross < 0)
		{
			return false;
		}
	}
	return true;
}

int main(int argc, char** argv)
{
  // 读取数据
  initial_matrix();
  pcl::PointCloud<pcl::PointXYZ> cloud;
  pcl::io::loadPCDFile("/home/humanoid/work/plane_detection/catkin_plane_detection_test_ws/src/plane_detection/bag/inclined_plane/discard-rows 100cols 1000align2.pcd", cloud);
  cout<<cloud.width<<" "<<cloud.height<<endl;
  pcl::visualization::CloudViewer viewer("cloud");
  // viewer.showCloud(cloud.makeShared());
  // while (!viewer.wasStopped())
	// {
	// }
  // system("read -p 'Press Enter to continue...' var");
  // 转到机器人世界坐标系
  pcl::PointCloud<pcl::PointXYZ> cloud_in_world;
  cloud_in_world.height = cloud.height;
  cloud_in_world.width = cloud.width;
  for (auto & point : cloud)
  {
    Eigen::Vector4f p(point.x, point.y, point.z, 1);
    Eigen::Vector3f p_world = (ROBOTWORLD_T_CAMERA*p).head(3);
    cloud_in_world.emplace_back(pcl::PointXYZ(p_world.x(), p_world.y(), p_world.z()));
  }
  
  // viewer.showCloud(cloud_in_world.makeShared());
  // while (!viewer.wasStopped())
	// {
	// }
  // system("read -p 'Press Enter to continue...' var");

  pcl::PointCloud<pcl::PointXYZ> right_cy;
  for (auto & point : cloud_in_world)
  {
    if (point.z > 0.1 && point.z < 1.5)
    {
      if (point.y < 0)
      {
        right_cy.emplace_back(point);
      }
    }
  }
  // viewer.showCloud(left_cy.makeShared());
  // while (!viewer.wasStopped())
	// {
	// }
  // system("read -p 'Press Enter to continue...' var");

  // viewer.showCloud(right_cy.makeShared());
  // while (!viewer.wasStopped())
	// {
	// }
  // system("read -p 'Press Enter to continue...' var");
  vector<Eigen::Vector2f> right_cy_2d;

  for (auto & point : right_cy)
  {
    right_cy_2d.emplace_back(Eigen::Vector2f(point.x, point.y));
  }

  Eigen::Vector2f center_right;
  float radius_right;
  // // FitCenterByLeastSquares(left_cy_2d, center_left, radius_left);
  FitCenterByLeastSquares(right_cy_2d, center_right, radius_right);

  vector<Eigen::Vector2f> polygen_right;
  // 生成圆上的点

  for (size_t i = 0; i <= 360; i += 20)
  {
    float x = (radius_right + 0.18) * cos(_deg2rad(i));
    float y = (radius_right + 0.18) * sin(_deg2rad(i));
    polygen_right.emplace_back(Eigen::Vector2f(center_right.x() + x, center_right.y() + y));
  }


  for (auto & point : cloud)
  {
    Eigen::Vector4f p(point.x, point.y, point.z, 1);
    Eigen::Vector2f p_world2d = (ROBOTWORLD_T_CAMERA*p).head(2);
    // bool cond1 = (p_world2d - center_left).norm() > radius_left + 0.18;
    // bool cond2 = (p_world2d - center_right).norm() > radius_right + 0.18;
    // bool cond1 = !pointInConvexPolygon(p_world2d, polygen_left);
    bool cond2 = !pointInConvexPolygon(p_world2d, polygen_right);
    // bool cond2 = (p_world2d - center_right).norm() > radius_right + 0.18;
    if ((!cond2))
    {
      point.x = 0;
      point.y = 0;
      point.z = 0;
    }
  }
  pcl::io::savePCDFile("/home/humanoid/work/plane_detection/catkin_plane_detection_test_ws/src/plane_detection/bag/inclined_plane/cuting_discard.pcd", cloud);
  viewer.showCloud(cloud.makeShared());
  while (!viewer.wasStopped())
	{
	}
  system("read -p 'Press Enter to continue...' var");
  // 生成圆上的点
  // for (size_t i = 0; i < 360; i++)
  // {
  //   float x = radius * cos(_deg2rad(i));
  //   float y = radius * sin(_deg2rad(i));
  //   left_cy.emplace_back(pcl::PointXYZ(center.x() + x, center.y() + y, 0));
  // }
  
  // viewer.showCloud(left_cy.makeShared());
  // while (!viewer.wasStopped())
	// {
	// }
  // system("read -p 'Press Enter to continue...' var");

  // 拟合圆柱

  // 向外扩充

  // 去掉那些在圆柱内的点云

  return 0;
}