/*
 * @description: 
 * @param : 
 * @return: 
 */
#include <gtest/gtest.h>
#include <glog/logging.h>
#include <plane_detection/type.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>

class OrginazedPointsTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    pcl::PointCloud<pcl::PointXYZ> cloud;
    pcl::io::loadPCDFile("/home/humanoid/work/plane_detection/catkin_plane_detection_test_ws/src/plane_detection/bag/cloud_out.pcd", cloud);
    raw_points.initialByPCL(cloud);
    std::cout<<"width "<<raw_points.width<<" height "<<raw_points.height<<std::endl;
  }

  void TearDown()override
  {
    raw_points.clear();
  }

  orginazed_points raw_points;
};

TEST_F(OrginazedPointsTest, initialByPCL)
{
  pcl::PointCloud<pcl::PointXYZ> cloud;
  pcl::io::loadPCDFile("/home/humanoid/work/plane_detection/catkin_plane_detection_test_ws/src/plane_detection/bag/cloud_out.pcd", cloud);
  EXPECT_EQ(cloud.height, raw_points.height);
  EXPECT_EQ(cloud.width, raw_points.width);
  std::cout<<"cloud width "<<cloud.width<<" height "<<cloud.height<<std::endl;
  std::cout<<"raw point points "<<raw_points.points.size()<<endl;
  
  for (size_t i = 0; i < cloud.height; i++)
  {
    for (size_t j = 0; j < cloud.width; j++)
    {
      EXPECT_EQ(raw_points.points.at(i).at(j).x(), cloud.at(j, i).x);
      EXPECT_EQ(raw_points.points.at(i).at(j).y(), cloud.at(j, i).y);
      EXPECT_EQ(raw_points.points.at(i).at(j).z(), cloud.at(j, i).z);
    }
  }
}

TEST_F(OrginazedPointsTest, getRectPoint)
{

  std::cout<<"get rect points"<<std::endl;
  // 行 列 宽 高
  IndexPoints rectPoints = raw_points.getRectPoint(100, 200, 50, 40);
  ASSERT_LE(rectPoints.size(), 2000);
  std::cout<<"points size is "<<rectPoints.size()<<std::endl;
  IndexPoints::iterator iter_point = rectPoints.begin();
  for (size_t i = 0; i < 40; i++)// row
  {
    for (size_t j = 0; j < 50; j++)// col
    {
      if (!std::isnan(raw_points.points.at(100 + j).at(200 + i).z()))
      {
        ASSERT_EQ(raw_points.points.at(100 + i).at(200 + j).x(), iter_point->second.x());
        ASSERT_EQ(raw_points.points.at(100 + i).at(200 + j).y(), iter_point->second.y());
        ASSERT_EQ(raw_points.points.at(100 + i).at(200 + j).z(), iter_point->second.z());
        ASSERT_EQ(200 + j, iter_point->first.second);
        ASSERT_EQ(100 + i, iter_point->first.first);
        iter_point++;
      }
    }
  }
}