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
#include "plane_detection/load_parameter.h"
// #include "plane_detection/quadtree_new.h"
class OrginazedPointsPCLTest : public ::testing::Test
{
protected:
  void SetUp() override 
  {
    pcl::PointCloud<pcl::PointXYZ> cloud;
    pcl::io::loadPCDFile("/home/humanoid/work/plane_detection/catkin_plane_detection_test_ws/src/plane_detection/bag/cloud_out.pcd", cloud);
    pclPoints.pointcloud = cloud;
    pclPoints.width_pc = cloud.width;
    pclPoints.height_pc = cloud.height;
  }
  
  void TearDown()override
  {
    pclPoints.clear();
  }
  orginazed_points_pcl pclPoints;
};

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

class ParameterTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    p1.leafnode_width = 3;
    p1.patch_num_percent_th = 0.8;

    p2.leafnode_width = 3;
    p2.patch_num_percent_th = 0.8;

    p3.leafnode_width = 3;
    p3.patch_num_percent_th = 0.8;

    p4.leafnode_width = 3;
    p4.patch_num_percent_th = 0.8;

    p5.leafnode_width = 3;
    p5.patch_num_percent_th = 0.8;
  }

  parameter p1;
  parameter p2;
  parameter p3;
  parameter p4;
  parameter p5;
};

orginazed_points raw_points;
// parameter param;
void initialRawPoints(size_t start_col_, size_t start_row_, size_t width_, size_t height_)
{
  pcl::PointCloud<pcl::PointXYZ> cloud;
  pcl::io::loadPCDFile("/home/humanoid/work/plane_detection/catkin_plane_detection_test_ws/src/plane_detection/bag/cloud_out.pcd", cloud);
  orginazed_points raw_points_pcl;
  raw_points_pcl.initialByPCL(cloud);
  std::cout<<raw_points_pcl.width<<" "<<raw_points_pcl.height<<std::endl;
  std::cout<<raw_points_pcl.points.begin()->size()<<" "<<raw_points_pcl.points.size()<<std::endl;
  raw_points = raw_points_pcl.Rect(start_col_, start_row_, width_, height_);
}
// class Index2DTest : public ::testing::Test
// {
// protected:
//   void SetUp() override 
//   {
//     initialRawPoints(100, 200, 10, 8);
//     load_parameter(param);
//     param.initial(raw_points.width);
//     std::cout<<"leafnode_depth: "<<param.leafnode_depth<<std::endl;
//     param.showParameter();
//     quatree::node::setStaticMember(10, 8, param.quatree_width);
//     quatree::node* proot = new quatree::node(0, 0, param.quatree_width, 0, 0, param.quatree_width/param.leafnode_width, 0, nullptr);
//     proot->showNodeFrame();
//     size_t cols_2d = std::ceil((double)10/(double)param.leafnode_width);
//     size_t rows_2d = std::ceil((double)8/(double)param.leafnode_width);
//     pindex2d1 = new index2D(rows_2d, cols_2d, proot);

//     initialRawPoints(100, 200, 50, 40);
//     param.initial(raw_points.width);
//     std::cout<<"leafnode_depth: "<<param.leafnode_depth<<std::endl;
//     param.showParameter();
//     quatree::node::setStaticMember(50, 40, param.quatree_width);
//     quatree::node* proot2 = new quatree::node(0, 0, param.quatree_width, 0, 0, param.quatree_width/param.leafnode_width, 0, nullptr);
//     proot->showNodeFrame();
//     cols_2d = std::ceil((double)50/(double)param.leafnode_width);
//     rows_2d = std::ceil((double)40/(double)param.leafnode_width);
//     pindex2d2 = new index2D(rows_2d, cols_2d, proot2);
//   }
//   index2D* pindex2d1;
//   index2D* pindex2d2;
//   index2D* pindex2d3;
//   index2D* pindex2d4;
// };

// class levelsIndex2dTest : public ::testing::Test
// {
// protected:
//   void SetUp() override 
//   {
//     initialRawPoints(100, 200, 10, 8);
//     load_parameter(param);
//     param.initial(raw_points.width);
//     std::cout<<"leafnode_depth: "<<param.leafnode_depth<<std::endl;
//     param.showParameter();
//     quatree::node::setStaticMember(10, 8, param.quatree_width);
//     quatree::node* proot = new quatree::node(0, 0, param.quatree_width, 0, 0, param.quatree_width/param.leafnode_width, 0, nullptr);
//     proot->showNodeFrame();
//     size_t cols_2d = std::ceil((double)10/(double)param.leafnode_width);
//     size_t rows_2d = std::ceil((double)8/(double)param.leafnode_width);
//     plevelindex2d1 = new levelsIndex2d(proot);

//     initialRawPoints(100, 200, 50, 40);
//     param.initial(raw_points.width);
//     std::cout<<"leafnode_depth: "<<param.leafnode_depth<<std::endl;
//     param.showParameter();
//     quatree::node::setStaticMember(50, 40, param.quatree_width);
//     quatree::node* proot2 = new quatree::node(0, 0, param.quatree_width, 0, 0, param.quatree_width/param.leafnode_width, 0, nullptr);
//     proot->showNodeFrame();
//     cols_2d = std::ceil((double)50/(double)param.leafnode_width);
//     rows_2d = std::ceil((double)40/(double)param.leafnode_width);
//     plevelindex2d2 = new levelsIndex2d(proot2);
//   }

//   levelsIndex2d* plevelindex2d1;
//   levelsIndex2d* plevelindex2d2;
// };

TEST_F(OrginazedPointsPCLTest, GetRectPoints)
{
  std::cout<<"width: "<<pclPoints.width_pc<<" height: "<<pclPoints.height_pc<<std::endl;
  pcl::PointCloud<pcl::PointXYZ> rectPoints;
  pclPoints.getRectPoints(100, 200, 50, 40, rectPoints);
  EXPECT_EQ(rectPoints.size(), 2000);
  std::cout<<rectPoints.size()<<std::endl;
  for (size_t i = 0; i < 50; i++)
  {
    for (size_t j = 0; j < 40; j++)
    {
      EXPECT_EQ(pclPoints.pointcloud.at(100 + i, 200 + j).x, rectPoints.at(i, j).x);
      EXPECT_EQ(pclPoints.pointcloud.at(100 + i, 200 + j).y, rectPoints.at(i, j).y);
      EXPECT_EQ(pclPoints.pointcloud.at(100 + i, 200 + j).z, rectPoints.at(i, j).z);
    }
  }
}

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
  IndexPoints rectPoints = raw_points.getRectPoint(100, 200, 50, 40);
  ASSERT_LE(rectPoints.size(), 2000);
  std::cout<<"points size is "<<rectPoints.size()<<std::endl;
  IndexPoints::iterator iter_point = rectPoints.begin();
  for (size_t i = 0; i < 40; i++)// col
  {
    for (size_t j = 0; j < 50; j++)//row
    {
      if (!std::isnan(raw_points.points.at(100 + j).at(200 + i).z()))
      {
        ASSERT_EQ(raw_points.points.at(100 + i).at(200 + j).x(), iter_point->second.x());
        ASSERT_EQ(raw_points.points.at(100 + i).at(200 + j).y(), iter_point->second.y());
        ASSERT_EQ(raw_points.points.at(100 + i).at(200 + j).z(), iter_point->second.z());
        ASSERT_EQ(200 + j, iter_point->first.first);
        ASSERT_EQ(100 + i, iter_point->first.second);
        iter_point++;
      }
    }
  }
}

TEST_F(OrginazedPointsTest, Rect)
{
  orginazed_points rectPoints = raw_points.Rect(100, 200, 50, 40);
  std::cout<<rectPoints.points.begin()->size()<<" "<<rectPoints.points.size()<<std::endl;
  std::cout<<"get finish"<<std::endl;
  for (size_t i = 0; i < 40; i++)
  {
    for (size_t j = 0; j < 50; j++)
    {
      ASSERT_EQ(raw_points.points.at(200 + i).at(100 + j).x(), rectPoints.points.at(i).at(j).x());
      ASSERT_EQ(raw_points.points.at(200 + i).at(100 + j).y(), rectPoints.points.at(i).at(j).y());
      ASSERT_EQ(raw_points.points.at(200 + i).at(100 + j).z(), rectPoints.points.at(i).at(j).z());
    }
  }
}

TEST_F(ParameterTest, initial)
{
  p1.initial(1280);
  EXPECT_EQ(p1.leafnode_depth, 9);
  EXPECT_EQ(p1.quatree_width, 1536);

  p3.initial(720);
  EXPECT_EQ(p3.leafnode_depth, 8);
  EXPECT_EQ(p3.quatree_width, 768);

  p2.initial(640);
  EXPECT_EQ(p2.leafnode_depth, 8);
  EXPECT_EQ(p2.quatree_width, 768);

  p4.initial(480);
  EXPECT_EQ(p4.leafnode_depth, 8);
  EXPECT_EQ(p4.quatree_width, 768);

  p5.initial(50);
  EXPECT_EQ(p5.leafnode_depth, 5);
  EXPECT_EQ(p5.quatree_width, std::pow(2, 5) * p5.leafnode_width);
}

// TEST_F(Index2DTest, indextest)
// {
//   pindex2d1->showInfo();
//   pindex2d2->showInfo();
// }

// TEST_F(Index2DTest, getNeighbors)
// {
//   std::cout<<"index2d test get neighbors...."<<std::endl;
//   pindex2d1->showInfo();
//   for (size_t i = 0; i < pindex2d1->rows; i++)
//   {
//     for (size_t j = 0; j < pindex2d1->cols; j++)
//     {
//       quatree::node* tmpnode = pindex2d1->getNodeByIndex(i, j);
//       std::cout<<"tmp node address "<<tmpnode<<std::endl;
//       std::set<quatree::node*, quatree::compnode> neighbors;
//       pindex2d1->getNeighbors(tmpnode, neighbors);
//       std::cout<<" neighbors are: "<<endl;
//       for (auto & iter_neighbor : neighbors)
//       {
//         std::cout<<iter_neighbor<<std::endl;
//       }
//       std::cout<<std::endl;
//     }
//   }

//   initialRawPoints(100, 200, 10, 8);
//   load_parameter(param);
//   param.initial(raw_points.width);
//   std::cout<<"leafnode_depth: "<<param.leafnode_depth<<std::endl;
//   param.showParameter();
//   quatree::node::setStaticMember(10, 8, param.quatree_width);
//   quatree::quatree qq;
//   std::cout<<"1111111111111"<<std::endl;
//   qq.showFrame();
//   qq.mergePatchsForSeeds();
//   qq.showFrame();
//   size_t cols_2d = std::ceil((double)10/(double)param.leafnode_width);
//   size_t rows_2d = std::ceil((double)8/(double)param.leafnode_width);
//   pindex2d3 = new index2D(rows_2d, cols_2d, qq.getRoot());
//   pindex2d3->showInfo();

//   for (size_t i = 0; i < pindex2d3->rows; i++)
//   {
//     for (size_t j = 0; j < pindex2d3->cols; j++)
//     {
//       quatree::node* tmpnode = pindex2d3->getNodeByIndex(i, j);
//       std::cout<<"tmp node address "<<tmpnode<<std::endl;
//       std::set<quatree::node*, quatree::compnode> neighbors;
//       pindex2d3->getNeighbors(tmpnode, neighbors);
//       std::cout<<" neighbors are: "<<endl;
//       for (auto & iter_neighbor : neighbors)
//       {
//         std::cout<<iter_neighbor<<std::endl;
//       }
//       std::cout<<std::endl;
//     }
//   }
//   std::cout<<"22222222"<<std::endl;
//   initialRawPoints(150, 250, 20, 16);
//   param.initial(raw_points.width);
//   quatree::node::setStaticMember(20, 16, param.quatree_width);
//   quatree::quatree qq2;
//   qq2.mergePatchsForSeeds();
//   cols_2d = std::ceil((double)20/(double)param.leafnode_width);
//   rows_2d = std::ceil((double)16/(double)param.leafnode_width);
//   pindex2d4 = new index2D(rows_2d, cols_2d, qq2.getRoot());
//   pindex2d4->showInfo();

//   for (size_t i = 0; i < pindex2d4->rows; i++)
//   {
//     for (size_t j = 0; j < pindex2d4->cols; j++)
//     {
//       quatree::node* tmpnode = pindex2d4->getNodeByIndex(i, j);
//       std::cout<<"tmp node address "<<tmpnode<<std::endl;
//       std::set<quatree::node*, quatree::compnode> neighbors;
//       pindex2d4->getNeighbors(tmpnode, neighbors);
//       std::cout<<" neighbors are: "<<endl;
//       for (auto & iter_neighbor : neighbors)
//       {
//         std::cout<<iter_neighbor<<std::endl;
//       }
//       std::cout<<std::endl;
//     }
//   }
// }



// TEST_F(levelsIndex2dTest, levelindextest)
// {
//   std::cout<<"level index test to show node structure"<<std::endl;
//   plevelindex2d1->showInfo();
//   plevelindex2d2->showInfo();
// }
