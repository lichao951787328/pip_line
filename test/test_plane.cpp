/*
 * @description: 
 * @param : 
 * @return: 
 */
#include <gtest/gtest.h>
#include "plane_detection/type.h"
#include "plane_detection/load_parameter.h"
#include "plane_detection/quatree_node.h"
#include "plane_detection/quadtree_new.h"
#include "plane_detection/plane_new.h"
#include <pcl/io/pcd_io.h>


orginazed_points raw_points;
parameter param;
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

class planeTest : public :: testing::Test
{
protected:
  void SetUp() override 
  {

  }

  plane* plane1;
  plane* plane2;
  plane* plane3;
};

TEST_F(planeTest, constructor)
{
  initialRawPoints(100, 200, 10, 8);
  load_parameter(param);
  param.initial(raw_points.width);
  std::cout<<"leafnode_depth: "<<param.leafnode_depth<<std::endl;
  param.showParameter();
  quatree::node::setStaticMember(10, 8, param.quatree_width);
  quatree::quatree qq1;
  qq1.pindex2d->showInfo();
  qq1.showFrame();
  plane1 = new plane(qq1.getSeedNode());
  plane1->regionGrowing();

  // initialRawPoints(300, 200, 20, 16);
  // param.initial(raw_points.width);
  // // std::cout<<raw_points.width<<" "<<raw_points.height<<std::endl;
  // quatree::node::setStaticMember(20, 16, param.quatree_width);
  // quatree::quatree qq2;
  // qq2.pindex2d->showInfo();
  // plane2 = new plane(qq2.getSeedNode());
  // LOG(INFO)<<"PLANE2 REGION GROWING";
  // plane2->regionGrowing();
  // qq2.showFrame();
  // index2d_rows = std::ceil(double(16)/double(param.leafnode_width));
  // index2d_cols = std::ceil(double(20)/double(param.leafnode_width));
  // std::cout<<qq2.getRoot()->start_rows_2d + qq2.getRoot()->width_2d<<" "<<qq2.getRoot()->start_cols_2d + qq2.getRoot()->width_2d<<std::endl;
  // index2D index2d2(index2d_rows, index2d_cols, qq2.getRoot());
  // index2d2.showInfo();
  // plane2 = new plane(index2d_rows, index2d_cols, index2d2);
  // std::cout<<plane2->index_image.cols<<" "<<plane2->index_image.rows<<std::endl;
}

// TEST_F(planeTest, getNeighborNode)
// {
//   initialRawPoints(100, 200, 10, 8);
//   load_parameter(param);
//   param.initial(raw_points.width);
//   std::cout<<"leafnode_depth: "<<param.leafnode_depth<<std::endl;
//   param.showParameter();
//   quatree::node::setStaticMember(10, 8, param.quatree_width);
//   quatree::quatree qq1;
//   qq1.showFrame();
//   qq1.mergePatchsForSeeds();
//   qq1.showFrame();
//   size_t index2d_rows = std::ceil(double(8)/double(param.leafnode_width));
//   size_t index2d_cols = std::ceil(double(10)/double(param.leafnode_width));
//   index2D index2d(index2d_rows, index2d_cols, qq1.getRoot());
//   index2d.showInfo();
//   plane1 = new plane(index2d_rows, index2d_cols, index2d);
//   plane1->addNode2Plane(qq1.getSeedNode());
//   quatree::deleteNodeInQuatree(qq1.getSeedNode());
//   std::priority_queue<quatree::node*, std::vector<quatree::node*>, quatree::compnode> neighbors;
//   plane1->getNeighborNode(neighbors);
//   while (!neighbors.empty())
//   {
//     std::cout<<neighbors.top()<<std::endl;
//     neighbors.pop();
//   }

//   std::cout<<"qq3:`````````"<<std::endl;
//   quatree::quatree qq3;
//   index2D index2dqq3(index2d_rows, index2d_cols, qq3.getRoot());
//   index2dqq3.showInfo();
//   vector<quatree::node*> leafnodes = qq3.getLeafnodes();
//   plane3 = new plane(index2d_rows, index2d_cols, index2dqq3);
//   plane3->showImage();
//   std::cout<<"show delete node"<<std::endl;
//   for (auto & i : {1, 3, 4, 5, 7, 9, 10, 11})
//   {
//     std::cout<<leafnodes.at(i)<<std::endl;
//     plane3->setIndexImage(leafnodes.at(i));
//   }
//   plane3->showImage();
//   plane3->getNeighborNode(neighbors);
//   while (!neighbors.empty())
//   {
//     std::cout<<neighbors.top()<<std::endl;
//     neighbors.pop();
//   }

//   std::cout<<"plane 2 "<<std::endl<<std::endl;
//   initialRawPoints(300, 200, 20, 16);
//   param.initial(raw_points.width);
//   std::cout<<raw_points.width<<" "<<raw_points.height<<std::endl;
//   quatree::node::setStaticMember(20, 16, param.quatree_width);
//   quatree::quatree qq2;
//   // qq2.showFrame();
//   qq2.mergePatchsForSeeds();
//   index2d_rows = std::ceil(double(16)/double(param.leafnode_width));
//   index2d_cols = std::ceil(double(20)/double(param.leafnode_width));
//   std::cout<<qq2.getRoot()->start_rows_2d + qq2.getRoot()->width_2d<<" "<<qq2.getRoot()->start_cols_2d + qq2.getRoot()->width_2d<<std::endl;
//   index2D index2d2(index2d_rows, index2d_cols, qq2.getRoot());
//   index2d2.showInfo();
//   plane2 = new plane(index2d_rows, index2d_cols, index2d2);
//   LOG(INFO)<<"seed: "<<qq2.getSeedNode()<<std::endl;
//   LOG(INFO)<<"valid_points_size: "<<plane2->valid_points_size<<std::endl;

//   LOG(INFO)<<qq2.getSeedNode();
//   plane2->addNode2Plane(qq2.getSeedNode());

//   // 这个算不算两次删除，还是删除也不影响。。。
//   LOG(INFO)<<qq2.getSeedNode();
  
//   // quatree::deleteNodeInQuatree(qq2.getSeedNode());
//   plane2->getNeighborNode(neighbors);
//   while (!neighbors.empty())
//   {
//     std::cout<<neighbors.top()<<std::endl;
//     neighbors.pop();
//   }
// }

// TEST_F(planeTest, addNode2Plane)
// {
//   initialRawPoints(300, 200, 20, 16);
//   load_parameter(param);
//   param.initial(raw_points.width);
//   std::cout<<raw_points.width<<" "<<raw_points.height<<std::endl;
//   quatree::node::setStaticMember(20, 16, param.quatree_width);
//   quatree::quatree qq2;
//   // qq2.showFrame();
//   qq2.mergePatchsForSeeds();
//   size_t index2d_rows = std::ceil(double(16)/double(param.leafnode_width));
//   size_t index2d_cols = std::ceil(double(20)/double(param.leafnode_width));
//   index2D index2d2(index2d_rows, index2d_cols, qq2.getRoot());
//   index2d2.showInfo();
//   plane2 = new plane(index2d_rows, index2d_cols, index2d2);
//   LOG(INFO)<<"seed: "<<qq2.getSeedNode()<<std::endl;
//   plane2->addNode2Plane(qq2.getSeedNode());
//   std::priority_queue<quatree::node*, std::vector<quatree::node*>, quatree::compnode > neighbors;
//   plane2->getNeighborNode(neighbors);
//   while (!neighbors.empty())
//   {
//     quatree::node* tmpnode = neighbors.top();
//     neighbors.pop();
//     if (tmpnode->is_plane)
//     {
//       plane2->addNode2Plane(tmpnode);
//     }
//   }
// }

// TEST_F(planeTest, testNeighborsUsedGraph)
// {
//   initialRawPoints(300, 200, 20, 16);
//   load_parameter(param);
//   param.initial(raw_points.width);
//   std::cout<<raw_points.width<<" "<<raw_points.height<<std::endl;
//   quatree::node::setStaticMember(20, 16, param.quatree_width);
//   quatree::quatree qq2;
//   plane2 = new plane(qq2.getRoot());
  
// }
