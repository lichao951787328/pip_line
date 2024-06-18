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

#include <gtest/gtest.h>
#include "plane_detection/quatree_node.h"
#include "plane_detection/type.h"
#include <pcl/io/pcd_io.h>
#include "plane_detection/load_parameter.h"
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
class QuatreeNodeTest :public ::testing::Test
{
protected:
  void SetUp() override 
  {
    initialRawPoints(100, 200, 10, 8);
    load_parameter(param);
    param.initial(raw_points.width);
    std::cout<<"leafnode_depth: "<<param.leafnode_depth<<std::endl;
    param.showParameter();
    quatree::node::setStaticMember(10, 8, param.quatree_width);
    proot1 = new quatree::node(0, 0, param.quatree_width, 0, 0, param.quatree_width/param.leafnode_width, 0, nullptr);
  }
  quatree::node* proot1;

  quatree::node* proot2;

  quatree::node* proot3;
};

TEST_F(QuatreeNodeTest, initialnode)
{
  proot1->showNodeFrame();
  EXPECT_FALSE(false);
}