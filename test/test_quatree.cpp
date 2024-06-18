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
class QuatreeTest : public ::testing::Test
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
    qq1 = new quatree::quatree();
    qq3 = new quatree::quatree();
    qq4 = new quatree::quatree();

    initialRawPoints(100, 200, 20, 17);
    param.initial(raw_points.width);
    std::cout<<"leafnode_depth: "<<param.leafnode_depth<<std::endl;
    param.showParameter();
    quatree::node::setStaticMember(20, 17, param.quatree_width);
    qq2 = new quatree::quatree();
  }
  quatree::quatree* qq1;

  quatree::quatree* qq2;

  quatree::quatree* qq3;

  quatree::quatree* qq4;

};

TEST_F(QuatreeTest, mergeTest)
{
  size_t rows_ = std::ceil((double)8/(double)param.leafnode_width);
  size_t cols_ = std::ceil(double(10)/(double)param.leafnode_width);
  index2D index2d_before(rows_, cols_, qq1->getRoot());
  index2d_before.showInfo();
  qq1->mergePatchsForSeeds();
  std::cout<<std::endl;
  index2D index2d_after(rows_, cols_, qq1->getRoot());
  index2d_after.showInfo();
  
  rows_ = std::ceil((double)17/(double)param.leafnode_width);
  cols_ = std::ceil(double(20)/(double)param.leafnode_width);
  index2D index2d_before2(rows_, cols_, qq2->getRoot());
  index2d_before2.showInfo();
  qq2->mergePatchsForSeeds();
  std::cout<<std::endl;
  index2D index2d_after2(rows_, cols_, qq2->getRoot());
  index2d_after2.showInfo();
}

TEST_F(QuatreeTest, getSeedNode)
{
  qq1->mergePatchsForSeeds();
  quatree::node* seed = qq1->getSeedNode();
  std::list<quatree::node*> plane_nodes = qq1->getPatchsList();
  quatree::node* candSeed = plane_nodes.back();
  for (auto & iter_node : plane_nodes)
  {
    if (iter_node->depth < candSeed->depth)
    {
      candSeed = iter_node;
    }
    else if (iter_node->depth == candSeed->depth)
    {
      if (iter_node->mse < candSeed->mse)
      {
        candSeed = iter_node;
      }
    }
  }
  EXPECT_EQ(seed, candSeed);

  qq2->mergePatchsForSeeds();
  seed = qq2->getSeedNode();
  plane_nodes.clear();
  plane_nodes = qq2->getPatchsList();
  candSeed = plane_nodes.back();
  for (auto & iter_node : plane_nodes)
  {
    if (iter_node->depth < candSeed->depth)
    {
      candSeed = iter_node;
    }
    else if (iter_node->depth == candSeed->depth)
    {
      if (iter_node->mse < candSeed->mse)
      {
        candSeed = iter_node;
      }
    }
  }
  EXPECT_EQ(seed, candSeed);
}

TEST_F(QuatreeTest, checkNodeChildrenIsPlane)
{
  qq1->showFrame();
  std::list<quatree::node*> l;
  l.emplace_back(qq1->getRoot());
  while (!l.empty())
  {
    quatree::node* tmpnode = l.front();
    l.pop_front();
    if (quatree::checkNodeChildrenIsPlane(tmpnode))
    {
      std::cout<<"node "<<tmpnode<<" is plane after check"<<std::endl;
    }
    else
    {
      std::cout<<"node "<<tmpnode<<" is not plane after check"<<std::endl;
    }
    
    std::cout<<"new check: "<<std::endl;

    if (quatree::checkNodeChildrenIsPlaneMean(tmpnode))
    {
      std::cout<<"node "<<tmpnode<<" is plane after check"<<std::endl;
    }
    else
    {
      std::cout<<"node "<<tmpnode<<" is not plane after check"<<std::endl;
    }
    std::cout<<std::endl;
    for (auto & iter_child : tmpnode->children)
    {
      if (iter_child)
      {
        l.emplace_back(iter_child);
      }
    }
  }





}

TEST_F(QuatreeTest, checkTree)
{
  LOG(INFO)<<"in check tree"<<std::endl;
  std::list<quatree::node*> nodes = qq1->getPatchsList();
  LOG(INFO)<<"GET PATCH"<<endl;
  std::list<quatree::node*> deletenodes;
  size_t index = 0;
  while (index < 3)
  {
    deletenodes.emplace_back(nodes.back());
    nodes.pop_back();
    index++;
  }
  LOG(INFO)<<"get delete nodes";
  qq1->showFrame();
  for (auto & iter_node : deletenodes)
  {
    for (size_t i = 0; i < 4; i++)
    {
      if (iter_node->parent->children.at(i))
      {
        if (iter_node->parent->children.at(i) == iter_node)
        {
          iter_node->parent->children.at(i) = nullptr;
          break;
        }
      }
    }
    delete iter_node;
    iter_node = nullptr;
    
  }
  std::cout<<"show delte frame"<<std::endl;
  qq1->showFrame();//验证上面的删除方式对不对
  
  qq1->refreshTree();
  std::cout<<"after check"<<std::endl;
  qq1->showFrame();
  std::cout<<"after show"<<std::endl;
  
  qq3->showFrame();
  nodes = qq3->getPatchsList();
  for (auto & iter_node : nodes)
  {
    
    quatree::deleteNodeInQuatree(iter_node);
  }
  std::cout<<"qq3 show"<<std::endl;
  qq3->showFrame();

  qq3->refreshTree();

  qq3->showFrame();
}

TEST_F(QuatreeTest, getQuatreeNeighbors)
{
  qq1->pindex2d->showInfo();
  std::list<quatree::node*> l;
  l.emplace_back(qq1->getRoot());
  while (!l.empty())
  {
    quatree::node* tmpnode = l.front();
    l.pop_front();
    if (tmpnode->is_plane || tmpnode->is_leafnode)
    {
      std::cout<<"tmp node address "<<tmpnode<<std::endl;
      std::cout<<" neighbors are: "<<endl;
      for (auto & iter_neighbor : tmpnode->neighbors)
      {
        std::cout<<iter_neighbor<<std::endl;
      }
      std::cout<<std::endl;
      // 这里不需要在邻近节点的邻近节点集中加入该节点，因为后期会添加
    }
    else
    {
      for (auto & iter_child : tmpnode->children)
      {
        if (iter_child)
        {
          l.emplace_back(iter_child);
        }
      }
    }
  }

  qq2->pindex2d->showInfo();
  l.clear();
  l.emplace_back(qq2->getRoot());
  while (!l.empty())
  {
    quatree::node* tmpnode = l.front();
    l.pop_front();
    if (tmpnode->is_plane || tmpnode->is_leafnode)
    {
      std::cout<<"tmp node address "<<tmpnode<<std::endl;
      std::cout<<" neighbors are: "<<endl;
      for (auto & iter_neighbor : tmpnode->neighbors)
      {
        std::cout<<iter_neighbor<<std::endl;
      }
      std::cout<<std::endl;
      // 这里不需要在邻近节点的邻近节点集中加入该节点，因为后期会添加
    }
    else
    {
      for (auto & iter_child : tmpnode->children)
      {
        if (iter_child)
        {
          l.emplace_back(iter_child);
        }
      }
    }
  }
}
