/*
 * @description: 
 * @param : 
 * @return: 
 */
#include "plane_detection/quadtree_new.h"
// #include "plane_detection/type.h"
#include <Eigen/Eigenvalues>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <ctime>
// #include "libxl.h"
// extern orginazed_points raw_points;
// extern parameter param;
namespace quatree
{
quatree::quatree(orginazed_points & org_points_, parameter & param_):raw_points(org_points_),param(param_)
{
  // root = std::make_shared<node>(0, 0, param.quatree_width, 0, 0, param.quatree_width/param.leafnode_width, 0, nullptr);
  root = node::create(0, 0, param.quatree_width, 0, 0, param.quatree_width / param.leafnode_width, 0, nullptr);
  refreshTree();// 去除子节点全部为nullptr的点
  // root->showNodeFrame();
  // showLeafNodeImage();
  mergePatchsForSeeds();
  // // root->showNodeFrame();
  // showMergeLeafNodeImage();
  refreshIndex2D();
  // pindex2d->pritfIndex();
  getQuatreeNeighbors();
  // showPatchInfo();
  // showSeedNodeAndNeighbors();
  // LOG(INFO)<<"merged: ";
}

quatree::~quatree()
{
  clear(root);
}

void quatree::clear(std::shared_ptr<node> p)
{
  if (p == root)
  {
    p.reset();
    return;
  }
  
  if (p != nullptr)
  {
    for (size_t i = 0; i < 4; i++)
    {
      if (p->parent->children.at(i) == p)
      {
        p->parent->children.at(i).reset();
      }
      // clear(p->children.at(i));
    }
    // delete p;
    // p = nullptr;
  }
}

// 四个子结点均为plane时，考虑能不能将其合并，为了获得稳定的种子点云
void quatree::mergePatchsForSeeds()
{
  mergePatchs(root);
}



// 需要把所有为nullptr子节点的父节点也全部合并，不仅仅是平面节点
void quatree::mergePatchs(std::shared_ptr<node> p)
{
  // LOG(INFO)<<"in mergePatchs function."<<endl;
  if (p != nullptr && !p->is_leafnode)
  {
    for (auto & iter_node : p->children)
    {
      mergePatchs(iter_node);
    }
    if (checkNodeChildrenIsPlaneMean(p))
    {
      refreshNodeParam(p);
    }
  }
}

bool quatree::checkNodeChildrenIsPlaneMean(std::shared_ptr<node> p)
{
  // if (p->valid_points_size == 0)
  // {
  //   for (size_t i = 0; i < 4; i++)
  //   {
  //     // LOG(INFO)<<"at "<<i<<" child"<<endl;
  //     if (p->parent->children.at(i) == p)
  //     {
  //       // LOG(INFO)<<"at "<<i<<"child, need delete"<<endl;
  //       p->parent->children.at(i) = nullptr;
  //       delete p;
  //       p = nullptr;
  //       break;
  //     }
  //   }
  //   // delete p;
  //   // p = nullptr;
  // }
  
  // if (p->children.at(0) == nullptr && 
  //   p->children.at(1) == nullptr && 
  //   p->children.at(2) == nullptr &&
  //   p->children.at(3) == nullptr)
  // {
  //   for (int j = 0; j < 4; j++)
  //   {
  //     if (p->parent->children.at(j) == p)
  //     {
  //       p->parent->children.at(j) = nullptr;
  //       delete p;
  //       p = nullptr;
  //       break;
  //     }
  //   }
  // }
  
  

  for (size_t i = 0; i < 4; i++)
  {
    if (p->children.at(i) == nullptr || !p->children.at(i)->is_plane)
    {
      return false;
    }
  }
  // 表明每个子节点是平面，检查平面法向量
  // 这个相当于每次判断是否合并，都需要判断12次
  // 可不可以改成先求法向量的平均值，再将每个子节点的法向量与平均值进行比较，做一个测试
  // Eigen::Vector3f mean_normal = Eigen::Vector3f::Zero();
  // size_t sum_num = 0;
  // for (auto & iter_child : p->children)
  // {
  //   mean_normal += iter_child->normal * iter_child->valid_points_size;
  //   sum_num += iter_child->valid_points_size;
  // }
  // mean_normal /= sum_num;
  
  // for (auto & iter_child : p->children)
  // {
  //   if (mean_normal.dot(iter_child->normal) < param.quatree_merge_normal_dot_th)
  //   {
  //     return false;
  //   }
  // }

  // 任意两法向量差距比较大

  // 距离因素 任意两个子节点之间的距离都很小 法向量差距很小
  for (int i = 0; i < 3; i++)
  {
    for (int j = i + 1; j < 4; j++)
    {
      if (abs(p->children.at(i)->normal.dot(p->children.at(j)->normal)) < param.quatree_merge_normal_dot_th)
      {
        return false;
      }
      else
      {
        if (abs((p->children.at(i)->center - p->children.at(j)->center).dot(p->children.at(i)->normal)) > param.merge_normal_distance)
        {
          return false;
        }
      }
    }
  }
  


  return true;
}


// 只表明了可以子结点都为平面，并不代表可以合并
/**
 * @description: 对属于同一平面的节点进行合并，为减小计算量，该函数已不再使用，取而代之的是checkNodeChildrenIsPlaneMean
 * @param {node*} p
 * @return {*}
 */
bool quatree::checkNodeChildrenIsPlane(std::shared_ptr<node> p)
{
  // LOG(INFO)<<"in checkNodeChildrenIsPlane function."<<endl;
  for (size_t i = 0; i < 4; i++)
  {
    if (p->children.at(i) == nullptr || !p->children.at(i)->is_plane)
    {
      return false;
    }
  }

  for (size_t i = 0; i < 3; i++)
  {
    for (size_t j = i + 1; j < 4; j++)
    {
      if (p->children.at(i)->normal.dot(p->children.at(j)->normal) < param.quatree_merge_normal_dot_th)// merge 法向量预判断阈值
      {
        return false;
      }
    }
  }
  return true;
}
// 在法向量满足阈值差距时，才合并，否则不合并
void quatree::refreshNodeParam(std::shared_ptr<node> p)
{
  // LOG(INFO)<<"in refreshNodeParam function."<<endl;
  // 默认所有patch 已经是属于同一平面的， 2014年博士论文公式4.14
  // std::array<Eigen::Vector3f, 4> centers;
  // std::array<size_t, 4> valid_nums;
  // std::array<Eigen::Vector3f, 4>::iterator iter_center = centers.begin();
  // std::array<size_t, 4>::iterator iter_num = valid_nums.begin();
  // for (auto & iter_node : p->children)
  // {
  //   assert(iter_node != nullptr && "the child is empty");
  //   assert(iter_center != centers.end() && "the center iterator is out of range");
  //   assert(iter_num != valid_nums.end() && "the center iterator is out of range");
  //   *iter_center = iter_node->center;
  //   *iter_num = iter_node->valid_points_size;
  //   iter_center ++;
  //   iter_num ++;
  //   // LOG(INFO)<<"current node J:"<<endl<<iter_node->J<<endl;
  //   p->J += iter_node->J;
  // }
  // Eigen::Vector3f M = Eigen::Vector3f::Zero();
  // size_t sum_nums = 0;
  // for (size_t i = 0; i < 4; i++)
  // {
  //   M += valid_nums.at(i) * centers.at(i);
  //   sum_nums += valid_nums.at(i);
  // }
  // p->center = M/sum_nums;
  // // LOG(INFO)<<"get center.."<<endl;
  // Eigen::Matrix3f S = p->J - M*p->center.transpose();
  // // LOG(INFO)<<"node J: "<<endl<<p->J<<endl;
  // // LOG(INFO)<<"M: "<<endl<<M<<endl;
  // // LOG(INFO)<<"node center: "<<p->center.transpose()<<endl;
  // Eigen::EigenSolver<Eigen::Matrix3f> es(S);
  // Eigen::Matrix3f::Index b;
  // auto minEigenValue = es.eigenvalues().real().minCoeff(&b);
  // double eigenValuesSum = es.eigenvalues().real().sum();
  // p->normal = es.eigenvectors().real().col(b);
  // LOG(INFO)<<"MIN SUM NORMAL: "<<minEigenValue<<" "<<eigenValuesSum<<" "<<p->normal.transpose()<<endl;
  // LOG(INFO)<<"get normal.."<<endl;
  p->stats = Stats(p->children.at(0)->stats, p->children.at(1)->stats, p->children.at(2)->stats, p->children.at(3)->stats);
  p->stats.compute(p->center, p->normal, p->mse, p->curvature);
  // 在此判断是否可以合并，如果不性，修改上述已经赋值的参数
  // bool isSamePlane = true;
  // for (auto & iter_node : p->children)
  // {
  //   if (!std::abs(p->normal.dot(iter_node->normal)) > param.merge_normal_dot_th)// 满足合并条件
  //   {
  //     isSamePlane = false;
  //     break;
  //   }
  // }
  // LOG(INFO)<<"refresh information.."<<endl;
  if (p->curvature < param.eigen_value_th && p->mse < param.patch_mse_th)
  {
    // p->mse = minEigenValue/sum_nums;// 验证计算方式
    p->is_plane = true;
    p->is_validnode = true;
    p->valid_points_size = p->stats.N;
    
    // clear memory of children 
    // 改进点2
    // 是不是可以不考虑点，直接修改该节点的特征就行
    for (auto & iter_node : p->children)
    {
      p->valid_points.insert(p->valid_points.end(), iter_node->valid_points.begin(), iter_node->valid_points.end());
      // delete iter_node;
      iter_node.reset();
    }
    p->children = {nullptr, nullptr, nullptr, nullptr};
  }
  // 虽然属于同一平面
  else // 在判断属于同一平面的前提下进行
  {
    p->stats.clear();
    p->normal = Eigen::Vector3f::Zero();
    p->center = Eigen::Vector3f::Zero();
    // p->J = Eigen::Matrix3f::Zero();
    p->is_plane = false;
  }
}

/**
 * @description: 当四叉树中有节点被删除时，需要对四叉树进行检查，以保证四叉树的节点的有效性，如果四叉树的某个节点的子节点都为空，那么这个节点也无效，成为空节点。注意：节点的有效性判断，不能以该节点是否是平面，而是该节点是否就能认定为空
 * @param {node *} p
 * @return {*}
 */
bool quatree::check(std::shared_ptr<node> p)
{
  // LOG(INFO)<<"tmp node is "<<p<<std::endl;
  if (p == nullptr)
  {
    return false;
  }
  if (!p->is_leafnode)
  {
    if (p->is_plane)
    {
      return true;
    }
    
    std::array<bool, 4> children_flag = {false};
    for (size_t i = 0; i < 4; i++)
    {
      if (check(p->children.at(i)))
      {
        // LOG(INFO)<<"now the child "<<i<<" is true"<<std::endl;
        children_flag.at(i) = true;
      }
    }
    // 不一定是子节点都为nullptr就把该节点删除
    if (children_flag.at(0) == true || 
        children_flag.at(1) == true ||
        children_flag.at(2) == true ||
        children_flag.at(3) == true)
    {
      // LOG(INFO)<<"now the node is true"<<std::endl;
      return true;
    }
    else // 所有子结点都为false，
    {
      if (p == root)
      {
        deleteNodeInQuatree(p);
        root = nullptr;
      }
      else
      {
        deleteNodeInQuatree(p);// 重新测试？？

      }
      return false;
    }
  }
  else
  {
    // return p->is_plane;
    // 对于节点虽然不是平面，但是依然是不可舍弃的节点
    if (p->valid_points_size != 0)
    {
      return true;
    }
    else
    {
      return false;
    }
    
    // return p != nullptr;
  }
  // LOG(INFO)<<"is a valid leaf node"<<std::endl;
  return true;
}

// 返回大顶堆 这里可以使用list实现,也可以使用priority_queue<node>,但开支较大
// 在必要时建议使用lambda函数, 先定义一个比较函数auto cmp = [](const node* a,const node* b){return a.size < b.size;};
// priority_queue<node*,vector<node*>,decltype(cmp)> que4(cmp);
std::priority_queue<std::shared_ptr<node>, std::vector<std::shared_ptr<node>>, reverseComnode> quatree::getPatchs()
{
  std::priority_queue<std::shared_ptr<node>, std::vector<std::shared_ptr<node>>, reverseComnode> patchs;
  std::list<std::shared_ptr<node>> Q; 
  Q.emplace_back(root);
  while (!Q.empty())
  {
    std::shared_ptr<node> tmpnode = Q.front();
    Q.pop_front();
    if (tmpnode->is_plane)
    {
      patchs.push(tmpnode);
    }
    else
    {
      for (auto & iter_node : tmpnode->children)
      {
        if (iter_node)
        {
          Q.push_back(iter_node);
        }
      }
    }
  }
  return patchs;
}

void quatree::refreshIndex2D()
{
  pindex2d = new index2D(std::ceil(double(raw_points.height)/double(param.leafnode_width)), std::ceil(double(raw_points.width)/double(param.leafnode_width)), root);
}

void quatree::PreRefreshIndex2D()
{
  delete pindex2d;
  pindex2d = nullptr;
  pindex2d = new index2D(std::ceil(double(raw_points.height)/double(param.leafnode_width)), std::ceil(double(raw_points.width)/double(param.leafnode_width)), root);
}

std::shared_ptr<node> quatree::getSeedNode()
{
  std::shared_ptr<node> candNode = nullptr;
  std::list<std::shared_ptr<node>> Q; 
  Q.emplace_back(root);
  while (!Q.empty())
  {
    std::shared_ptr<node> tmpnode = Q.front();
    // LOG(INFO)<<"current node is "<<tmpnode<<std::endl;
    Q.pop_front();
    if (tmpnode == nullptr)
    {
      continue;
    }
    // 每一层检测一次，寻找最大，mse最小的节点为seed
    if (tmpnode->is_plane)
    {
      // LOG(INFO)<<"node "<<tmpnode<<" is plane"<<std::endl;
      if (!candNode)
      {
        // LOG(INFO)<<"the candinate node is nullptr, refresh node address."<<std::endl;
        candNode = tmpnode;
      }
      else
      {
        // LOG(INFO)<<"the candinate node is not nullptr."<<std::endl;
        if (candNode->depth == tmpnode->depth)
        {
          if (candNode->mse > tmpnode->mse)
          {
            // LOG(INFO)<<"choose mse small."<<std::endl;
            candNode = tmpnode;
          }
        }
        else
        {
          // LOG(INFO)<<"RETURN 1"<<std::endl;
          return candNode;
        }
          
      }
    }
    else
    {
      if (!candNode ||(candNode && candNode->depth > tmpnode->depth))// 必须满足candNode->depth > tmpnode->depth
      {
        for (auto & iter_node : tmpnode->children)
        {
          // LOG(INFO)<<"ADD CHILD "<<iter_node<<std::endl;
          if (iter_node)
          {
            Q.emplace_back(iter_node);
          }
        }
      }
    }
  }
  // LOG(INFO)<<"RETURN 2"<<std::endl;
  return candNode;
}

// void quatree::refreshIndex2D()
// {
//   for (auto & iter_rows : index2D)
//   {
//     for (auto & iter_rows_cols : iter_rows)
//     {
//       iter_rows_cols = nullptr;
//     }
//   }
//   std::list<node*> Q; 
//   Q.emplace_back(root);
//   while (!Q.empty())
//   {
//     node* tmpnode = Q.front();
//     Q.pop_front();
//     if (tmpnode->is_plane)
//     {
//       size_t rows_index = tmpnode->start_rows/leafnode_width;
//       size_t cols_index = tmpnode->start_cols/leafnode_width;
//       size_t width_index = tmpnode->width/leafnode_width;
//       std::vector<std::vector<node*>>::iterator rows_iter = index2D.begin() + rows_index;
//       for (size_t i = 0; i < width_index; i++)
//       {
//         std::vector<node*>::iterator rows_cols_iter = rows_iter->begin() + cols_index;
//         for (size_t j = 0; j < width_index; j++)
//         {
//           *rows_cols_iter = tmpnode;
//           rows_cols_iter++;
//         }
//         rows_iter++;
//       }
//     }
//     else
//     {
//       for (auto & iter_node : tmpnode->children)
//       {
//         if (iter_node)
//         {
//           Q.emplace_back(iter_node);
//         }
//       }
//     }
//   }
// }

std::list<std::shared_ptr<node>> quatree::getPatchsList()
{
  std::list<std::shared_ptr<node>> patchs;
  std::list<std::shared_ptr<node>> Q; 
  Q.emplace_back(root);
  while (!Q.empty())
  {
    std::shared_ptr<node> tmpnode = Q.front();
    Q.pop_front();
    if (tmpnode->is_plane)
    {
      patchs.emplace_back(tmpnode);
    }
    else
    {
      for (auto & iter_node : tmpnode->children)
      {
        if (iter_node)
        {
          Q.emplace_back(iter_node);
        }
      }
    }
  }
  return patchs;
}

void quatree::refreshTree()
{
  // clock_t start = clock();
  check(root);
  // std::cout<<"check tree costs "<<(double)(clock() - start)/CLOCKS_PER_SEC <<std::endl;
  // if (root)
  // {
  //   pindex2d->setNullptr();
  //   pindex2d->initial(root);
  //   getQuatreeNeighbors();
  // }
  // std::cout<<"refresh neighbors costs "<<(double)(clock() - start)/CLOCKS_PER_SEC <<std::endl;

}

// 合并四叉树之后，获取节点的邻近节点
void quatree::getQuatreeNeighbors()
{
  // list是一个双端队列，不是一个优先队列
  std::list<std::shared_ptr<node>> l;
  l.emplace_back(root);
  while (!l.empty())
  {
    std::shared_ptr<node> tmpnode = l.front();
    l.pop_front();
    if (tmpnode->is_plane || tmpnode->is_leafnode)
    {
      pindex2d->getNeighbors(tmpnode, tmpnode->neighbors);
      // cout<<"node: "<<tmpnode<<", neighbors: "<<endl;
      // for (auto & nei : tmpnode->neighbors)
      // {
      //   cout<<nei<<" ";
      // }
      // cout<<endl;
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
// void quatree::saveAsExcel(string filePath)
// {
//   using namespace libxl;
//   Book* book = xlCreateBook();  
//   Format* format = book->addFormat();
//   format->setAlignH(ALIGNH_CENTER);
//   format->setAlignV(ALIGNV_CENTER);
//   Sheet* sheet = book->addSheet("Sheet1");
//   std::list<node*> node_list;
//   node_list.emplace_back(root);
//   size_t start_row = 0, end_row = 0;
//   size_t start_col = 0, end_row = 0;
//   size_t level_node = 0;
//   while (!node_list.empty())
//   {
//     node* tmpnode = node_list.front();
//     node_list.pop_front();
//     if (level_node == tmpnode->depth)
//     {   
//     }
//   }
// }

// std::vector<std::shared_ptr<node>> quatree::getLeafnodes()
// {
//   vector<node*> leafnodes;
//   std::list<std::shared_ptr<node>> Q; 
//   Q.emplace_back(root);
//   while (!Q.empty())
//   {
//     node* tmpnode = Q.front();
//     Q.pop_front();
//     if (tmpnode->is_leafnode)
//     {
//       leafnodes.emplace_back(tmpnode);
//     }
//     else
//     {
//       for (auto & iter_node : tmpnode->children)
//       {
//         if (iter_node)
//         {
//           Q.push_back(iter_node);
//         }
//       }
//     }
//   }
//   return leafnodes;
// }

void quatree::showPatchInfo()
{
  std::list<std::shared_ptr<node>> l;
  l.emplace_back(root);
  while (!l.empty())
  {
    std::shared_ptr<node> tmpnode = l.front();
    l.pop_front();
    if (tmpnode->is_plane || tmpnode->is_leafnode)
    {
      LOG(INFO)<<"node: "<<tmpnode;
      LOG(INFO)<<tmpnode->valid_points_size<<" "<<tmpnode->valid_points.size()<<" "<<tmpnode->mse;
      LOG(INFO)<<"its neighbors: ";
      for (auto & iter_node : tmpnode->neighbors)
      {
        LOG(INFO)<<iter_node;
      }
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

// void quatree::showFrame()
// {
//   if (root)
//   {
//     LOG(INFO)<<"root is "<<root<<endl;
//     root->showNodeFrame();
//   }
//   else
//     LOG(INFO)<<"it is an empty tree."<<endl;
// }

void quatree::showLeafNodeImage()
{
  cv::Mat color_image = cv::imread("/home/lichao/catkin_plane_detection/src/my_plane_detection_ros-master/bag/color_part_align.png");
  // 对于是平面节点的点画点，对于非平面节点画叉叉，节点边界用细线画
  std::list<std::shared_ptr<node>> Q;
  Q.emplace_back(root);
  while (!Q.empty())
  {
    std::shared_ptr<node> tmpnode = Q.front();
    Q.pop_front();
    if (tmpnode->is_leafnode)
    {
      if (tmpnode->is_plane)
      {
        cv::rectangle(color_image, cv::Rect(tmpnode->start_cols, tmpnode->start_rows, tmpnode->width, tmpnode->width), cv::Scalar(50, 0, 0), 1);
        
        cv::circle(color_image, cv::Point2f(tmpnode->start_cols + tmpnode->width/2, tmpnode->start_rows + tmpnode->width/2), 3, cv::Scalar(0, 255, 120), -1);//画点，其实就是实心圆
      }
      else
      {
        cv::rectangle(color_image, cv::Rect(tmpnode->start_cols, tmpnode->start_rows, tmpnode->width, tmpnode->width), cv::Scalar(50, 0, 0), 1);
      
        cv::putText(color_image, "x", cv::Point2f(tmpnode->start_cols + tmpnode->width/2 - 7, tmpnode->start_rows + tmpnode->width/2 + 3), cv::FONT_HERSHEY_COMPLEX_SMALL, 1, cv::Scalar(0, 255, 120));
      }
      
    }
    else
    {
      for (auto & iter_node : tmpnode->children)
      {
        if (iter_node)
        {
          Q.push_back(iter_node);
        }
      }
    }
  }
  cv::imwrite("/home/lichao/catkin_plane_detection/src/my_plane_detection_ros-master/bag/leaf_node.png", color_image);
  cv::imshow("leaf node", color_image);
  cv::waitKey(0);
}

void quatree::showMergeLeafNodeImage()
{
  // 如果有彩色图时
  cv::Mat color_image = cv::imread("/home/lichao/TCDS/src/pip_line/data/rotatedImage.png");
  LOG(INFO)<<"color_image size: "<<color_image.size;
  // 如果没有彩色图时
  // cv::Mat color_image = cv::Mat::zeros(raw_points.height, raw_points.width, CV_8UC3);
  // 对于是平面节点的点画点，对于非平面节点画叉叉，节点边界用细线画
  int scale = 8;
  std::list<std::shared_ptr<node>> Q;
  Q.emplace_back(root);
  while (!Q.empty())
  {
    std::shared_ptr<node> tmpnode = Q.front();
    Q.pop_front();

    if (!tmpnode->is_leafnode && !tmpnode->is_plane)
    {
      for (auto & iter_node : tmpnode->children)
      {
        if (iter_node)
        {
          Q.push_back(iter_node);
        }
      }
    }
    else
    {
      if (tmpnode->is_plane)
      {
        cv::rectangle(color_image, cv::Rect(tmpnode->start_cols * scale, tmpnode->start_rows * scale, tmpnode->width * scale, tmpnode->width * scale), cv::Scalar(51, 160, 44), 2);
      }
      else
      {
        cv::rectangle(color_image, cv::Rect(tmpnode->start_cols * scale, tmpnode->start_rows * scale, tmpnode->width * scale, tmpnode->width * scale), cv::Scalar(51, 160, 44), 2);
      
        cv::putText(color_image, "x", cv::Point2f(tmpnode->start_cols  * scale + tmpnode->width * scale/2  - 1.2 * scale, tmpnode->start_rows  * scale + tmpnode->width * scale/2 + scale), cv::FONT_HERSHEY_COMPLEX_SMALL, 1.5, cv::Scalar(0, 0, 255), 2);
      }
    }
  }
  cv::imwrite("/home/lichao/TCDS/src/pip_line/data/leaf_node_merge.png", color_image);
  cv::imshow("leaf node", color_image);
  cv::waitKey(0);
}

void quatree::showSeedNodeAndNeighbors()
{
  cv::Mat color_image = cv::imread("/home/lichao/catkin_plane_detection/src/my_plane_detection_ros-master/bag/color_part_align.png");
  std::shared_ptr<node> seed = getSeedNode();
  cv::rectangle(color_image, cv::Rect(seed->start_cols, seed->start_rows, seed->width, seed->width), cv::Scalar(50, 0, 0), 1);
  cv::imshow("seed node", color_image);
  cv::imwrite("/home/lichao/catkin_plane_detection/src/my_plane_detection_ros-master/bag/seed_node.png", color_image);
  cv::waitKey(0);
  for (auto & neighbor : seed->neighbors)
  {
    cv::rectangle(color_image, cv::Rect(neighbor->start_cols, neighbor->start_rows, neighbor->width, neighbor->width), cv::Scalar(50, 0, 0), 1);
  }
  cv::imshow("merge step", color_image);
  cv::imwrite("/home/lichao/catkin_plane_detection/src/my_plane_detection_ros-master/bag/merge_step.png", color_image);
  cv::waitKey(0);
}

void quatree::printfNeighbors()
{
  std::list<std::shared_ptr<node>> Q;
  Q.emplace_back(root);
  while (!Q.empty())
  {
    std::shared_ptr<node> tmpnode = Q.front();
    Q.pop_front();
    if (tmpnode->is_leafnode)
    {
      LOG(INFO)<<"node: "<<tmpnode;
      for (auto & nei : tmpnode->neighbors)
      {
        cout<<nei<<" ";
      }
      cout<<endl;
    }
    else
    {
      if (tmpnode->is_plane)
      {
        LOG(INFO)<<"node: "<<tmpnode;
        for (auto & nei : tmpnode->neighbors)
        {
          cout<<nei<<" ";
        }
        cout<<endl;
      }
      else
      {
        for (auto & child : tmpnode->children)
        {
          if (child)
          {
            Q.emplace_back(child);
          }
        }
      }
    }
  }
}

}
