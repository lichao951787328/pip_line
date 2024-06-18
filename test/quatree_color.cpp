/*
 * @description: 
 * @param : 
 * @return: 
 */
#include <iostream>
#include <list>
#include <set>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc.hpp>
using namespace std;
vector<pair<size_t, size_t>> nonplanar_index;
cv::Scalar color(127, 255, 0);
void draw_planar_marker(cv::Mat & image, size_t start_cols, size_t start_rows, size_t width)
{
  cv::rectangle(image, cv::Rect(start_cols, start_rows, width, width), color, 1);
  // cv::rectangle(image, cv::Rect(start_cols, start_rows, width, width), cv::Scalar(128, 138, 135), 1);
  cv::drawMarker(image, cv::Point2d(start_cols + width/2, start_rows + width/2), cv::Scalar(220, 220, 220), cv::MARKER_STAR, 2);
}

void draw_nonplanar_marker(cv::Mat & image, size_t start_cols, size_t start_rows, size_t width)
{
  cv::rectangle(image, cv::Rect(start_cols, start_rows, width, width), color, 1);
  cv::drawMarker(image, cv::Point2d(start_cols + width/2, start_rows + width/2), cv::Scalar(220, 220, 220), cv::MARKER_TILTED_CROSS, 10, 2);
}

bool checkIsPlanarNode(vector<pair<size_t, size_t>> & set, size_t x, size_t y)
{
  for (auto & i : set)
  {
    if (i.first == x && i.second == y)
    {
      return false;
    }
  }
  return true;
}

struct quatree_node
{
  static size_t leafnode_depth, quatree_width, quatree_height;
  size_t x, y, w, d;
  bool is_planar;
  bool is_leafnode = false;
  std::array<quatree_node*, 4> children;
  // vector<quatree_node*> neighbors;
  quatree_node(size_t x_, size_t y_, size_t w_, size_t depth, bool is_planar_ = true)
  {
    // cout<<"node width: "<<w_<<endl;
    x = x_; y = y_; w = w_; d = depth;
    if (depth == leafnode_depth)
    {
      if (checkIsPlanarNode(nonplanar_index, x_, y_))
      {
        is_planar = is_planar_;
      }
      else
      {
        is_planar = false;
      }
      is_leafnode = true; 
    }
    else
    {
      is_leafnode = false;
      is_planar = false;
      size_t child_width = w_>>1;
      for (size_t i = 0; i < 4; i++)
      {
        size_t child_x = x_ + child_width * (i >>1 & 1);
        size_t child_y = y_ + child_width * (i&1);
        if (child_x >= quatree_width || child_y >= quatree_height)
        {
          children.at(i) = nullptr;
        }
        else
        {
          children.at(i) = new quatree_node(child_x, child_y, child_width, depth + 1);
        }
      }
    }
  }

  bool containIndex(pair<size_t, size_t> index)
  {
    if (index.first >= x && index.first < x + w)
    {
      if (index.second >= y && index.second < y + w)
      {
        return true;
      }
    }
    return false;
  }
};

class quatree_tree
{
private:
  size_t width;
  quatree_node* root;
public:
  quatree_tree(size_t w)
  {
    root = new quatree_node(0, 0, w, 0);
  }

  void getPlanarnodes(vector<quatree_node*> & planarnodes, vector<quatree_node*> & nonplanarnodes)
  {
    list<quatree_node*> l;
    l.emplace_back(root);
    while (!l.empty())
    {
      quatree_node* tmpnode = l.front();
      l.pop_front();
      if (tmpnode->is_planar)
      {
        planarnodes.emplace_back(tmpnode);
        
      }
      else
      {
        if (tmpnode->is_leafnode)
        {
          nonplanarnodes.emplace_back(tmpnode);
        }
        else
        {
          for (auto & iter : tmpnode->children)
          {
            if (iter)
            {
              l.emplace_back(iter);
            }
          }
        }
      }
    }
  }

  void merge()
  {
    mergePatchs(root);
  
  }

  void mergePatchs(quatree_node * p)
  {
    if (p != nullptr && !p->is_leafnode)
    {
      for (auto & iter_node : p->children)
      {
        if (iter_node)
        {
          mergePatchs(iter_node);
        }
      }
      if (checkIsPlanar(p))
      {
        p->is_planar = true;
      }
    }
  }

  bool checkIsPlanar(quatree_node* p)
  {
    for (auto & node : p->children)
    {
      if (!node || !node->is_planar)
      {
        return false;
      }
    }
    return true;
  }

  quatree_node* getSeed()
  {
    list<quatree_node*> l;
    l.emplace_back(root);
    while (!l.empty())
    {
      quatree_node* tmpnode = l.front();
      l.pop_front();
      if (tmpnode->is_planar)
      {
        return tmpnode;
      }
      else
      {
        for (auto & node : tmpnode->children)
        {
          if (node)
          {
            l.emplace_back(node);
          }
        }
      }
    }
  }
  
  vector<quatree_node*> getNeighbors(quatree_node* p, size_t mat_width, size_t mat_height)
  {
    vector<pair<size_t, size_t>> neighborsIndex;
    cout<<"node position "<<p->x<<" "<<p->y<<" "<<p->w<<endl;
    size_t left_x = p->x > 0 ? p->x-1 : 0;
    size_t right_x = (p->x + p->w) < mat_width - 1 ? p->x + p->w : mat_width - 1;
    cout<<"left_x "<<left_x<<" right_x "<<right_x<<endl;
    if (p->y > 0)
    {
      for (size_t i = left_x; i <= right_x; i++)
      {
        neighborsIndex.emplace_back(make_pair(i, p->y - 1));
      }
    }
    size_t top_y = p->y > 0 ? p->y - 1 : 0;
    size_t down_y = p->y + p->w < mat_height - 1 ? p->y + p->w : mat_height - 1;
    cout<<"top_y "<<top_y<<" down_y "<<down_y<<endl;
    for (size_t i = top_y; i < down_y; i++)
    {
      if (p->x > 0)
      {
        neighborsIndex.emplace_back(make_pair(p->x - 1, i));
      }
      if (p->x + p->w < mat_width - 1)
      {
        neighborsIndex.emplace_back(make_pair(p->x + p->w, i));
      }
    }
    cout<<"p->y + p->w + 1 "<<p->y + p->w<<endl;
    if (p->y + p->w < mat_height - 1)
    {
      for (size_t i = left_x; i <= right_x; i++)
      {
        neighborsIndex.emplace_back(make_pair(i, p->y + p->w));
      }
    }
    vector<quatree_node*> neighborsnode;
    vector<quatree_node*> allnodes = getAllNodes();
    for (auto & node : allnodes)
    {
      for (auto & index : neighborsIndex)
      {
        if (node->containIndex(index))
        {
          bool isin = false;
          for (auto & neighbor : neighborsnode)
          {
            if (neighbor == node)
            {
              isin = true;
              break;
            }
          }
          if (!isin)
          {
            neighborsnode.emplace_back(node);
          }
        }
      }
    }
    return neighborsnode;
  }

  vector<quatree_node*> getAllNodes()
  {
    vector<quatree_node*> planarnodes, nonplanarnodes, allnodes;
    getPlanarnodes(planarnodes, nonplanarnodes);
    allnodes.insert(allnodes.end(), planarnodes.begin(), planarnodes.end());
    allnodes.insert(allnodes.end(), nonplanarnodes.begin(), nonplanarnodes.end());
    return allnodes;
  }
  ~quatree_tree()
  {

  }
};

size_t quatree_node::leafnode_depth;
size_t quatree_node::quatree_width;
size_t quatree_node::quatree_height;

int main(int argc, char** argv)
{
  cv::Mat color_image = cv::imread("/home/humanoid/work/plane_detection/catkin_plane_detection_test_ws/src/plane_detection/bag/color_part_align2.png");
  size_t width = color_image.cols; size_t height = color_image.rows;
  cout<<"width: "<<width<<" height: "<<height<<endl;//540 380
  cout<<"node width: "<<width/20<<" node height: "<<height/20<<endl;
  
  
  {
  nonplanar_index.emplace_back(make_pair(0,0));
  nonplanar_index.emplace_back(make_pair(1,0));
  nonplanar_index.emplace_back(make_pair(2,0));
  nonplanar_index.emplace_back(make_pair(3,0));
  nonplanar_index.emplace_back(make_pair(4,0));
  nonplanar_index.emplace_back(make_pair(5,0));
  nonplanar_index.emplace_back(make_pair(6,0));
  nonplanar_index.emplace_back(make_pair(22,0));
  nonplanar_index.emplace_back(make_pair(23,0));
  nonplanar_index.emplace_back(make_pair(24,0));
  nonplanar_index.emplace_back(make_pair(25,0));
  nonplanar_index.emplace_back(make_pair(26,0));
  nonplanar_index.emplace_back(make_pair(1,1));
  nonplanar_index.emplace_back(make_pair(2,1));
  nonplanar_index.emplace_back(make_pair(3,1));
  nonplanar_index.emplace_back(make_pair(4,1));
  nonplanar_index.emplace_back(make_pair(5,1));
  nonplanar_index.emplace_back(make_pair(6,1));
  nonplanar_index.emplace_back(make_pair(22,1));
  nonplanar_index.emplace_back(make_pair(23,1));
  nonplanar_index.emplace_back(make_pair(24,1));
  nonplanar_index.emplace_back(make_pair(25,1));
  nonplanar_index.emplace_back(make_pair(26,1));
  nonplanar_index.emplace_back(make_pair(2,2));
  nonplanar_index.emplace_back(make_pair(3,2));
  nonplanar_index.emplace_back(make_pair(4,2));
  nonplanar_index.emplace_back(make_pair(5,2));
  nonplanar_index.emplace_back(make_pair(6,2));
  nonplanar_index.emplace_back(make_pair(22,2));
  nonplanar_index.emplace_back(make_pair(23,2));
  nonplanar_index.emplace_back(make_pair(24,2));
  nonplanar_index.emplace_back(make_pair(25,2));
  nonplanar_index.emplace_back(make_pair(26,2));
  nonplanar_index.emplace_back(make_pair(23,3));
  nonplanar_index.emplace_back(make_pair(24,3));
  nonplanar_index.emplace_back(make_pair(25,3));
  nonplanar_index.emplace_back(make_pair(26,3));
  nonplanar_index.emplace_back(make_pair(24,4));
  nonplanar_index.emplace_back(make_pair(25,4));
  nonplanar_index.emplace_back(make_pair(26,4));

  nonplanar_index.emplace_back(make_pair(12,5));
  nonplanar_index.emplace_back(make_pair(13,5));
  nonplanar_index.emplace_back(make_pair(14,5));
  nonplanar_index.emplace_back(make_pair(15,5));
  nonplanar_index.emplace_back(make_pair(16,5));
  nonplanar_index.emplace_back(make_pair(17,5));
  nonplanar_index.emplace_back(make_pair(18,5));
  nonplanar_index.emplace_back(make_pair(19,5));
  nonplanar_index.emplace_back(make_pair(20,5));
  nonplanar_index.emplace_back(make_pair(21,5));
  nonplanar_index.emplace_back(make_pair(25,5));
  nonplanar_index.emplace_back(make_pair(26,5));
  nonplanar_index.emplace_back(make_pair(9,6));
  nonplanar_index.emplace_back(make_pair(10,6));
  nonplanar_index.emplace_back(make_pair(11,6));
  nonplanar_index.emplace_back(make_pair(21,6));
  nonplanar_index.emplace_back(make_pair(26,6));
  nonplanar_index.emplace_back(make_pair(9,7));
  nonplanar_index.emplace_back(make_pair(21,7));
  nonplanar_index.emplace_back(make_pair(9,8));
  nonplanar_index.emplace_back(make_pair(21,8));

  nonplanar_index.emplace_back(make_pair(9,9));
  nonplanar_index.emplace_back(make_pair(22,9));
  nonplanar_index.emplace_back(make_pair(9,10));
  nonplanar_index.emplace_back(make_pair(22,10));
  nonplanar_index.emplace_back(make_pair(10,11));
  nonplanar_index.emplace_back(make_pair(22,11));
  nonplanar_index.emplace_back(make_pair(10,12));
  nonplanar_index.emplace_back(make_pair(20,12));
  nonplanar_index.emplace_back(make_pair(21,12));
  nonplanar_index.emplace_back(make_pair(22,12));
  nonplanar_index.emplace_back(make_pair(10,13));
  nonplanar_index.emplace_back(make_pair(11,13));
  nonplanar_index.emplace_back(make_pair(12,13));
  nonplanar_index.emplace_back(make_pair(13,13));
  nonplanar_index.emplace_back(make_pair(14,13));
  nonplanar_index.emplace_back(make_pair(15,13));
  nonplanar_index.emplace_back(make_pair(16,13));
  nonplanar_index.emplace_back(make_pair(17,13));
  nonplanar_index.emplace_back(make_pair(18,13));
  nonplanar_index.emplace_back(make_pair(19,13));
  }
  size_t quatree_max_depth = std::ceil(log(width/20)/log(2));
  size_t quatree_full_width = std::pow(2, quatree_max_depth);
  quatree_node::leafnode_depth = quatree_max_depth;
  quatree_node::quatree_width = width/20;
  quatree_node::quatree_height = height/20;
  cout<<"max depth is "<<quatree_max_depth<<endl;
  cout<<"quatree full width "<<quatree_full_width<<endl;

  quatree_tree q(quatree_full_width);
  cout<<"construct quadtree"<<endl;
  vector<quatree_node*> planarnodes, nonplanarnodes;
  q.getPlanarnodes(planarnodes, nonplanarnodes);
  cout<<"planar node size "<<planarnodes.size()<<" nonplanar node size "<<nonplanarnodes.size()<<endl;
  cout<<"get nodes"<<endl;
  cv::imshow("color_image", color_image);
  cv::waitKey(0);
  cv::Mat leafnode_image = color_image.clone();
  
  for (auto & node :planarnodes)
  {
    draw_planar_marker(leafnode_image, node->x * 20, node->y * 20, node->w*20);
  }

  for (auto & node :nonplanarnodes)
  {
    draw_nonplanar_marker(leafnode_image, node->x * 20, node->y * 20, node->w*20);
  }
  cv::imwrite("/home/humanoid/work/plane_detection/catkin_plane_detection_test_ws/src/plane_detection/bag/leafnode_pic.png", leafnode_image);
  cv::imshow("leafnode_image", leafnode_image);
  cv::waitKey(0);
  q.merge();
  planarnodes.clear();
  nonplanarnodes.clear();
  q.getPlanarnodes(planarnodes, nonplanarnodes);
  cout<<"planar node size "<<planarnodes.size()<<" nonplanar node size "<<nonplanarnodes.size()<<endl;
  cv::Mat patch_image = color_image.clone();
  
  for (auto & node :planarnodes)
  {
    cout<<"x: "<<node->x<<" y: "<<node->y<<endl;
    draw_planar_marker(patch_image, node->x * 20, node->y * 20, node->w*20);
  }

  for (auto & node :nonplanarnodes)
  {
    draw_nonplanar_marker(patch_image, node->x * 20, node->y * 20, node->w*20);
  }
  cv::imwrite("/home/humanoid/work/plane_detection/catkin_plane_detection_test_ws/src/plane_detection/bag/patch_image_pic.png", patch_image);
  
  cv::imshow("patch_image", patch_image);
  cv::waitKey(0);

  cv::Mat seedmat = color_image.clone();
  quatree_node* seed = q.getSeed();
  draw_planar_marker(seedmat, seed->x*20, seed->y*20, seed->w * 20);
  cv::imwrite("/home/humanoid/work/plane_detection/catkin_plane_detection_test_ws/src/plane_detection/bag/seedmat_pic.png", seedmat);
  cv::imshow("seedmat", seedmat);
  cv::waitKey(0);
  cv::Mat neighbormat = color_image.clone();
  vector<quatree_node*> neighbors = q.getNeighbors(seed, width/20, height/20);
  for (auto & neighbor : neighbors)
  {
    if (neighbor->is_planar)
    {
      draw_planar_marker(neighbormat, neighbor->x * 20, neighbor->y * 20, neighbor->w * 20);
    }
    else
    {
      draw_nonplanar_marker(neighbormat, neighbor->x * 20, neighbor->y * 20, neighbor->w * 20);
    }
  }
  cv::imwrite("/home/humanoid/work/plane_detection/catkin_plane_detection_test_ws/src/plane_detection/bag/neighbormat_pic.png", neighbormat);
  cv::imshow("neighbormat", neighbormat);
  cv::waitKey(0);
  // for (size_t i = 0; i < width/20; i++)
  // {
  //   for (size_t j = 0; j < height/20; j++)
  //   {
  //     draw_planar_marker(color_image, i*20, j*20, 20);
  //   }
  // }
  // for (auto & nonplanar : nonplanar_index)
  // {
  //   draw_nonplanar_marker(color_image, nonplanar.first * 20, nonplanar.second * 20, 20);
  // }

  

  // draw_nonplanar_marker(color_image, 0, 0, 20);
  // draw_nonplanar_marker(color_image, 1*20, 0, 20);
  // draw_nonplanar_marker(color_image, 2*20, 0, 20);
  // draw_nonplanar_marker(color_image, 3*20, 0, 20);
  // draw_nonplanar_marker(color_image, 4*20, 0, 20);
  // draw_nonplanar_marker(color_image, 5*20, 0, 20);
  // draw_nonplanar_marker(color_image, 6*20, 0, 20);
  // draw_nonplanar_marker(color_image, 22*20, 0, 20);
  // draw_nonplanar_marker(color_image, 23*20, 0, 20);
  // draw_nonplanar_marker(color_image, 24*20, 0, 20);
  // draw_nonplanar_marker(color_image, 25*20, 0, 20);
  // draw_nonplanar_marker(color_image, 26*20, 0, 20);
  // draw_nonplanar_marker(color_image, 1*20, 1*20, 20);
  // draw_nonplanar_marker(color_image, 2*20, 1*20, 20);
  // draw_nonplanar_marker(color_image, 3*20, 1*20, 20);
  // draw_nonplanar_marker(color_image, 4*20, 1*20, 20);
  // draw_nonplanar_marker(color_image, 5*20, 1*20, 20);
  // draw_nonplanar_marker(color_image, 6*20, 1*20, 20);
  // draw_nonplanar_marker(color_image, 22*20, 1*20, 20);
  // draw_nonplanar_marker(color_image, 23*20, 1*20, 20);
  // draw_nonplanar_marker(color_image, 24*20, 1*20, 20);
  // draw_nonplanar_marker(color_image, 25*20, 1*20, 20);
  // draw_nonplanar_marker(color_image, 26*20, 1*20, 20);
  // draw_nonplanar_marker(color_image, 2*20, 2*20, 20);
  // draw_nonplanar_marker(color_image, 3*20, 2*20, 20);
  // draw_nonplanar_marker(color_image, 4*20, 2*20, 20);
  // draw_nonplanar_marker(color_image, 5*20, 2*20, 20);
  // draw_nonplanar_marker(color_image, 6*20, 2*20, 20);
  // draw_nonplanar_marker(color_image, 22*20, 2*20, 20);
  // draw_nonplanar_marker(color_image, 23*20, 2*20, 20);
  // draw_nonplanar_marker(color_image, 24*20, 2*20, 20);
  // draw_nonplanar_marker(color_image, 25*20, 2*20, 20);
  // draw_nonplanar_marker(color_image, 26*20, 2*20, 20);
  // draw_nonplanar_marker(color_image, 23*20, 3*20, 20);
  // draw_nonplanar_marker(color_image, 24*20, 3*20, 20);
  // draw_nonplanar_marker(color_image, 25*20, 3*20, 20);
  // draw_nonplanar_marker(color_image, 26*20, 3*20, 20);
  // draw_nonplanar_marker(color_image, 24*20, 4*20, 20);
  // draw_nonplanar_marker(color_image, 25*20, 4*20, 20);
  // draw_nonplanar_marker(color_image, 26*20, 4*20, 20);
  // draw_nonplanar_marker(color_image, 13*20, 5*20, 20);
  // draw_nonplanar_marker(color_image, 14*20, 5*20, 20);
  // draw_nonplanar_marker(color_image, 15*20, 5*20, 20);
  // draw_nonplanar_marker(color_image, 16*20, 5*20, 20);
  // draw_nonplanar_marker(color_image, 17*20, 5*20, 20);
  // draw_nonplanar_marker(color_image, 18*20, 5*20, 20);
  // draw_nonplanar_marker(color_image, 19*20, 5*20, 20);
  // draw_nonplanar_marker(color_image, 20*20, 5*20, 20);
  // draw_nonplanar_marker(color_image, 21*20, 5*20, 20);
  // draw_nonplanar_marker(color_image, 25*20, 5*20, 20);
  // draw_nonplanar_marker(color_image, 26*20, 5*20, 20);
  // draw_nonplanar_marker(color_image, 9*20, 6*20, 20);
  // draw_nonplanar_marker(color_image, 21*20, 6*20, 20);
  // draw_nonplanar_marker(color_image, 26*20, 6*20, 20);
  // draw_nonplanar_marker(color_image, 9*20, 7*20, 20);
  // draw_nonplanar_marker(color_image, 21*20, 7*20, 20);
  // draw_nonplanar_marker(color_image, 21*20, 8*20, 20);
  // draw_nonplanar_marker(color_image, 22*20, 9*20, 20);
  // draw_nonplanar_marker(color_image, 22*20, 10*20, 20);
  // draw_nonplanar_marker(color_image, 22*20, 11*20, 20);
  // draw_nonplanar_marker(color_image, 20*20, 12*20, 20);
  // draw_nonplanar_marker(color_image, 21*20, 12*20, 20);
  // draw_nonplanar_marker(color_image, 22*20, 12*20, 20);
  // draw_nonplanar_marker(color_image, 10*20, 13*20, 20);
  // draw_nonplanar_marker(color_image, 11*20, 13*20, 20);
  // draw_nonplanar_marker(color_image, 12*20, 13*20, 20);
  // draw_nonplanar_marker(color_image, 13*20, 13*20, 20);
  // draw_nonplanar_marker(color_image, 14*20, 13*20, 20);
  // draw_nonplanar_marker(color_image, 15*20, 13*20, 20);
  // draw_nonplanar_marker(color_image, 16*20, 13*20, 20);
  // draw_nonplanar_marker(color_image, 17*20, 13*20, 20);
  // draw_nonplanar_marker(color_image, 18*20, 13*20, 20);

  return 0;
}