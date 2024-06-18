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
/*
 * @description: 
 * @param : 
 * @return: 
 */
// 先分割成小块，再进行合并

#ifndef QUADTREE_H_
#define QUADTREE_H_
#include <iostream>
#include <queue>
#include <limits.h>
#include <list>
#include <vector>
#include "plane_detection/quatree_node_20240618.h"
#include "plane_detection/type.h"
namespace quatree
{
// 先一股脑的分成最小块，再进行合并，利用本身的的长宽来确定那些节点不用再分
class quatree
{
protected:
  node* root;
  
  void mergePatchs(node* p);
  void clear(node* p);
  
  // void refreshNodeParam(node* p);
  bool check(node * p);
public:
  index2D* pindex2d;
  
  orginazed_points raw_points;
  parameter param;
  quatree(orginazed_points & org_points_, parameter & param_);
  ~quatree();
  void mergePatchsForSeeds();
  /**
   * @description: get seed node
   * @return {*}
   */  
  node* getSeedNode();
  void refreshIndex2D();
  std::priority_queue<node*, std::vector<node*>, compnode> getPatchs(); 
  std::list<node*> getPatchsList();
  /**
   * @description: if some node in quatree has deleted, check the tree and refresh it
   * @return {*}
   */  
  void refreshTree();


  void getQuatreeNeighbors();

  void showLeafNodeImage();
  void showMergeLeafNodeImage();
  void showSeedNodeAndNeighbors();
  bool checkNodeChildrenIsPlaneMean(node* p);
  bool checkNodeChildrenIsPlane(node* p);
  void refreshNodeParam(node* p);
  std::vector<node*> getLeafnodes();
  void showFrame();
  // void saveAsExcel(string filePath);
  void showPatchInfo();
  inline node* getRoot()
  {
    return root;
  }

};
bool checkNodeChildrenIsPlane(node* p);
bool checkNodeChildrenIsPlaneMean(node* p);
void refreshNodeParam(node* p);
}// end nameapce
#endif