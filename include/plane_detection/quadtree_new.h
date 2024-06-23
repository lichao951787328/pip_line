/*
 * @Author: lichao951787328 951787328@qq.com
 * @Date: 2024-06-18 22:26:10
 * @LastEditors: lichao951787328 951787328@qq.com
 * @LastEditTime: 2024-06-20 10:21:25
 * @FilePath: /pip_line/include/plane_detection/quadtree_new.h
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
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
  std::shared_ptr<node> root;
  
  void mergePatchs(std::shared_ptr<node> p);
  void clear(std::shared_ptr<node> p);
  
  // void refreshNodeParam(node* p);
  bool check(std::shared_ptr<node> p);
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
  std::shared_ptr<node> getSeedNode();
  void refreshIndex2D();
  std::priority_queue<std::shared_ptr<node>, std::vector<std::shared_ptr<node>>, reverseComnode> getPatchs(); 
  std::list<std::shared_ptr<node>> getPatchsList();
  /**
   * @description: if some node in quatree has deleted, check the tree and refresh it
   * @return {*}
   */  
  void refreshTree();

  void PreRefreshIndex2D();
  void getQuatreeNeighbors();
  void printfNeighbors();
  void showLeafNodeImage();
  void showMergeLeafNodeImage();
  void showSeedNodeAndNeighbors();
  bool checkNodeChildrenIsPlaneMean(std::shared_ptr<node> p);
  bool checkNodeChildrenIsPlane(std::shared_ptr<node> p);
  void refreshNodeParam(std::shared_ptr<node> p);
  std::vector<std::shared_ptr<node>> getLeafnodes();
  // void showFrame();
  // void saveAsExcel(string filePath);
  void showPatchInfo();
  inline std::shared_ptr<node> getRoot()
  {
    return root;

  }

};
bool checkNodeChildrenIsPlane(std::shared_ptr<node> p);
bool checkNodeChildrenIsPlaneMean(std::shared_ptr<node> p);
void refreshNodeParam(std::shared_ptr<node> p);
}// end nameapce
#endif