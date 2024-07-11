/*
 * @description: 
 * @param : 
 * @return: 
 */




#ifndef _PLANE_SEGMENTATION_H_
#define _PLANE_SEGMENTATION_H_

#include <plane_detection/plane_new.h>
#include <plane_detection/quadtree_new.h>

#include <vector>
using namespace std;
class plane_segmentation
{
private:
  size_t segmented_index;
  vector<plane_info> planes;
  cv::Mat segmented_result;
  cv::Mat segmented_contours;
  index2D index_node_2d;
  quatree::quatree* pqq;
  std::vector<int> colors_;
  std::vector<cv::Mat> seg_images;
  orginazed_points raw_points;
  parameter param;
public:
  /**
   * @description: creat plane segmentation class
   * @param {quatree &} qq
   * @return {*}
   */
  plane_segmentation(size_t rows, size_t cols, quatree::quatree * pqq_, orginazed_points & raw_points_, parameter & param_);
  void getSinglePlaneResult();
  // void getPlane(plane & plane);
  inline vector<plane_info> getPlaneResult()
  {
    return planes;
  }

  inline cv::Mat getSegResult()
  {
    return segmented_result;
  }

  inline vector<cv::Mat> getSegResultSingle()
  {
    return seg_images;
  }
  void showNodeAndNeisImage(std::shared_ptr<quatree::node> n);
  ~plane_segmentation();
};




#endif