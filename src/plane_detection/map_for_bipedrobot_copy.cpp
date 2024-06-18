/*
 * @description: 
 * @param : 
 * @return: 
 */

#include <plane_detection/map_for_bipedrobot.h>
#include <deque>
#include <list>
#include <boost/foreach.hpp>
#include "matplotlibcpp.h"
namespace plt = matplotlibcpp;
#define SHOWIMAGE
// #define DEBUG


void showPolygon(polygon & p)
{
  
  vector<float> x_outer, y_outer;
  for (auto & point : p.outer())
  {
    x_outer.emplace_back(point.x());
    y_outer.emplace_back(point.y());
  }
  

  vector<vector<float>> x_inners, y_inners;
  for (auto & inner : p.inners())
  {
    vector<float> x_inner, y_inner;
    for (auto & point : inner)
    {
      x_inner.emplace_back(point.x());
      y_inner.emplace_back(point.y());
    }
    x_inners.emplace_back(x_inner);
    y_inners.emplace_back(y_inner);
  }

  plt::plot(x_outer, y_outer);
  for (size_t i = 0; i < x_inners.size(); i++)
  {
    plt::plot(x_inners.at(i), y_inners.at(i));
  }
  plt::show();

  std::cout<<"polygon show over."<<std::endl;
}

vector<plane_info> getLandedArea(vector<plane_info> & planes, vector<vector<Eigen::Vector2f>> obstacles, Eigen::Matrix4f & ROBOTWORLD_T_CAMERA)
{
  // for (auto & plane : planes)
  // {
  //   for (auto & contour : plane.contours)
  //   {
  //     vector<float> x, y;
  //     for (auto & point_3d : contour)
  //     {
  //       x.emplace_back(point_3d.x());
  //       y.emplace_back(point_3d.y());
  //     }
  //     x.emplace_back(*x.begin());
  //     y.emplace_back(*y.begin());
  //     // plt::plot(x, y);
  //     // plt::show();
  //   }
  // }
  
  // 转换到机器人世界坐标系
#ifdef DEBUG
  std::cout<<planes.at(0).contours.at(0).size()<<std::endl;
#endif
  vector<plane_info> planes_ROBOTWORLD;
  planes_ROBOTWORLD.reserve(planes.size());
  for (auto & plane : planes)
  {
    planes_ROBOTWORLD.emplace_back(plane.transform(ROBOTWORLD_T_CAMERA));
  }
  
  // 显示轮廓图像
  // 保证多边形都是逆时针
// #ifdef SHOWIMAGE
//   std::cout<<"enter getLandedArea func:"<<std::endl;
//   for (auto & plane : planes_ROBOTWORLD)
//   {
//     if (plane.contours.size() == 0)
//     {
//       std::cout<<"error plane information."<<std::endl;
//     }
//     else
//     {
//       for (auto & points_3d : plane.contours)
//       {
//         cout<<"contours "<<endl;
//         vector<float> x;
//         vector<float> y;
//         for (auto & point_3d : points_3d)
//         {
//           cout<<"x = "<<point_3d.x()<<" y = "<<point_3d.y()<<endl;
//           x.emplace_back(point_3d.x());
//           y.emplace_back(point_3d.y());
//         }
//         x.emplace_back(*x.begin());
//         y.emplace_back(*y.begin());
//         plt::plot(x, y);
//         plt::show();
//       }
//     }
//   }
//   // const char* filename = "./basic.png";
//   // std::cout << "Saving result to " << filename << std::endl;;
//   // plt::save(filename);

//   std::cout<<"obstacles contours: "<<endl;
//   for (auto & polygon : obstacles)
//   {
//     vector<float> x_tmp;
//     vector<float> y_tmp;
//     for (auto & point_2d : polygon)
//     {
//       x_tmp.emplace_back(point_2d.x());
//       y_tmp.emplace_back(point_2d.y());
//     }
//     x_tmp.emplace_back(*x_tmp.begin());
//     y_tmp.emplace_back(*y_tmp.begin());
//     plt::plot(x_tmp, y_tmp);
//     plt::show();
//   }

// #endif
  // std::cout<<"test boost"<<std::endl;
  // polygon test_polygon;
  // for (auto & point : *(planes_ROBOTWORLD.begin()->contours.begin()))
  // {
  //   boost::geometry::append(test_polygon.outer(), point_t(point.x(), point.y()));
  // }

  // vector<polygon> polygons;
  // polygons.reserve(planes.size());
  // polygon tmppolygon;
  // if (planes.begin()->contours.size() > 1)
  // {
  //   tmppolygon.inners().resize(planes.begin()->contours.size() - 1);
  //   for (size_t i = 0; i < planes.begin()->contours.size(); i++)
  //   {
  //     if (i == 0)
  //     {
  //       for (auto & point :planes_ROBOTWORLD.begin()->contours.at(i))
  //       {
  //         boost::geometry::append(tmppolygon.outer(), point_t(point.x(), point.y()));
  //       }
  //     }
  //     else
  //     {
  //       for (auto & point :planes_ROBOTWORLD.begin()->contours.at(i))
  //       {
  //         boost::geometry::append(tmppolygon.inners()[i-1], point_t(point.x(), point.y()));     
  //       }
  //     }
  //   }
  // }
  // else
  // {
  //   for (auto & point : *planes_ROBOTWORLD.begin()->contours.begin())
  //   {
  //     boost::geometry::append(test_polygon.outer(), point_t(point.x(), point.y()));
  //   }
  // }


  // std::cout<<"obstacle polygon"<<std::endl;
  vector<polygon> obstacle_polygons;
  obstacle_polygons.reserve(obstacles.size());
  for (auto & obstacle : obstacles)
  {
    polygon tmppolygon;
    for (auto & point : obstacle)
    {
      boost::geometry::append(tmppolygon.outer(), point_t((point.x()), (point.y())));
      // boost::geometry::append(tmppolygon.outer(), point_t(correct2threeDecimal(point.x()), correct2threeDecimal(point.y())));
    }
    boost::geometry::append(tmppolygon.outer(), *tmppolygon.outer().begin());
#ifdef DEBUG
    showPolygon(tmppolygon);
#endif
    obstacle_polygons.emplace_back(tmppolygon);
#ifdef DEBUG
    std::cout<<"polygon start point "<<tmppolygon.outer().begin()->x()<<" "<<tmppolygon.outer().begin()->y()<<std::endl;
    std::cout<<"polygon end point "<<(tmppolygon.outer().end()-1)->x()<<" "<<(tmppolygon.outer().end()-1)->y()<<std::endl;
#endif
  }
#ifdef DEBUG
  std::cout<<"obstacles size is "<<obstacle_polygons.size()<<std::endl;
#endif
  // for (auto & p : obstacle_polygons)
  // {
  //   showPolygon(p);
  // }
#ifdef DEBUG
  std::cout<<"landed area calculate"<<std::endl;
  std::cout<<"plane size "<<planes_ROBOTWORLD.size()<<std::endl;
#endif
  vector<plane_info> planes_landed;
  for (auto & plane : planes_ROBOTWORLD)
  {
    std::cout<<plane.contours.size()<<endl;
    polygon tmppolygon;
    if (plane.contours.size() > 1)
    {
      tmppolygon.inners().resize(plane.contours.size() - 1);
      for (size_t i = 0; i < plane.contours.size(); i++)
      {
        if (i == 0)
        {
          for (auto & point : plane.contours.at(i))
          {
            boost::geometry::append(tmppolygon.outer(), point_t((point.x()), (point.y())));
            // boost::geometry::append(tmppolygon.outer(), point_t(correct2threeDecimal(point.x()), correct2threeDecimal(point.y())));
          }
          boost::geometry::append(tmppolygon.outer(), *tmppolygon.outer().begin());
        }
        else
        {
          for (auto & point : plane.contours.at(i))
          {
            boost::geometry::append(tmppolygon.inners()[i - 1], point_t((point.x()), (point.y())));
            // boost::geometry::append(tmppolygon.inners()[i - 1], point_t(correct2threeDecimal(point.x()), correct2threeDecimal(point.y())));
          }
          boost::geometry::append(tmppolygon.inners()[i - 1], *tmppolygon.inners()[i - 1].begin());
        }
      }
    }
    else
    {
      for (auto & point : *plane.contours.begin())
      {
        boost::geometry::append(tmppolygon.outer(), point_t((point.x()), (point.y())));
        // boost::geometry::append(tmppolygon.outer(), point_t(correct2threeDecimal(point.x()), correct2threeDecimal(point.y())));
      }
      boost::geometry::append(tmppolygon.outer(), *tmppolygon.outer().begin());
    }
#ifdef DEBUG
    std::cout<<"polygon inner size "<<tmppolygon.inners().size()<<endl;
    std::cout<<"show tmp polygon..."<<std::endl;
    showPolygon(tmppolygon);
#endif
    vector<polygon> unions;
    vector<polygon> inputs;
    inputs.emplace_back(tmppolygon);
    for (size_t i = 0; i < obstacle_polygons.size(); i++)
    {
      vector<vector<polygon>> outputs;
      outputs.resize(inputs.size());
      for (size_t j = 0; j < inputs.size(); j++)
      {
        boost::geometry::union_(inputs.at(j), obstacle_polygons.at(j), outputs.at(j));
      }
      inputs.clear();
      for (auto & polygons : outputs)
      {
        for (auto & polygon : polygons)
        {
          inputs.emplace_back(polygon);
        }
      }
    }
    unions = inputs;
#ifdef DEBUG
    std::cout<<"show unions..."<<std::endl;
    std::cout<<"unions size "<<unions.size()<<std::endl;
    for (auto & polygon : unions)
    {
      showPolygon(polygon);
    }
#endif
    vector<polygon> results_polygon;
    for (auto & union_single : unions)
    {
      vector<vector<polygon>> output_diff;
      vector<polygon> inputs_diff;
      inputs_diff.emplace_back(union_single);
      for (size_t i = 0; i < obstacle_polygons.size(); i++)
      {
        output_diff.clear();
        output_diff.resize(inputs_diff.size());
        for (size_t j = 0; j < inputs_diff.size(); j++)
        {
          boost::geometry::difference(inputs_diff.at(j), obstacle_polygons.at(i), output_diff.at(j));
        }
        inputs_diff.clear();
        for (auto & diff_polygons : output_diff)
        {
          for (auto & diff_polygon : diff_polygons)
          {
            inputs_diff.emplace_back(diff_polygon);
          }
        }
      }
      results_polygon.insert(results_polygon.end(), inputs_diff.begin(), inputs_diff.end());
    }
#ifdef DEBUG
    for (auto & draw_polygon : results_polygon)
    {
      showPolygon(draw_polygon);
    }
#endif
    // 转化成多边形
    float d_neg =  plane.center.dot(plane.normal);
    for (auto & polygon : results_polygon)
    {
      plane_info tmp_polygon;
      tmp_polygon.center = plane.center;
      tmp_polygon.normal = plane.normal;
      vector<Eigen::Vector3f> contour;
      for (auto & point_2d : polygon.outer())
      {
        float z = (d_neg - plane.normal.x()*point_2d.x() - plane.normal.y()*point_2d.y())/plane.normal.z();
        contour.emplace_back(Eigen::Vector3f(point_2d.x(), point_2d.y(), z));
      }
      tmp_polygon.contours.emplace_back(contour);
      for (auto & inner_contour : polygon.inners())
      {
        vector<Eigen::Vector3f> inner_contour_plane;
        for (auto & point_2d : inner_contour)
        {
          float z = (d_neg - plane.normal.x()*point_2d.x() - plane.normal.y()*point_2d.y())/plane.normal.z();
          inner_contour_plane.emplace_back(Eigen::Vector3f(point_2d.x(), point_2d.y(), z));
        }
        tmp_polygon.contours.emplace_back(inner_contour_plane);
      }
      planes_landed.emplace_back(tmp_polygon);
    }
  }
  std::cout<<"planes_landed size "<<planes_landed.size()<<std::endl;
#ifdef DEBUG
  vector<polygon> polygons_check;
  for (auto & plane_landed : planes_landed)
  {
    polygon tmppolygon;
    if (plane_landed.contours.size() > 1)
    {
      for (auto & point : plane_landed.contours.at(0))
      {
        boost::geometry::append(tmppolygon.outer(), point_t((point.x()), (point.y())));
      }
      tmppolygon.inners().resize(plane_landed.contours.size() - 1);
      for (size_t i = 1; i < plane_landed.contours.size(); i++)
      {
        for (auto & point : plane_landed.contours.at(i))
        {
          boost::geometry::append(tmppolygon.inners()[i - 1], point_t((point.x()), (point.y())));
        }
      }
    }
    else
    {
      for (auto & point : plane_landed.contours.at(0))
      {
        boost::geometry::append(tmppolygon.outer(), point_t((point.x()), (point.y())));
      }
    }
    polygons_check.emplace_back(tmppolygon);
  }
  cout<<"show result..."<<endl;
  for (auto & p : polygons_check)
  {
    showPolygon(p);
  }
#endif
  return planes_landed;
}