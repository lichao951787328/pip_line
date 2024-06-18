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
  std::cout<<planes.at(0).contours.at(0).size()<<std::endl;
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
  std::cout<<"test boost"<<std::endl;
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


  std::cout<<"obstacle polygon"<<std::endl;
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
    showPolygon(tmppolygon);
    obstacle_polygons.emplace_back(tmppolygon);
    std::cout<<"polygon start point "<<tmppolygon.outer().begin()->x()<<" "<<tmppolygon.outer().begin()->y()<<std::endl;
    std::cout<<"polygon end point "<<(tmppolygon.outer().end()-1)->x()<<" "<<(tmppolygon.outer().end()-1)->y()<<std::endl;
  }
  std::cout<<"obstacles size is "<<obstacle_polygons.size()<<std::endl;
  // for (auto & p : obstacle_polygons)
  // {
  //   showPolygon(p);
  // }
  std::cout<<"landed area calculate"<<std::endl;
  std::cout<<"plane size "<<planes_ROBOTWORLD.size()<<std::endl;
  vector<plane_info> planes_landed;
  int index_plane = 0;
  for (auto & plane : planes_ROBOTWORLD)
  {
    std::cout<<"processing index "<<index_plane<<endl;
    index_plane++;
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
    std::cout<<"polygon inner size "<<tmppolygon.inners().size()<<endl;
    showPolygon(tmppolygon);

    // std::cout<<"get plane polygon over."<<std::endl;
    std::vector<polygon> result_polygons;
    vector<polygon> inputs;
    inputs.emplace_back(tmppolygon);
    for (size_t i = 0; i < obstacle_polygons.size(); i++)
    {
      std::cout<<"obstacle "<<i<<endl;

      // 每一个input均需要进行一次求取intersections
      std::vector<polygon> intersections;
      // 平面与障碍的相交平面
      boost::geometry::intersection(tmppolygon, obstacle_polygons.at(i), intersections);
      std::cout<<"intersections size "<<intersections.size()<<endl;
      for (auto & intersection : intersections)
      {
        showPolygon(intersection);
      }
      
      // tmppolygon必须减去所有的相交部分 这里要注意
      
      for (size_t j = 0; j < intersections.size(); j++)
      {
        cout<<"inputs size "<<inputs.size()<<endl;
        std::vector<std::list<polygon>> output_polygon;
        output_polygon.resize(inputs.size());
        // 每个多边形均需减去相交部分
        for (size_t k = 0; k < inputs.size(); k++)
        {
          boost::geometry::difference(inputs.at(k), intersections.at(j), output_polygon.at(k));
          cout<<"output_polygon "<<k<<" size "<<output_polygon.at(k).size()<<endl;
          for (auto & p : output_polygon.at(k))
          {
            showPolygon(p);
          }
        }
        
        inputs.clear();
        for (auto & polygon_list : output_polygon)
        {
          for (auto & polygon_single : polygon_list)
          {
            inputs.emplace_back(polygon_single);
          }
        }
      }
      result_polygons = inputs;
    }
    std::cout<<"show result..."<<endl;
    for (auto & p : result_polygons)
    {
      cout<<"outer: "<<endl;
      for (auto & point : p.outer())
      {
        std::cout<<"x = "<<point.x()<<" y = "<<point.y()<<" ";
      }
      cout<<endl;
      cout<<"inners: "<<endl;
      for (auto & inner : p.inners())
      {
        for (auto & point : inner)
        {
          std::cout<<"x = "<<point.x()<<" y = "<<point.y()<<" ";
        }
        cout<<endl;
      }
      showPolygon(p);
    }

    cout<<"result_polygons size "<<result_polygons.size()<<endl;
    float d_neg =  plane.center.dot(plane.normal);
    for (auto & polygon : result_polygons)
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


  

  return planes_landed;
  std::cout<<"test over"<<std::endl;
  // opencv找到轮廓的外轮廓为逆时针，内轮廓为顺时针
  // 可以通过记录findcontours的第三个参数来找到外轮廓，这里先默认第一个是外轮廓
  // vector<plane_info> planes_landed;
  // planes_landed.reserve(planes.size());
  // for (auto & plane : planes_ROBOTWORLD)
  // {
  //   // 得到平面的boost 多边形表达
  //   polygon tmppolygon;
  //   bool detect_outer = false;
  //   tmppolygon.inners().resize(plane.contours.size() - 1);
  //   for (auto & contour : plane.contours)
  //   {
  //     if (!detect_outer)// 证明是外轮廓此时
  //     {
  //       for (auto & point : contour)
  //       {
  //         boost::geometry::append(tmppolygon.outer(), point_t(point.x(), point.y()));
  //       }
  //       detect_outer = true;
  //     }
  //     else
  //     {
  //       int i = 0;
  //       for (auto & point : contour)
  //       {
  //         boost::geometry::append(tmppolygon.inners()[i], point_t(point.x(), point.y()));
  //         i++;
  //       }
  //     }
  //   }
    
  //   // 求多边形与障碍的交集
  //   for (auto & obstacle : obstacles)
  //   {
  //     polygon obstacle_polygon;
  //     for (auto & point : obstacle)
  //     {
  //       boost::geometry::append(obstacle_polygon.outer(), point_t(point.x(), point.y()));
  //     }
  //     std::deque<polygon> intersections;
  //     boost::geometry::intersection(tmppolygon, obstacle_polygon, intersections);
  //     for (auto & intersection : intersections)
  //     {
  //       std::list<polygon> output;
  //       boost::geometry::difference(tmppolygon, intersection, output);
  //     }
  //   }
  // }

}