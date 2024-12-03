#include <iostream>

#include <grid_map_ros/grid_map_ros.hpp>
#include <grid_map_msgs/GetGridMap.h>
#include <grid_map_core/iterators/SubmapIterator.hpp>
#include <grid_map_core/iterators/CircleIterator.hpp>
#include <grid_map_core/iterators/SpiralIterator.hpp>
#include <grid_map_core/iterators/GridMapIterator.hpp>
#include <thread> // 必须包含此头文件
#include <chrono> // 用于时间单位
#include <opencv2/opencv.hpp>
#include <sensor_msgs/Image.h>
#include <random>
// #include <AstarHierarchicalFootstepPlanner/AstarHierarchicalFootstepPlannerPropose.h>
// #include <AstarHierarchicalFootstepPlanner/AstarHierarchicalFootstepPlannerTraditional.h>
#include <various_terrain_planner.h>
using namespace std;

#define GENERAL //yiban taijie dixing
// #define WAVE // polang dixing

variousTerrainPlanner::variousTerrainPlanner(ros::NodeHandle nh_):nh(nh_)
{
    map_pub = nh.advertise<grid_map_msgs::GridMap>("map", 1, true);
    // nh.param("foot_param_x_upper", foot_param.x_upper, 0.16);
    // nh.param("foot_param_x_button", foot_param.x_button, 0.11);
    // nh.param("foot_param_y_left", foot_param.y_left, 0.065);
    // nh.param("foot_param_y_right", foot_param.y_right, 0.065);
    // nh.param("foot_param_x_fore_button", foot_param.x_fore_button, 0.1);
    // nh.param("foot_param_x_hind_top", foot_param.x_hind_top, 0.);
    nh.param("hip_width", hip_width, 0.2);
    nh.param("checkXupper", checkXupper, 0.13);
    nh.param("checkXButton", checkXButton, 0.04);

 
}

// bool variousTerrainPlanner::CheckFeasibleGoal(grid_map::GridMap & map, int radius, vector<Eigen::Vector3d> & goal_points_final)
// {
//     if (local_planner_traditional.getPlannerPtr() && local_planner_propose.getPlannerPtr())
//     {
//         vector<Eigen::Vector2d> goal_points;
//         goal_points.emplace_back(Eigen::Vector2d(4.6, 0));
//         // goal_points.emplace_back(Eigen::Vector2d(3.8, 0));
//         // goal_points.emplace_back(Eigen::Vector2d(4.4, 2));
//         // goal_points.emplace_back(Eigen::Vector2d(4.4, -2));
//         // goal_points.emplace_back(Eigen::Vector2d(3.4, 2));
//         // goal_points.emplace_back(Eigen::Vector2d(3.4, -2));
//         for (auto & tmp_goal_point : goal_points)
//         {
//             bool goal_flag = false;
//             grid_map::SpiralIterator iterator(map, tmp_goal_point, radius);
//             while (!iterator.isPastEnd())
//             {
//                 grid_map::Position position;
//                 map.getPosition(*iterator, position);
//                 vector<Eigen::Vector3d>goal_points;
                
//                 // 0度
//                 goal_points.emplace_back(Eigen::Vector3d(position.x(), position.y(), 0));

//                 // 5度
//                 goal_points.emplace_back(Eigen::Vector3d(position.x(), position.y(), 5/57.3));

//                 // -5度
//                 goal_points.emplace_back(Eigen::Vector3d(position.x(), position.y(), -5/57.3));

//                 // 10度
//                 goal_points.emplace_back(Eigen::Vector3d(position.x(), position.y(), 10/57.3));

//                 // -10度
//                 goal_points.emplace_back(Eigen::Vector3d(position.x(), position.y(), -10/57.3));

//                 for (auto & goal_point : goal_points)
//                 {
//                     if (local_planner_traditional.isGoalFeasible(goal_point) && local_planner_propose.isGoalFeasible(goal_point))
//                     {
//                         goal_points_final.emplace_back(goal_point);
//                         goal_flag = true;
//                         break; 
//                     }
//                 }
//                 if (goal_flag)
//                 {
//                     break;
//                 }
//                 ++iterator;
//             }
//         }
//         if (goal_points_final.empty())
//         {
//             return false;
//         }
//         else
//         {
//             return true;
//         }
        
//     }
//     else
//     {
//         return false;
//     }
    
// }

// bool variousTerrainPlanner::checkFeasibleStart(grid_map::GridMap & map, Eigen::Vector3d & left_foot_tra, Eigen::Vector3d & left_right_tra, Eigen::Vector3d & propose_left_foot, Eigen::Vector3d & propose_right_foot)
// {
//     grid_map::Position start(0.15, 0);
//     grid_map::SpiralIterator iterator(map, start, 0.1);
//     while (!iterator.isPastEnd())
//     {
//         grid_map::Position position;
//         map.getPosition(*iterator, position);
//         vector<Eigen::Vector3d> start_points;
        
//         // 0度
//         start_points.emplace_back(Eigen::Vector3d(position.x(), position.y(), 0));

//         // 5度
//         start_points.emplace_back(Eigen::Vector3d(position.x(), position.y(), 5/57.3));

//         // -5度
//         start_points.emplace_back(Eigen::Vector3d(position.x(), position.y(), -5/57.3));

//         // 10度
//         start_points.emplace_back(Eigen::Vector3d(position.x(), position.y(), 10/57.3));

//         // -10度
//         start_points.emplace_back(Eigen::Vector3d(position.x(), position.y(), -10/57.3));

//         for (auto & tmp_start_point : start_points)
//         {
//             // 两种情况均成立，才以此点为终点
//             if (local_planner_traditional.isStartFeasible(tmp_start_point, left_foot_tra, left_right_tra) && local_planner_propose.isStartFeasible(tmp_start_point, propose_left_foot, propose_right_foot))
//             {
//                 return true; 
//             }
//             else
//             {
//                 return false;
//             }
            
//         }
//         ++iterator;
//     }
//     return false;
// }

// void variousTerrainPlanner::traditionalPlanner() 
// {
//     std::cout << "Starting traditional_localplaner.plan()...\n";
//     local_planner_traditional.plan();
//     std::cout << "traditional_localplaner.plan() completed successfully.\n";
// }

// void variousTerrainPlanner::proposePlanner() 
// {
//     std::cout << "Starting propose_localplaner.plan()...\n";
//     // 执行新的任务逻辑
//     local_planner_propose.plan();
//     std::cout << "propose_localplaner.plan() completed.\n";
// }

// void variousTerrainPlanner::executeTaskWithTimeout(std::function<void()> task, int timeout) 
// {
//     // 使用 std::async 异步执行任务
//     auto future = std::async(std::launch::async, task);

//     // 等待任务完成或超时
//     if (future.wait_for(std::chrono::seconds(timeout)) == std::future_status::timeout) {
//         std::cout << "Task timed out, skipping to the next task...\n";
//     } else {
//         std::cout << "Task completed within the time limit.\n";
//     }
// }

void variousTerrainPlanner::execute()
{
    double map_length = 5.0;
    double map_width = 5.0;
    double resolution = 0.02;
    // 输入多变的地形,台阶宽度0.06-0.27，间隙0.02-0.17

    double step_elevation = 0.2;
    double gap_elevation = 0.0;
using namespace grid_map;
GridMap map({"elevation"});
  map.setFrameId("map");
  map.setGeometry(Length(1.2, 2.0), 0.03);
  ROS_INFO("Created map with size %f x %f m (%i x %i cells).",
    map.getLength().x(), map.getLength().y(),
    map.getSize()(0), map.getSize()(1));
for (GridMapIterator it(map); !it.isPastEnd(); ++it) {
      Position position;
      map.getPosition(*it, position);
      map.at("elevation", *it) =0;
    }
}
    // grid_map::GridMap map({"elevation"});
    // grid_map::GridMap map;
    
    // map.setFrameId("map");
    // map.setGeometry(grid_map::Length(map_length, map_width), resolution, grid_map::Position(map_length/2.0, 0));
    // map.add("elevation", 0.0);

    // grid_map::GridMap map({"elevation"});
    // map.setFrameId("map");
    // map.setGeometry(grid_map::Length(map_length, map_width), 0.02, grid_map::Position(map_length/2.0, 0));
    // // ROS_INFO("Created map with size %f x %f m (%i x %i cells).",
    // // map.getLength().x(), map.getLength().y(),
    // // map.getSize()(0), map.getSize()(1));

    // for (grid_map::GridMapIterator it(map); !it.isPastEnd(); ++it) 
    // {
    //     grid_map::Position position;
    //     map.getPosition(*it, position);
    //     map.at("elevation", *it) = 0;
    // }
//     while (ros::ok())
//     {
// #ifdef GENERAL
//         for (int i = 0; i <= 1; i++) //9
//         {
//             for (int j = 0; j <= 1; j++) //8
//             {
//                 // double step_width = 0.06 +resolution*i;
//                 // double gap_width = 0.02 + resolution*j;
//                 // int step_index_length = step_width/resolution;
//                 // int gap_index_length = gap_width/resolution;
//                 // map.clearAll();
//                 // cout<<step_width<<" "<<gap_width<<endl;
//                 // int index_length = 0;
//                 // while (index_length < map.getSize().x())
//                 // {
//                 //     for (int i_x = 0; i_x < step_index_length; i_x++)
//                 //     {
//                 //         for(int j_y = 0; j_y < map.getSize().y(); j_y++)
//                 //         {
//                 //             if (i_x + index_length < map.getSize().x())
//                 //             {
//                 //                 map["elevation"](i_x + index_length, j_y) = step_elevation;
//                 //             }
//                 //         }
//                 //     }
//                 //     index_length = index_length + step_index_length;
//                 //     if (index_length >= map.getSize().x())
//                 //     {
//                 //         break;
//                 //     }
//                 //     for (int i_x = 0; i_x < gap_index_length; i_x++)
//                 //     {
//                 //         for(int j_y = 0; j_y < map.getSize().y(); j_y++)
//                 //         {
//                 //             if (i_x + index_length < map.getSize().x())
//                 //             {
//                 //                 map["elevation"](i_x + index_length, j_y) = gap_elevation;
//                 //             }
//                 //         }
//                 //     }
//                 //     index_length = index_length + gap_index_length;
//                 // }

//                 grid_map_msgs::GridMap map_msg;
//                 grid_map::GridMapRosConverter::toMessage(map, map_msg);
//                 map_pub.publish(map_msg);
                
//                 std::shared_ptr<AstarHierarchicalFootstepPlannerTraditional> traditional_planner_ptr = std::make_shared<AstarHierarchicalFootstepPlannerTraditional>();
//                 std::shared_ptr<AstarHierarchicalFootstepPlannerPropose> propose_planner_ptr = std::make_shared<AstarHierarchicalFootstepPlannerPropose>();

//                 traditional_planner_ptr->setCheckParam(checkXupper, checkXButton);
                
//                 local_planner_traditional.setPlanner(traditional_planner_ptr);
//                 local_planner_traditional.setFootParam(foot_param);
//                 local_planner_traditional.setHipWidth(hip_width);
//                 // local_planner_traditional.mapPrepare(map);

//                 local_planner_propose.setPlanner(propose_planner_ptr);
//                 local_planner_propose.setFootParam(foot_param);
//                 local_planner_propose.setHipWidth(hip_width);
//                 // local_planner_propose.mapPrepare(map);
                
//                 // Eigen::Vector3d traditional_left_foot, traditional_right_foot;
//                 // Eigen::Vector3d propose_left_foot, propose_right_foot;
//                 // vector<Eigen::Vector3d> goal_points_final;
//                 // if (CheckFeasibleGoal(map, 0.15, goal_points_final) && checkFeasibleStart(map, traditional_left_foot, traditional_right_foot, propose_left_foot, propose_right_foot))
//                 // {
//                 //     LOG(INFO)<<"traditional start: "<<traditional_left_foot.transpose()<<" "<<traditional_right_foot.transpose();
//                 //     LOG(INFO)<<"propose start: "<<propose_left_foot.transpose()<<" "<<propose_right_foot.transpose();
//                 //     if (!goal_points_final.empty())
//                 //     {
//                 //         LOG(INFO)<<"final goal: "<<goal_points_final[0].transpose();
//                 //         // for (auto & final_goal : goal_points_final)
//                 //         // {
//                 //         //     LOG(INFO)<<"final goal: "<<final_goal.transpose();
//                 //         //     local_planner_traditional.initial(traditional_left_foot, traditional_right_foot, 0, final_goal);
//                 //         //     local_planner_propose.initial(propose_left_foot, propose_right_foot, 0, final_goal);
//                 //         //     const int TIME_LIMIT = 60;
//                 //         //     // executeTaskWithTimeout(traditionalPlanner, TIME_LIMIT);
//                 //         //     // executeTaskWithTimeout(proposePlanner, TIME_LIMIT);
//                 //         //     executeTaskWithTimeout([this]() { traditionalPlanner(); }, TIME_LIMIT);
//                 //         //     executeTaskWithTimeout([this]() { proposePlanner(); }, TIME_LIMIT);
//                 //         //     LOG(INFO)<<"i: "<<i<<" j: "<<j;
//                 //         // }
//                 //         sleep(1);
//                 //     }
//                 //     else
//                 //     {
//                 //         LOG(INFO)<<"no suitable goal point";
//                 //     }
//                 // }
//                 LOG(INFO)<<"no suitable goal point";
//             }
//             LOG(INFO)<<"no suitable goal point";
//         }
//         cout<<"out"<<endl;
// #endif

// #ifdef WAVE
//         cv::Mat image = cv::Mat::zeros(map.getSize().x(), map.getSize().y(), CV_8UC1);
//         // 通过调节lineGap 和 phaseShift 来实现场景的变化
//         // 波浪线的参数
//         int amplitude = 20;  // 振幅（波的高度）
//         int frequency = 100;  // 波浪的频率（每个周期内的点数）

//         for (int i = 0; i <=9; i++)
//         {
//             for (int j = 0; j <= 8; j++)
//             {
//                 image = cv::Mat::zeros(map.getSize().x(), map.getSize().y(), CV_8UC1);
//                 map.clearAll();
//                 double step_width = 0.06 +resolution*(i+2);
//                 double gap_width = 0.02 + resolution*(j);
//                 int step_index_length = step_width/resolution;
//                 int gap_index_length = gap_width/resolution;

//                 int lineGap = step_index_length + gap_index_length;      // 每条波浪线之间的垂直间隔
//                 int thickness = step_index_length;     // 波浪线的厚度
//                 // int phaseShift = (int)(frequency/(2*(image.rows/lineGap)));

//                 // 循环绘制波浪线，覆盖整个图像
//                 for (int i = 0, offsetY = 0; offsetY < map.getSize().x(); ++i, offsetY += lineGap) 
//                 {
//                     // int xOffset = i * phaseShift; // 计算当前波浪线的横向偏移量

//                     std::random_device rd; // 随机数种子
//                     std::mt19937 gen(rd()); // 随机数生成器

//                     // 整数范围
//                     std::uniform_int_distribution<> dist_int(0, frequency/2); // 1到10
//                     int xOffset = dist_int(gen);

//                     for (int x = 0; x < map.getSize().y(); ++x) 
//                     {
//                         // 计算波浪线在 y 方向上的位置，加入横向偏移量
//                         int y = static_cast<int>(offsetY + amplitude * std::sin(2 * CV_PI * (x + xOffset) / frequency));
//                         if (x > 0) 
//                         {
//                             // 连接相邻的点，生成连续的波浪线
//                             cv::line(image, 
//                                     cv::Point(x - 1, static_cast<int>(offsetY + amplitude * std::sin(2 * CV_PI * ((x - 1) + xOffset) / frequency))),
//                                     cv::Point(x, y), 
//                                     cv::Scalar(255), // 白色波浪线
//                                     thickness);               // 波浪线的厚度
//                         }
//                     }
//                 }
//                 sensor_msgs::ImagePtr image_msg = cv_bridge::CvImage(std_msgs::Header(), "mono8", image).toImageMsg();
//                 image_pub.publish(image_msg);
//                 for (int x_i = 0; x_i < image.rows; x_i++)
//                 {
//                     for (int y_j = 0; y_j < image.cols; y_j++)
//                     {
//                         if (image.at<uchar>(x_i, y_j) == 255)
//                         {
//                             map["elevation"](x_i, y_j) = step_elevation;
//                         }
//                         else
//                         {
//                             map["elevation"](x_i, y_j) = gap_elevation;
//                         }
//                     }
                    
//                 }
//                 grid_map_msgs::GridMap map_msg;
//                 grid_map::GridMapRosConverter::toMessage(map, map_msg);
//                 map_pub.publish(map_msg);
//                 std::this_thread::sleep_for(std::chrono::seconds(2));

//                 localPlanner local_planner
//                 local_planner.setFootParam(foot_param);
//                 nh.param("hip_width", hip_width, 0.05);
//                 local_planner.setHipWidth(hip_width);
//                 local_planner.mapPrepare(map);

//                 // 找起始点
//                 bool start_flag = false;
//                 Eigen::Vector3d start_point;
//                 // 在初始圆内找起始点
//                 grid_map::Position center(0.15, 0);
//                 double radius = 0.1;
//                 grid_map::SpiralIterator iterator(map, center, radius);
//                 while (!iterator->isPastEnd())
//                 {
//                     grid_map::Position position;
//                     map.getPosition(*iterator, position);
//                     vector<Eigen::Vector3d> start_points;
                    
//                     // 0度
//                     start_points.emplace_back(Eigen::Vector3d(position.x(), position.y(), 0));

//                     // 5度
//                     start_points.emplace_back(Eigen::Vector3d(position.x(), position.y(), 5/57.3));

//                     // -5度
//                     start_points.emplace_back(Eigen::Vector3d(position.x(), position.y(), -5/57.3));

//                     // 10度
//                     start_points.emplace_back(Eigen::Vector3d(position.x(), position.y(), 10/57.3));

//                     // -10度
//                     start_points.emplace_back(Eigen::Vector3d(position.x(), position.y(), -10/57.3));

//                     for (auto & tmp_start_point : start_points)
//                     {
//                         // 两种情况均成立，才以此点为终点
//                         if (/* condition */)
//                         {
//                             start_point = tmp_start_point;
//                             start_flag = true;
//                             break; 
//                         }
//                     }
//                     if (start_flag)
//                     {
//                         break;
//                     }
//                     ++iterator;
//                 }

//                 // 找到合适的终点
//                 // 先从前进方向上找到5个候选终点区域，在从这5个区域中选择一个终点
//                 vector<Eigen::Vector3d> goal_points_final;
//                 vector<Eigen::Vector2d> goal_points;
//                 goal_points.emplace_back(Eigen::Vector2d(4.6, 0));
//                 goal_points.emplace_back(Eigen::Vector2d(3.8, 0));
//                 goal_points.emplace_back(Eigen::Vector2d(4.4, 2));
//                 goal_points.emplace_back(Eigen::Vector2d(4.4, -2));
//                 goal_points.emplace_back(Eigen::Vector2d(3.4, 2));
//                 goal_points.emplace_back(Eigen::Vector2d(3.4, -2));
//                 for (auto & tmp_goal_point : goal_points)
//                 {
//                     bool goal_flag = false;
//                     // Eigen::Vector3d goal;
//                     grid_map::SpiralIterator iterator(map, tmp_goal_point, radius);
//                     while (!iterator->isPastEnd())
//                     {
//                         grid_map::Position position;
//                         map.getPosition(*iterator, position);
//                         vector<Eigen::Vector3d>goal_points;
                        
//                         // 0度
//                         goal_points.emplace_back(Eigen::Vector3d(position.x(), position.y(), 0));

//                         // 5度
//                         goal_points.emplace_back(Eigen::Vector3d(position.x(), position.y(), 5/57.3));

//                         // -5度
//                         goal_points.emplace_back(Eigen::Vector3d(position.x(), position.y(), -5/57.3));

//                         // 10度
//                         goal_points.emplace_back(Eigen::Vector3d(position.x(), position.y(), 10/57.3));

//                         // -10度
//                         goal_points.emplace_back(Eigen::Vector3d(position.x(), position.y(), -10/57.3));

//                         for (auto & goal_point : goal_points)
//                         {

//                             if (/* condition */)
//                             {
//                                 goal_points_final.emplace_back(goal_point);
//                                 goal_flag = true;
//                                 break; 
//                             }
//                         }
//                         if (goal_flag)
//                         {
//                             break;
//                         }
//                         ++iterator;
//                     }
//                 }

//                 // 规划
            
//             }
//         }
// #endif
//     }




variousTerrainPlanner::~variousTerrainPlanner()
{
}
