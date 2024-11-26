#include <iostream>
#include <grid_map_core/GridMap.hpp>
#include <grid_map_ros/grid_map_ros.hpp>
#include <grid_map_msgs/GetGridMap.h>
#include <grid_map_core/iterators/SubmapIterator.hpp>
#include <grid_map_core/iterators/CircleIterator.hpp>
#include <grid_map_core/iterators/SpiralIterator.hpp>
#include <thread> // 必须包含此头文件
#include <chrono> // 用于时间单位
#include <opencv2/opencv.hpp>
#include <sensor_msgs/Image.h>
#include <random>
#include <local_planner.h>
using namespace std;

// #define GENERAL //yiban taijie dixing
#define WAVE // polang dixing

int main(int argc, char** argv)
{
    // int width = 800, height = 600;
    // cv::Mat image = cv::Mat::zeros(height, width, CV_8UC3);

    // // 波浪线的参数
    // int amplitude = 30;  // 振幅（波的高度）
    // int frequency = 100;  // 波浪的频率（每个周期内的点数）
    // int offsetYStart = 100; // 第一条波浪线的垂直起始位置
    // int waveGap = 50;      // 每条波浪线之间的垂直间隔
    // int thickness = 5;     // 波浪线的厚度
    // int numWaves = 5;      // 总共绘制的波浪线条数
    // // 循环绘制多条波浪线
    // for (int waveIndex = 0; waveIndex < numWaves; ++waveIndex) {
    //     int offsetY = offsetYStart + waveIndex * waveGap;
    //     // 绘制单条波浪线
    //     for (int x = 0; x < width; ++x) {
    //         int y = static_cast<int>(offsetY + amplitude * std::sin(2 * CV_PI * x / frequency));
    //         if (x > 0) {
    //             // 连接相邻的点，生成连续的波浪线
    //             cv::line(image, 
    //                      cv::Point(x - 1, static_cast<int>(offsetY + amplitude * std::sin(2 * CV_PI * (x - 1) / frequency))),
    //                      cv::Point(x, y), 
    //                      cv::Scalar(255, 0, 0), // 蓝色波浪线
    //                      thickness);           // 波浪线的厚度
    //         }
    //     }
    // }
    // // 显示图像
    // cv::imshow("Wave Lines", image);
    // cv::waitKey(0);
    // return 0;

    ros::init(argc, argv, "various_terrain");
    ros::NodeHandle nh;
    ros::Publisher map_pub = nh.advertise<grid_map_msgs::GridMap>("map", 1, true);
    ros::Publisher image_pub = nh.advertise<sensor_msgs::Image>("image", 1);  
    double map_length = 5.0;
    double map_width = 5.0;
    double resolution = 0.02;
    // 输入多变的地形,台阶宽度0.06-0.27，间隙0.02-0.17

    double step_elevation = 0.2;
    double gap_elevation = 0.0;
    grid_map::GridMap map({"elevation"});
    map.setFrameId("map");
    map.setGeometry(grid_map::Length(map_length, map_width), resolution, grid_map::Position(map_length/2.0, 0));

    while (ros::ok())
    {
#ifdef GENERAL
        for (int i = 0; i <=9; i++)
        {
            for (int j = 0; j <= 8; j++)
            {
                double step_width = 0.06 +resolution*i;
                double gap_width = 0.02 + resolution*j;
                int step_index_length = step_width/resolution;
                int gap_index_length = gap_width/resolution;
                map.clearAll();
                cout<<step_width<<" "<<gap_width<<endl;

                int index_length = 0;
                while (index_length < map.getSize().x())
                {
                    for (int i_x = 0; i_x < step_index_length; i_x++)
                    {
                        for(int j_y = 0; j_y < map.getSize().y(); j_y++)
                        {
                            if (i_x + index_length < map.getSize().x())
                            {
                                map["elevation"](i_x + index_length, j_y) = step_elevation;
                            }
                        }
                    }
                    index_length = index_length + step_index_length;
                    if (index_length >= map.getSize().x())
                    {
                        break;
                    }
                    for (int i_x = 0; i_x < gap_index_length; i_x++)
                    {
                        for(int j_y = 0; j_y < map.getSize().y(); j_y++)
                        {
                            if (i_x + index_length < map.getSize().x())
                            {
                                map["elevation"](i_x + index_length, j_y) = gap_elevation;
                            }
                        }
                    }
                    index_length = index_length + gap_index_length;
                }

                // double insert_length = 0.0;
                // while (insert_length < map_length)
                // {
                //     grid_map::Index submap_topleft_index;
                //     map.getIndex(grid_map::Position(insert_length + step_width, map_width/2.0), submap_topleft_index);
                //     grid_map::Index submapBufferSize(ceil(step_width/resolution), ceil(map_width/resolution));
                //     for (grid_map::SubmapIterator iterator(map, submap_topleft_index, submapBufferSize); !iterator.isPastEnd(); ++iterator)
                //     {
                //         grid_map::Index index(*iterator);
                //         map["elevation"](index.x(), index.y()) = step_elevation;
                //     }
                //     insert_length = insert_length + step_width;
                //     if (insert_length >= map_length)
                //     {
                //         break;
                //     }
                //     map.getIndex(grid_map::Position(insert_length + gap_width, map_width/2.0), submap_topleft_index);
                //     submapBufferSize = grid_map::Index(ceil(gap_width/resolution), ceil(map_width/resolution));
                //     for (grid_map::SubmapIterator iterator(map, submap_topleft_index, submapBufferSize); !iterator.isPastEnd(); ++iterator)
                //     {
                //         grid_map::Index index(*iterator);
                //         map["elevation"](index.x(), index.y()) = gap_elevation;
                //     }
                //     insert_length = insert_length + gap_width;
                // }

                // genju ditu xuanze start and goal

                grid_map_msgs::GridMap map_msg;
                grid_map::GridMapRosConverter::toMessage(map, map_msg);
                map_pub.publish(map_msg);
                std::this_thread::sleep_for(std::chrono::seconds(2));
                cout<<"out"<<endl;
            }
            
        }
#endif

#ifdef WAVE
        cv::Mat image = cv::Mat::zeros(map.getSize().x(), map.getSize().y(), CV_8UC1);
        // 通过调节lineGap 和 phaseShift 来实现场景的变化
        // 波浪线的参数
        int amplitude = 20;  // 振幅（波的高度）
        int frequency = 100;  // 波浪的频率（每个周期内的点数）

        for (int i = 0; i <=9; i++)
        {
            for (int j = 0; j <= 8; j++)
            {
                image = cv::Mat::zeros(map.getSize().x(), map.getSize().y(), CV_8UC1);
                map.clearAll();
                double step_width = 0.06 +resolution*(i+2);
                double gap_width = 0.02 + resolution*(j);
                int step_index_length = step_width/resolution;
                int gap_index_length = gap_width/resolution;

                int lineGap = step_index_length + gap_index_length;      // 每条波浪线之间的垂直间隔
                int thickness = step_index_length;     // 波浪线的厚度
                // int phaseShift = (int)(frequency/(2*(image.rows/lineGap)));

                // 循环绘制波浪线，覆盖整个图像
                for (int i = 0, offsetY = 0; offsetY < map.getSize().x(); ++i, offsetY += lineGap) 
                {
                    // int xOffset = i * phaseShift; // 计算当前波浪线的横向偏移量

                    std::random_device rd; // 随机数种子
                    std::mt19937 gen(rd()); // 随机数生成器

                    // 整数范围
                    std::uniform_int_distribution<> dist_int(0, frequency/2); // 1到10
                    int xOffset = dist_int(gen);

                    for (int x = 0; x < map.getSize().y(); ++x) 
                    {
                        // 计算波浪线在 y 方向上的位置，加入横向偏移量
                        int y = static_cast<int>(offsetY + amplitude * std::sin(2 * CV_PI * (x + xOffset) / frequency));
                        if (x > 0) 
                        {
                            // 连接相邻的点，生成连续的波浪线
                            cv::line(image, 
                                    cv::Point(x - 1, static_cast<int>(offsetY + amplitude * std::sin(2 * CV_PI * ((x - 1) + xOffset) / frequency))),
                                    cv::Point(x, y), 
                                    cv::Scalar(255), // 白色波浪线
                                    thickness);               // 波浪线的厚度
                        }
                    }
                }
                sensor_msgs::ImagePtr image_msg = cv_bridge::CvImage(std_msgs::Header(), "mono8", image).toImageMsg();
                image_pub.publish(image_msg);
                for (int x_i = 0; x_i < image.rows; x_i++)
                {
                    for (int y_j = 0; y_j < image.cols; y_j++)
                    {
                        if (image.at<uchar>(x_i, y_j) == 255)
                        {
                            map["elevation"](x_i, y_j) = step_elevation;
                        }
                        else
                        {
                            map["elevation"](x_i, y_j) = gap_elevation;
                        }
                    }
                    
                }
                grid_map_msgs::GridMap map_msg;
                grid_map::GridMapRosConverter::toMessage(map, map_msg);
                map_pub.publish(map_msg);
                std::this_thread::sleep_for(std::chrono::seconds(2));

                localPlanner local_planner
                local_planner.setFootParam(foot_param);
                nh.param("hip_width", hip_width, 0.05);
                local_planner.setHipWidth(hip_width);
                local_planner.mapPrepare(map);

                // 找起始点
                bool start_flag = false;
                Eigen::Vector3d start_point;
                // 在初始圆内找起始点
                grid_map::Position center(0.15, 0);
                double radius = 0.1;
                grid_map::SpiralIterator iterator(map, center, radius);
                while (!iterator->isPastEnd())
                {
                    grid_map::Position position;
                    map.getPosition(*iterator, position);
                    vector<Eigen::Vector3d> start_points;
                    
                    // 0度
                    start_points.emplace_back(Eigen::Vector3d(position.x(), position.y(), 0));

                    // 5度
                    start_points.emplace_back(Eigen::Vector3d(position.x(), position.y(), 5/57.3));

                    // -5度
                    start_points.emplace_back(Eigen::Vector3d(position.x(), position.y(), -5/57.3));

                    // 10度
                    start_points.emplace_back(Eigen::Vector3d(position.x(), position.y(), 10/57.3));

                    // -10度
                    start_points.emplace_back(Eigen::Vector3d(position.x(), position.y(), -10/57.3));

                    for (auto & tmp_start_point : start_points)
                    {

                        if (/* condition */)
                        {
                            start_point = tmp_start_point;
                            start_flag = true;
                            break; 
                        }
                    }
                    if (start_flag)
                    {
                        break;
                    }
                    ++iterator;
                }

                // 找到合适的终点
                // 先从前进方向上找到5个候选终点区域，在从这5个区域中选择一个终点
                vector<Eigen::Vector3d> goal_points_final;
                vector<Eigen::Vector2d> goal_points;
                goal_points.emplace_back(Eigen::Vector2d(4.6, 0));
                goal_points.emplace_back(Eigen::Vector2d(3.8, 0));
                goal_points.emplace_back(Eigen::Vector2d(4.4, 2));
                goal_points.emplace_back(Eigen::Vector2d(4.4, -2));
                goal_points.emplace_back(Eigen::Vector2d(3.4, 2));
                goal_points.emplace_back(Eigen::Vector2d(3.4, -2));
                for (auto & tmp_goal_point : goal_points)
                {
                    bool goal_flag = false;
                    // Eigen::Vector3d goal;
                    grid_map::SpiralIterator iterator(map, tmp_goal_point, radius);
                    while (!iterator->isPastEnd())
                    {
                        grid_map::Position position;
                        map.getPosition(*iterator, position);
                        vector<Eigen::Vector3d>goal_points;
                        
                        // 0度
                        goal_points.emplace_back(Eigen::Vector3d(position.x(), position.y(), 0));

                        // 5度
                        goal_points.emplace_back(Eigen::Vector3d(position.x(), position.y(), 5/57.3));

                        // -5度
                        goal_points.emplace_back(Eigen::Vector3d(position.x(), position.y(), -5/57.3));

                        // 10度
                        goal_points.emplace_back(Eigen::Vector3d(position.x(), position.y(), 10/57.3));

                        // -10度
                        goal_points.emplace_back(Eigen::Vector3d(position.x(), position.y(), -10/57.3));

                        for (auto & goal_point : goal_points)
                        {

                            if (/* condition */)
                            {
                                goal_points_final.emplace_back(goal_point);
                                goal_flag = true;
                                break; 
                            }
                        }
                        if (goal_flag)
                        {
                            break;
                        }
                        ++iterator;
                    }
                }

                // 规划
            
            }
        }
#endif
    }

    
    
    return 0;

}