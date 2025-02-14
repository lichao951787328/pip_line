/*
 * @Author: lichao951787328 951787328@qq.com
 * @Date: 2025-02-10 22:18:43
 * @LastEditors: lichao951787328 951787328@qq.com
 * @LastEditTime: 2025-02-14 19:28:43
 * @FilePath: /pip_line/include/local_plan_node.h
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
#pragma once
#include <ros/ros.h>
#include <grid_map_msgs/GridMap.h>
#include <diy_msgs/robotState.h>
#include <local_plannerBase.h>
#include <nav_msgs/Path.h>
#include <diy_msgs/footSteps.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/LinearMath/Transform.h>
#include <mutex>
#include <list>
#include <atomic>
using namespace std;
// 如何保证map与state同步
class localPlanNode
{
private:
    ros::NodeHandle nh;
    string map_topic;
    ros::Subscriber sub_map;
    string robot_state_topic;
    ros::Subscriber sub_robot_state;
    std::mutex received_mutex;
    bool robot_state_received;
    list<diy_msgs::robotState> robot_states;
    // diy_msgs::robotState robot_state;  
    // 机器人参数
    FootParam foot_param;
    double hip_width;

    string global_path_topic;
    nav_msgs::Path global_path;
    ros::Subscriber sub_global_path;

    string foosteps_topic;
    diy_msgs::footSteps footsteps;
    ros::Publisher pub_footsteps_visual;
    ros::Publisher pub_footsteps;

    // localPlanner local_planner; 不使用此规划器，使用自己集成在local_plannerBase中的
    
    string global_terrain_map_frame;
    string local_map_frame;

    std::shared_ptr<tf2_ros::TransformListener> transform_listener_;
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
public:
    localPlanNode(ros::NodeHandle & n);
    void mapCallback(const grid_map_msgs::GridMap::ConstPtr& msg);
    void robotStateCallback(const diy_msgs::robotState::ConstPtr& msg);
    void globalPathCallback(const nav_msgs::Path::ConstPtr& msg);
    vector<tf2::Transform> transformPose(tf2::Transform transform);
    // bool getLocalGoalFromPath(tf2::Transform transform_localmap_globalmap, grid_map::GridMap & map, Eigen::Vector3d & goal_localmap);
    bool getRobotState(uint32_t map_id, diy_msgs::robotState & robot_state);

    void cancelPlanning(); 

    void publishFootsteps(diy_msgs::footSteps steps);
    ~localPlanNode();
};

