#pragma once
#include <ros/ros.h>
// #include <local_plannerBase.h>
#include <grid_map_core/GridMap.hpp>
#include <future>
class variousTerrainPlanner
{
private:
    ros::NodeHandle nh;
    ros::Publisher map_pub;
    // FootParam foot_param;
    double hip_width, checkXupper, checkXButton;
    // localPlannerBase local_planner_traditional;
    // localPlannerBase local_planner_propose;
public:
    variousTerrainPlanner(ros::NodeHandle nh_);
    // bool CheckFeasibleGoal(grid_map::GridMap & map, int radius, vector<Eigen::Vector3d> & goal_points_final);
    // bool checkFeasibleStart(grid_map::GridMap & map, Eigen::Vector3d & left_foot_tra, Eigen::Vector3d & left_right_tra, Eigen::Vector3d & propose_left_foot, Eigen::Vector3d & propose_right_foot);
    // void traditionalPlanner();
    // void proposePlanner();
    // void executeTaskWithTimeout(std::function<void()> task, int timeout);
    void execute();
    ~variousTerrainPlanner();
};