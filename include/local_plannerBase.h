/*
 * @Author: lichao951787328 951787328@qq.com
 * @Date: 2025-02-13 10:10:00
 * @LastEditors: lichao951787328 951787328@qq.com
 * @LastEditTime: 2025-02-13 14:50:10
 * @FilePath: /pip_line/include/local_plannerBase.h
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
#pragma once
#include <grid_map_core/GridMap.hpp>
#include <peac/PEAC_plane_detection.hpp>
#include <Eigen/Core>
#include <AstarHierarchicalFootstepPlanner/AstarHierarchicalFootstepPlannerBase.h>
#include <diy_msgs/footSteps.h>
class localPlannerBase
{
private:
    grid_map::GridMap map;
    plane_detection pd;
    Eigen::Vector3d start, pre_start, goal;
    int support_flag; // 0 左脚支撑, 1, 右脚支撑, 2, 双脚支撑
    FootParam foot_param;
    double hip_width;
    diy_msgs::footSteps footstep_msg;
    // 这个要根据你想要选择的算法来决定选择哪种规划方式
    std::shared_ptr<AstarHierarchicalFootstepPlannerBase> planner_P = nullptr;
    // AstarHierarchicalFootstepPlanner astar_planner;
    vector<Footstep> steps;
    string package_path;
    cv::Mat seg_image;
    vector<planeInfo>  planes_info;
    vector<cv::Mat> single_results;
    vector<planeInfo>  merge_planes;
    vector<cv::Mat> merge_results;
    
public:
    string time_consume;
    localPlannerBase();
    localPlannerBase(std::shared_ptr<AstarHierarchicalFootstepPlannerBase> a);
    void setPlanner(std::shared_ptr<AstarHierarchicalFootstepPlannerBase> a);
    void setFootParam(FootParam & foot_param_);
    void setHipWidth(double hip_width_);
    void initial(Eigen::Vector3d start_, Eigen::Vector3d pre_start_, int support_flag_, Eigen::Vector3d goal_);
    bool isStartFeasible(Eigen::Vector3d start, Eigen::Vector3d & left_foot, Eigen::Vector3d & right_foot);
    void mapPrepare(grid_map::GridMap & map_);
    pcl::PointCloud<pcl::PointXYZ> gridMap2PointcloudOrganized();
    void Inpaint(int radius);
    bool isGoalFeasible(Eigen::Vector3d goal);
    void detectionPlane();
    void mergePlanes();
    // void constructFeasibleMap();
    bool plan();

    // inline void setTimeConsumption(string s)
    // {
    //     time_consume = s;
    // }
    // inline string getTimeConsumption()
    // {
    //     // LOG(INFO)<<planner_P->time_consume;
    //     return time_consume;
    // }
    inline diy_msgs::footSteps getResultSteps()
    {
        for (auto & footstep : steps)
        {
            diy_msgs::footStep step;
            step.is_left = footstep.robot_side == LEFT;
            step.x = footstep.x;
            step.y = footstep.y;
            step.z = footstep.z;
            step.roll = footstep.roll;
            step.pitch = footstep.pitch;
            step.yaw = footstep.yaw;
            footstep_msg.footsteps.emplace_back(step);
        }
        return footstep_msg;
    }
    inline vector<Footstep> getFootsteps()
    {
        return steps;
    }
    inline std::shared_ptr<AstarHierarchicalFootstepPlannerBase> getPlannerPtr()
    {
        return planner_P;
    }
    ~localPlannerBase();
};


