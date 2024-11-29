#pragma once
#include <grid_map_core/GridMap.hpp>
#include <peac/PEAC_plane_detection.hpp>
#include <Eigen/Core>
#include <AstarHierarchicalFootstepPlanner/AstarHierarchicalFootstepPlanner.h>
#include <diy_msgs/footSteps.h>
class localPlannerPropose
{
private:
    grid_map::GridMap map;
    plane_detection pd;
    Eigen::Vector3d start_left, start_right, goal;
    int support_flag; // 0 initial, 1, left foot, 2, right foot
    FootParam foot_param;
    double hip_width;
    diy_msgs::footSteps footstep_msg;
    AstarHierarchicalFootstepPlanner astar_planner;
    vector<Footstep> steps;
    string package_path;
    cv::Mat seg_image;
    vector<planeInfo>  planes_info;
    vector<cv::Mat> single_results;
    vector<planeInfo>  merge_planes;
    vector<cv::Mat> merge_results;
public:
    localPlannerPropose();
    void setFootParam(FootParam & foot_param_);
    void setHipWidth(double hip_width_);
    void initial(Eigen::Vector3d start_left_, Eigen::Vector3d start_right_, int support_flag_, Eigen::Vector3d goal_);
    void mapPrepare(grid_map::GridMap & map_);
    pcl::PointCloud<pcl::PointXYZ> gridMap2PointcloudOrganized(grid_map::GridMap & map);
    void Inpaint(int radius);
    bool isGoalFeasible(Eigen::Vector3d goal);
    void detectionPlane();
    void mergePlanes();
    // void constructFeasibleMap();
    void plan();
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
    ~localPlannerPropose();
};


