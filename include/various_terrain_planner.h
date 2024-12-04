#include <ros/ros.h>
#include <local_plannerBase.h>
#include <fstream>
class variousTerrainPlanner
{
private:
    ros::NodeHandle nh;
    ros::Publisher map_pub;
    FootParam foot_param;
    double hip_width, checkXupper, checkXButton;
    localPlannerBase local_planner_traditional;
    localPlannerBase local_planner_propose;
    std::string filename = "output.txt";
    std::ofstream file;
public:
    variousTerrainPlanner(ros::NodeHandle nh_);

    bool CheckFeasibleGoalTraditional(grid_map::GridMap & map, Eigen::Vector2d cand_goal, Eigen::Vector3d & goal);
    bool CheckFeasibleGoalPropose(grid_map::GridMap & map, Eigen::Vector2d cand_goal, Eigen::Vector3d & goal);

    bool CheckFeasibleGoalTraditional(grid_map::GridMap & map, vector<Eigen::Vector3d> & goal_points_final);
    bool CheckFeasibleGoalPropose(grid_map::GridMap & map, vector<Eigen::Vector3d> & goal_points_final);

    bool checkFeasibleStartTraditional(grid_map::GridMap & map, Eigen::Vector3d & left_foot_tra, Eigen::Vector3d & right_foot_tra);
    bool checkFeasibleStartPropose(grid_map::GridMap & map, Eigen::Vector3d & propose_left_foot, Eigen::Vector3d & propose_right_foot);

    bool getPlannerResultTraditional();
    bool getPlannerResultPropose();



    void traditionalPlanner();
    void proposePlanner();
    void execute();
    void executeTaskWithTimeout(std::function<void()> task, int timeout);
    ~variousTerrainPlanner();
};


