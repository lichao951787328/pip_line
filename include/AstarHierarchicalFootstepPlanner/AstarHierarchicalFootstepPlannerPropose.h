/*
 * @Author: lichao951787328 951787328@qq.com
 * @Date: 2025-02-13 10:10:00
 * @LastEditors: lichao951787328 951787328@qq.com
 * @LastEditTime: 2025-02-13 15:44:20
 * @FilePath: /pip_line/include/AstarHierarchicalFootstepPlanner/AstarHierarchicalFootstepPlannerPropose.h
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
#pragma once
#include <AstarHierarchicalFootstepPlanner/AstarHierarchicalFootstepPlannerBase.h>
using namespace std;

// #define DEBUG
class AstarHierarchicalFootstepPlannerPropose:public AstarHierarchicalFootstepPlannerBase
{
public:
    /**
     * @brief 判断某点能否作为起始点
     * 
     * @param start z为角度
     * @param left_foot 返回左脚
     * @param right_foot 返回右脚
     * @return true 
     * @return false 
     */
    // bool isStartFeasible(Eigen::Vector3d start, Eigen::Vector3d & left_foot, Eigen::Vector3d & right_foot) override;

    // 使用传统方法来确定初始左右脚
    // bool isStartFeasibleTradition(Eigen::Vector3d start, Eigen::Vector3d & left_foot, Eigen::Vector3d & right_foot);

    /**
     * @brief 节点扩展
     * 
     * @param current_node 当前节点
     * @param pre_node 前一个节点
     * @param child_nodes 扩展得到的节点
     * @return true 
     * @return false 
     */
    // bool nodeExtension(FootstepNodePtr current_node, FootstepNodePtr pre_node, vector<FootstepNodePtr> & child_nodes) override;

    /**
     * @brief 根据localmap计算在ankle处，支撑点数、超出点数、平面法向量，这是根据落脚点的支撑平方面计算方式来区别进行的，需要为纯虚函数
     * 
     * @param ankle 
     * @param max_size 
     * @param above_points 
     * @param plane_normal 
     * @return true 
     * @return false 
     */
    bool computeLandInfo(Eigen::Vector3d ankle, int & max_size, int & above_points, Eigen::Vector3d & plane_normal, double & step_height, double & pitch, double & roll, int & plane_indx) override;

    
    // 根据终点解算机器人在终点时左右脚的位置
    /**
     * @brief 计算终点处左右脚的站立位置
     * 
     * @param goal 
     * @return true 
     * @return false 
     */
    // bool computerLeftRightGoal(Eigen::Vector3d goal) override;

    // bool checkFeasibleGoal(Eigen::Vector3d goal) override;

    bool getPointsInFootArea(Eigen::Vector3d ankle, std::unordered_map<int, int> & fore_left_foot_counter, std::unordered_map<int, int> & fore_right_foot_counter, std::unordered_map<int, int> & hind_left_foot_counter, std::unordered_map<int, int> & hind_right_foot_counter);

    bool getSupportPlaneIndex(std::unordered_map<int, int> & counter, Eigen::Vector3d & mid, int thred, int & plane_index, int & support_size);

    bool getSquareCounter(Eigen::Vector2d TL, Eigen::Vector2d TR, Eigen::Vector2d BL, Eigen::Vector2d BR, std::unordered_map<int, int> & counter);

    ~AstarHierarchicalFootstepPlannerPropose();
};


