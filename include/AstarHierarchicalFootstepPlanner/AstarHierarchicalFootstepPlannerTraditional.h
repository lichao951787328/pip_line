#pragma once
#include <AstarHierarchicalFootstepPlanner/AstarHierarchicalFootstepPlannerBase.h>
using namespace std;

// #define DEBUG
class AstarHierarchicalFootstepPlannerTraditional:public AstarHierarchicalFootstepPlannerBase
{
protected:
    double checkXupper, checkXButton;
public:

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


    void setCheckParam(double checkXupper_, double checkXButton_);

    ~AstarHierarchicalFootstepPlannerTraditional();
};


