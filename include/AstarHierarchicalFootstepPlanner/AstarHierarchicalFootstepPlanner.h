#pragma once
#include <unordered_map>
#include <queue>
#include <grid_map_core/GridMap.hpp>
#include <vector>
#include <Eigen/Core>
#include <unordered_set>
#include <diy_msgs/footSteps.h>
#include <glog/logging.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <opencv2/opencv.hpp>
#include <ros/ros.h>
#include <fstream>
#include <glog/logging.h>
#include <peac/PEAC_plane_detection.hpp>
using namespace std;

// #define DEBUG
enum RobotSide{LEFT, RIGHT, _NAN_};

struct Footstep
{
    
    RobotSide robot_side;
    double x, y, z, roll, pitch, yaw;
    Footstep()
    {
        x = NAN;
        y = NAN;
        z = NAN;
        roll = NAN;
        pitch = NAN;
        yaw = NAN;
        robot_side = _NAN_;
    }
    Footstep(Eigen::Vector3d point, int robotside)// 0 left; 1, right
    {
        x = point.x();
        y = point.y();
        z = NAN;
        roll = NAN;
        pitch = NAN;
        yaw = point.z();
        if (robotside == 0)
        {
            robot_side = LEFT;
        }
        else
        {
            robot_side = RIGHT;
        }
    }

    Footstep(Eigen::Vector3d point, double height, double roll_, double pitch_, int robotside)
    {
        x = point.x();
        y = point.y();
        z = height;
        roll = roll_;
        pitch = pitch_;
        yaw = point.z();
        if (robotside == 0)
        {
            robot_side = LEFT;
        }
        else
        {
            robot_side = RIGHT;
        }
    }

    Footstep & operator=(const Footstep & other)
    {
        if (this == &other)
        {
            return *this;
        }
        this->x = other.x;
        this->y = other.y;
        this->z = other.z;
        this->roll = other.roll;
        this->pitch = other.pitch;
        this->yaw = other.yaw;
        this->robot_side = other.robot_side;
        return *this;
    }
    RobotSide getInverseRobotSide()
    {
        if (robot_side == LEFT)
        {
            return RIGHT;
        }
        else
        {
            return LEFT;
        }
    }
};

struct FootstepNode
{
    Footstep footstep;
    double cost, Hcost, Gcost;
    int plane_index;//脚属于哪一个平面，后续可以根据平面是否变化及xyz的位移信息进行筛选更适合机器人的步态点
    std::shared_ptr<FootstepNode> PreFootstepNode = nullptr;
    FootstepNode():footstep()
    {
        cost = NAN;
        Hcost = NAN;
        Gcost = NAN;
        PreFootstepNode = nullptr;
    }
    FootstepNode(Eigen::Vector3d point, int robotside):footstep(point, robotside)
    {
        cost = NAN;
        Hcost = NAN;
        Gcost = NAN;
        PreFootstepNode = nullptr;
    }

    FootstepNode(Eigen::Vector3d point, double height, double roll, double pitch, int robotside):footstep(point, height, roll, pitch, robotside)
    {
        cost = NAN;
        Hcost = NAN;
        Gcost = NAN;
        PreFootstepNode = nullptr;
    }
    FootstepNode & operator=(const FootstepNode & other)
    {
        if (this == &other)
        {
            return *this;
        }
        this->footstep = other.footstep;
        this->PreFootstepNode = other.PreFootstepNode;
        cost = other.cost;
        Hcost = other.Hcost;
        Gcost = other.Gcost;
        return *this;
    }
};

typedef std::shared_ptr<FootstepNode> FootstepNodePtr;
// 小顶堆
struct FootstepNodeCompare
{
    bool operator()(const FootstepNodePtr n1, const FootstepNodePtr n2)
    {
        if (std::isnan(n1->cost) || std::isnan(n2->cost))
        {
            std::cout<<"the cost is nan"<<std::endl;
        }
        return n1->cost > n2->cost;
    }
};

struct FootParam
{
    double x_upper, x_button, y_left, y_right, x_fore_button, x_hind_top;
    FootParam()
    {

    }
    FootParam(double x_u, double x_b, double y_l, double y_r, double x_f_b, double x_h_t)
    {
        CHECK(x_u > 0 && x_b > 0 && y_l > 0 && y_r > 0);
        CHECK(x_f_b >= 0 && x_h_t >= 0);
        x_upper = x_u;
        x_button = x_b;
        y_left = y_l;
        y_right = y_r;
        x_fore_button = x_f_b;
        x_hind_top = x_h_t;
    }
    FootParam & operator=(const FootParam & other)
    {
        if (this == &other)
        {
            return *this;
        }
        this->x_button = other.x_button;
        this->x_upper = other.x_upper;
        this->y_left = other.y_left;
        this->y_right = other.y_right;
        this->x_fore_button = other.x_fore_button;
        this->x_hind_top = other.x_hind_top;
        return *this;
    }
};

struct ScoreMarkerNode
{
    Eigen::Vector3d point; //  相对于支撑脚的平移量， x y yaw
    Eigen::Vector3d normal;
    double score;
    double height;
    int plane_index;
    double roll, pitch;
    ScoreMarkerNode(Eigen::Vector3d point_, double score_)
    {
        point = point_;
        score = score_;
    }
    ScoreMarkerNode(Eigen::Vector3d point_, double score_, double height_, Eigen::Vector3d normal_, int plane_index_, double roll_, double pitch_)
    {
        point = point_;
        score = score_;
        height = height_;
        normal = normal_;
        plane_index = plane_index_;
        roll = roll_;
        pitch = pitch_;
    }
};
typedef std::shared_ptr<ScoreMarkerNode> ScoreMarkerNodePtr;
// 分数大的在前面
struct ScoreMarkerNodeCompare
{
    bool operator()(const ScoreMarkerNodePtr n1, const ScoreMarkerNodePtr n2)
    {
        if (std::isnan(n1->score) || std::isnan(n2->score))
        {
            std::cout<<"the cost is nan"<<std::endl;
        }
        return n1->score < n2->score;
    }
};

// 直方图投票
struct HistogramVoting
{
    std::unordered_map<int, int> counter;
    int nan_points = 0;
    int all_points = 0;
    void add(int index, Eigen::Vector3d & p)
    {
        if (std::isnan(index))
        {
            nan_points++;
            return;
        }
        else
        {
            counter[index]++;
        }
        all_points++;
    }

    void addNANPoints()
    {
        nan_points++;
    }
};

struct IndexPlanePoints
{
    std::unordered_map<int, vector<Eigen::Vector3d>> counter;
    int max_index = -1;  // 计数最多的索引值
    int max_count = 0;   // 计数最多的次数
    int nan_points = 0;
    void addNANPoints()
    {
        nan_points++;
    }
    void add(int index, Eigen::Vector3d p) 
    {
        if (std::isnan(index))
        {
            nan_points++;
            return;
        }
        else
        {
            counter[index].emplace_back(p);
            if (counter[index].size() > max_count) 
            {
                max_count = counter[index].size();
                max_index = index;
            }
        }
    }

    // 获取计数最多的索引值
    int getMaxIndex() const 
    {
        return max_index;
    }

    vector<Eigen::Vector3d> getNoMaxPoints()
    {
        int max_index = getMaxIndex();
        vector<Eigen::Vector3d> non_max_points;
        for (auto & count : counter)
        {
            if (count.first != max_index)
            {
                non_max_points.insert(non_max_points.end(), count.second.begin(), count.second.end());
            }
        }
        return non_max_points;
    }

    int getMaxPointsSize()
    {
        return max_count;
    }
};

/**
 * @brief 分层A* 落脚点规划器
 * @param localmap 局部地图
 * @param label_localmap 带标签的局部地图，标签即为平面标签
 * @param label_index 带标签的矩阵，与上一个参数作用一样
 * @param mapsize 地图的size，即共有多山个栅格
 * @param footsize_inmap 脚能占据的栅格数
 * @param hip_width 髋宽
 * @param planes 地图中包含的平面数量
 * @param resolution 分辨率
 * @param footparam 脚的参数
 * @param start_p 规划起点脚步
 * @param prestart_p 规划起点的前一步
 * @param end_p 终点，这里不能是脚步
 * @param end_left_p 终点左脚，根据上一个参数计算出来的
 * @param end_right_p 终点右脚，根据上一个参数计算出来的
 * @param p_queue 队列中的节点
 * @param steps 规划结果，不算起点的落脚点
 * @param transitions nodeExpantion时的基础节点
 * @param plane_images 提取的平面，不切割掉碰撞区域
 * @param plane_image 在这个平面图上画落脚点，用于检查。跟label_local对应的平面
 */
class AstarHierarchicalFootstepPlanner
{
private:
    grid_map::GridMap localmap;
    grid_map::GridMap label_localmap;
    Eigen::MatrixXi label_index;
    // Eigen::Matrix2i label_index;// 如果用label_localmap不能索引，就只能用这个了
    int mapsize;
    int footsize_inmap;
    double hip_width;
    int planes;
    double resolution;
    FootParam footparam;
    FootstepNodePtr start_p = nullptr;
    FootstepNodePtr prestart_p = nullptr;
    FootstepNodePtr end_p;
    FootstepNodePtr end_left_p;
    FootstepNodePtr end_right_p;
    std::priority_queue<FootstepNodePtr, std::vector<FootstepNodePtr>, FootstepNodeCompare> p_queue;
    std::unordered_set<std::string> close_set;
    vector<Footstep> steps;
    // 初始化参数 支撑脚为右脚，扩展参数为左脚
    vector<Eigen::Vector3d> transitions;
    vector<Eigen::Vector3d> combine_transitions;
    

    // 后续计算障碍点时会用到
    cv::Mat plane_image;
    unsigned char default_colors[12][3] =
			{
				{255, 0, 0},
				{255, 255, 0},
				{100, 20, 50},
				{0, 30, 255},
				{10, 255, 60},
				{80, 10, 100},
				{0, 255, 200},
				{10, 60, 60},
				{255, 0, 128},
				{60, 128, 128},
                {10, 255, 60},
                {100, 20, 50}
			};
    // 这是灰度图
    vector<cv::Mat> plane_images;
    vector<planeInfo> planes_info;
    // for debug
    std::ofstream outfile;
    
    // 论文中用到的
    int checktime = 0;
    double total_time = 0.;

public:
    /**
     * @brief Construct a new Astar Hierarchical Footstep Planner object
     * 
     * @param lm 局部地图
     * @param footparam_ 脚的参数
     * @param hip_width_ 髋宽 
     */
    AstarHierarchicalFootstepPlanner(grid_map::GridMap & lm, FootParam footparam_, double hip_width_);

    /**
     * @brief Construct a new Astar Hierarchical Footstep Planner object. 使用已经切割好平面区域的进行构建
     * 
     * @param label_map 带标号的栅格地图
     * @param plane_iamage 多个平面表达在一个图里
     * @param planes_image 分多个图表达
     * @param footparam_ 脚参数
     * @param hip_width_ 髋宽
     */
    AstarHierarchicalFootstepPlanner(grid_map::GridMap & label_map, cv::Mat & plane_iamage_, vector<cv::Mat> & planes_image_, vector<planeInfo> & planes_info_, FootParam footparam_, double hip_width_);

    void initial_transitions();

    // 使用起始状态来初始化机器人起点及终点
    /**
     * @brief 根据起点，终点进行初始化
     * 
     * @param start 起点步态
     * @param prestart 起点前一步的步态
     * @param support_side 当前支撑脚
     * @param goal 终点
     * @return true 
     * @return false 
     */
    bool initial(Eigen::Vector3d start, Eigen::Vector3d prestart, int support_side, Eigen::Vector3d goal);

    /**
     * @brief 节点扩展
     * 
     * @param current_node 当前节点
     * @param pre_node 前一个节点
     * @param child_nodes 扩展得到的节点
     * @return true 
     * @return false 
     */
    bool nodeExtension(FootstepNodePtr current_node, FootstepNodePtr pre_node, vector<FootstepNodePtr> & child_nodes);

    /**
     * @brief 精修节点。对于某些节点不能很好的满足，我们对其进行精修，即找到其附近的一些节点，获取附近节点的参数。
     * 
     * @param parent_transition 待修节点
     * @return vector<Eigen::Vector3d> 候选节点参数
     */
    vector<Eigen::Vector3d> fineTransitionsBasic(Eigen::Vector3d parent_transition);

    /**
     * @brief Get the Point Height In Plane object
     * 
     * @param p 
     * @param height 
     * @return true 
     * @return false 
     */
    bool getPointInfoInPlane(Eigen::Vector3d p, double & height, int & plane_index, double & pitch, double & roll);

    /**
     * @brief 对落脚点进行精修，根据goal得到的左右脚可能有些缺陷，通过精修获得更合适的终点左右脚的姿态。这里只是为了获得候选节点
     * 
     * @param land_point 待修节点
     * @return vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>> 候选节点
     */
    vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>> fineLandPoint(Eigen::Vector3d land_point);
    // vector<Eigen::Vector3d> basicTransitions(FootstepNodePtr current_node);
    // 返回的是基扩展节点和完整扩展节点
    /**
     * @brief 这个不是在精修阶段使用的，而是在一般阶段使用。通过基节点获取在基于某个步态时其节点在地图坐标系下的状态
     * 
     * @param current_node 
     * @return vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>> 候选节点
     */
    vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>> basicTransitions(FootstepNodePtr current_node);

    /**
     * @brief 结合fineTransitionsBasic函数，获取在已知当前步态的情况下，候选节点在地图坐标系下的坐标
     * 
     * @param point 精修节点
     * @param current_node 当前步态
     * @return vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>> 精修节点与在地图坐标系的pair
     */
    vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>> fineTransitions(Eigen::Vector3d point, FootstepNodePtr current_node);

    /**
     * @brief 根据点，计算此处的z值，及落脚时的roll和pitch值
     * 
     * @param point 点
     * @param z 
     * @param roll 
     * @param pitch 
     * @return true 
     * @return false 
     */
    bool computeZRollPitch(Eigen::Vector3d point, double & z, Eigen::Vector3d & eular, int & plane_index);

    /**
     * @brief 计算localmap对应的含平面编号的平面信息
     * 
     */
    void computePlanarInfor();

    /**
     * @brief 计算在transition的高度值transition_height
     * 
     * @param transition 
     * @param transition_height 
     * @return true 
     * @return false 
     */
    bool computeTransitionHeight(Eigen::Vector3d transition, double & transition_height);

    // 计算某个transition的得分，使用基trans和转换后的trans
    /**
     * @brief 计算每个transition的得分，包括与当前节点的高度、摆动高度、落脚角度、与最佳位置的偏离程度、超出支撑面的点数、支撑面的点数
     * 
     * @param transition 
     * @param current_node 
     * @param pre_node 
     * @param score 
     * @return true 
     * @return false 
     */

    Eigen::Vector3d Quaterniond2EulerAngles(Eigen::Quaterniond q);
    Eigen::Vector3d Matrix3d2EulerAngles(Eigen::Matrix3d m);

    bool computeTransitionScore(std::pair<Eigen::Vector3d, Eigen::Vector3d> transition, FootstepNodePtr current_node, FootstepNodePtr pre_node, bool & dangerous, double & score, double & height, Eigen::Vector3d & plane_normal, int & plane_index, double & pitch, double & roll);

    /**
     * @brief 计算每个落脚点的得分，目的是为了找到终点处最佳的左右脚站立点
     * 
     * @param land_point 
     * @param score 
     * @return true 
     * @return false 
    //  */
    bool computeLandPointScore(std::pair<Eigen::Vector3d, Eigen::Vector3d> land_point, double & score, double & height, int & plane_index, double & pitch, double & roll);


    /**
     * @brief 在精修阶段，对超过支撑面高度的点要求很高。计算精修阶段的点的得分
     * 
     * @param transition 
     * @param current_node 
     * @param pre_node 
     * @param score 
     * @return true 
     * @return false 
     */
    bool computeTransitionStrictScore(std::pair<Eigen::Vector3d, Eigen::Vector3d> transition, FootstepNodePtr current_node, FootstepNodePtr pre_node, double & score, double & height, Eigen::Vector3d & plane_normal);

    /**
     * @brief 将地图转化为有序地图，供提取平面时使用
     * 
     * @return pcl::PointCloud<pcl::PointXYZ> 
     */
    pcl::PointCloud<pcl::PointXYZ> gridMap2Pointcloud();

    /**
     * @brief 将点转化为脚节点
     * 
     * @param p 点的第三个为yaw，不为z
     * @return FootstepNodePtr 
     */
    bool point2Node(Eigen::Vector3d p, FootstepNodePtr node);

    bool startPoint2Node(Eigen::Vector3d p, FootstepNodePtr node);

    bool getPointsInFootArea(Eigen::Vector3d ankle, HistogramVoting & fore_foot_HV, HistogramVoting & hind_foot_HV);

    bool getPointsInFootArea(Eigen::Vector3d ankle, IndexPlanePoints & index_plane);

    bool getPointsInForeFoot(Eigen::Vector3d ankle, HistogramVoting & fore_foot_HV);

    bool getPointsInHindFoot(Eigen::Vector3d ankle, HistogramVoting & hind_foot_HV);

    bool SqurePoints(Eigen::Vector2d TL, Eigen::Vector2d TR, Eigen::Vector2d BL, Eigen::Vector2d BR, vector<Eigen::Vector3d> & points);

    bool SqureHistogramVoting(Eigen::Vector2d TL, Eigen::Vector2d TR, Eigen::Vector2d BL, Eigen::Vector2d BR, HistogramVoting & HV);

    void getSquareHistogramVoting(Eigen::Vector2d TL, Eigen::Vector2d TR, Eigen::Vector2d BL, Eigen::Vector2d BR, HistogramVoting & HV);

    bool getLandAreaPoints(Eigen::Vector3d ankle, vector<Eigen::Vector3d> & points);




    /**
     * @brief 获取在ankle处落脚时，机器人包含几个平面点，及位于此区域所有的点集
     * 
     * @param ankle 
     * @param points_planes 
     * @param allpoints 
     * @return true 
     * @return false 
     */
    bool getPointsInFootArea(Eigen::Vector3d ankle, IndexPlanePoints & index_plane, int & area_cells);

    /**
     * @brief 根据localmap计算在ankle处，支撑点数、超出点数、平面法向量
     * 
     * @param ankle 
     * @param max_size 
     * @param above_points 
     * @param plane_normal 
     * @return true 
     * @return false 
     */
    bool computeLandInfo(Eigen::Vector3d ankle, int & max_size, int & above_points, Eigen::Vector3d & plane_normal, double & step_height, int & plane_index, double & pitch, double & roll);

    /**
     * @brief 根据地图坐标系下的normal，计算在经过yaw旋转后，平面的roll和pitch角
     * 
     * @param normal 
     * @param yaw 
     * @param roll 
     * @param pitch 
     */
    void computeRollPitch(Eigen::Vector3d normal, double yaw, Eigen::Vector3d & euler);

    /**
     * @brief 计算node的启发项，根据左右脚判断投影距离和角度差异
     * 
     * @param node 
     * @param hcost 
     * @return true 
     * @return false 
     */
    bool computeHcost(FootstepNodePtr node, double & hcost);
    // swing_height_change 终点减起点

    bool computeGcost(FootstepNodePtr node, double & gcost);

    /**
     * @brief 计算swing过程的最大摆动高度和起点与终点的高度差
     * 
     * @param start 
     * @param end 
     * @param swing_height_change 
     * @param swing_height_max 
     * @return true 
     * @return false 
     */
    bool swingHeight(grid_map::Position start, grid_map::Position end, double & swing_height_change, double & swing_height_max);
    // 根据终点解算机器人在终点时左右脚的位置
    /**
     * @brief 计算终点处左右脚的站立位置
     * 
     * @param goal 
     * @return true 
     * @return false 
     */
    bool computerLeftRightGoal(Eigen::Vector3d goal);

    /**
     * @brief 根据当前节点是哪只脚判断是否达到终点
     * 
     * @param node 
     * @return true 
     * @return false 
     */
    bool arriveGoal(FootstepNodePtr node);

    /**
     * @brief 规划
     * 
     * @return true 
     * @return false 
     */
    bool plan();

    /**
     * @brief Get the Footsteps object获取规划后的落脚点
     * 
     * @param node 
     * @return true 
     * @return false 
     */
    bool getFootsteps(FootstepNodePtr node);

    /**
     * @brief 补上并步的落脚
     * 
     * @param current_step 
     * @param repairStep 
     * @return true 
     * @return false 
     */
    bool repairStanceStep(Footstep current_step, Footstep & repairStep);

    /**
     * @brief Get the Node String object
     * 
     * @param node 
     * @param s 
     * @return true 
     * @return false 
     */
    bool getNodeString(FootstepNodePtr node, string & s);

    inline vector<Footstep> getResultSteps()
    {
        if (!checkFootstepsResult())
        {
            LOG(ERROR)<<"steps is error, planning algorithm is need to check";
            steps.clear();
            LOG(INFO)<<"clear the planed footsteps";
        }
        return steps;
    }

    /**
     * @brief 使用分割的plane_image和落脚点，计算障碍点
     * 
     * @return vector<vector<Eigen::Vector3d>> 
     */
    vector<vector<Eigen::Vector3d>> computeAvoidPoints();

    /**
     * @brief 计算两点之间的凸点，由于平面欠分割导致
     * 
     * @param start 
     * @param end 
     * @param convex_point 
     * @return true 
     * @return false 
     */
    bool getConvexPoint(Eigen::Vector3d start, Eigen::Vector3d end, Eigen::Vector3d convex_point);

    Eigen::Vector3d adjustEulerAngles(const Eigen::Vector3d& eulerAngles);

    bool checkFootstepsResult();

    // for test
    /**
     * @brief 测试节点扩展函数
     * 
     * @return true 
     * @return false 
     */
    bool testNodeExtension();

    bool testBasicTransitions();

    bool testComputeHcost();

    bool testGetNodeString();

    bool testArriveGoal();

    bool testGetPointsInFootArea();
    
    bool testComputeLandInfo();
    
    bool testFineTransitions();

    bool testFineTransitionsBasic();

    bool testFineLandPoint();

    // 构造函数时测试
    bool testComputerLeftRightGoal();

    bool testComputeLandPointScore();

    bool testComputeRollPitch();

    bool testSwingHeight();

    bool testComputeTransitionScore();

    bool testComputeTransitionStrictScore();

    bool testPlan();

    bool testComputeAvoidPoints();

    ~AstarHierarchicalFootstepPlanner();
};


