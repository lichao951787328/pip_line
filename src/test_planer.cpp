#include <local_plannerBase.h>
#include <grid_map_msgs/GridMap.h>
#include <grid_map_ros/GridMapRosConverter.hpp>
#include <grid_map_core/GridMap.hpp>
#include <ros/ros.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <mutex>
#include <tf2/utils.h>
#include <AstarHierarchicalFootstepPlanner/AstarHierarchicalFootstepPlannerPropose.h>
#include <local_plannerBase.h>
#include <glog/logging.h>
// 自己生成地图，选择起点终点来测试算法，不是局部规划
class testPlaner
{
private:
    ros::NodeHandle nh;
    ros::Publisher map_pub;
    ros::Subscriber goal_sub;
    ros::Subscriber start_sub;
    std::mutex data_mutex;
    bool is_goal_received = false;
    bool is_start_received = false;
    Eigen::Vector3d start, goal;
    ros::Timer timer; // 定时器
    grid_map_msgs::GridMap msg;
    grid_map::GridMap map;

    FootParam foot_param;
    double hip_width;
public:
    testPlaner(ros::NodeHandle & nh_);
    void goalCallback(const geometry_msgs::PoseStamped::ConstPtr & msg);
    void startCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr & msg);
    void timerCallback(const ros::TimerEvent & e);
    ~testPlaner();
};

testPlaner::testPlaner(ros::NodeHandle & nh_):nh(nh_)
{
    FootParam footparam(0.16, 0.11, 0.065, 0.065, 0.1, 0);


    nh.param("foot_param_x_upper", foot_param.x_upper, 0.16);
    nh.param("foot_param_x_button", foot_param.x_button, 0.11);
    nh.param("foot_param_y_left", foot_param.y_left, 0.065);
    nh.param("foot_param_y_right", foot_param.y_right, 0.065);
    nh.param("foot_param_x_fore_button", foot_param.x_fore_button, 0.1);
    nh.param("foot_param_x_hind_top", foot_param.x_hind_top, 0.);
    

    nh.param("hip_width", hip_width, 0.2);
    

    map_pub = nh.advertise<grid_map_msgs::GridMap>("/test_map", 1);
    goal_sub = nh.subscribe("/move_base_simple/goal", 1, &testPlaner::goalCallback, this);
    start_sub = nh.subscribe("/initialpose", 1, &testPlaner::startCallback, this);
    timer = nh.createTimer(ros::Duration(0.5), &testPlaner::timerCallback, this);

    map.add("elevation");
    // grid_map::GridMap map({"elevation"});
    map.setFrameId("map");
    // map.setGeometry(grid_map::Length(map_length, map_width), resolution, grid_map::Position(map_length/2.0, 0));

    // 创建地图
    // grid_map::GridMap con_map({"elevation"});
    grid_map::Length length(5, 5);
    map.setGeometry(length, 0.02, grid_map::Position(2.5, 0));
    int step_length = 1/map.getResolution();
    int insert_index = 0;
    int height_change = 0;
    // double step_elevation = 0;
    while (insert_index < map.getSize().x())
    {
        for (int i = 0; i < step_length; i++)
        {
            if (insert_index + i >= map.getSize().x())
            {
                break;
            }
            for (int j = 0; j < map.getSize().y(); j++)
            {
                map["elevation"](insert_index + i, j) = height_change * 0.1;
            }
        }
        insert_index += step_length;
        height_change++;
    }
    // con_map.setFrameId("map");
    grid_map::GridMapRosConverter::toMessage(map, msg);
}

void testPlaner::timerCallback(const ros::TimerEvent & event)
{
    // ROS_INFO("Timer triggered!");
    // ROS_INFO("Expected time: %f", event.current_expected.toSec());
    // ROS_INFO("Actual time: %f", ros::Time::now().toSec());
    map_pub.publish(msg);
    Eigen::Vector3d start_map, goal_map;
    if (is_goal_received && is_start_received)
    {
        {
            std::lock_guard<std::mutex> lock(data_mutex);
            try
            {
                goal_map = goal;
                is_goal_received = false;

                start_map = start;
                is_start_received = false;
            }
            catch (const std::exception& e)
            {
                // 处理异常，确保锁能正确释放
                std::cerr << "Exception caught: " << e.what() << std::endl;
            }
        }
        // 执行规划代码
        ROS_INFO("plan goal: %f, %f, %f", goal_map(0), goal_map(1), goal_map(2));
        ROS_INFO("plan start: %f, %f, %f", start_map(0), start_map(1), start_map(2));
        std::shared_ptr<AstarHierarchicalFootstepPlannerBase> planner_ptr = std::make_shared<AstarHierarchicalFootstepPlannerPropose>();
        // planner_ptr 使用move转移所有权，避免拷贝后，planner_ptr为空
        localPlannerBase localplaner(planner_ptr);
        localplaner.setFootParam(foot_param);
        localplaner.setHipWidth(hip_width);
        localplaner.mapPrepare(map);
        Eigen::Vector3d left_foot, right_foot;
        if (localplaner.isGoalFeasible(goal_map) && localplaner.isStartFeasible(start_map, left_foot, right_foot))
        {
            LOG(INFO)<<left_foot.transpose();
            LOG(INFO)<<right_foot.transpose();      
            localplaner.initial(left_foot, right_foot, 0, goal_map);
            localplaner.plan();
        }
        else
        {
            LOG(INFO)<<"error goal or start"<<endl;
        }
         
    }
}

void testPlaner::goalCallback(const geometry_msgs::PoseStamped::ConstPtr & msg)
{
    std::lock_guard<std::mutex> lock(data_mutex);
    goal(0) = msg->pose.position.x;
    goal(1) = msg->pose.position.y;
    goal(2) = tf2::getYaw(msg->pose.orientation);
    is_goal_received = true;
    ROS_INFO("Goal received: %f, %f, %f", goal(0), goal(1), goal(2));
}

void testPlaner::startCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr & msg)
{
    std::lock_guard<std::mutex> lock(data_mutex);
    start(0) = msg->pose.pose.position.x;
    start(1) = msg->pose.pose.position.y;
    start(2) = tf2::getYaw(msg->pose.pose.orientation);
    is_start_received = true;
    ROS_INFO("start received: %f, %f, %f", start(0), start(1), start(2));
}

testPlaner::~testPlaner()
{
}

int main(int argc, char **argv)
{
    google::InitGoogleLogging(argv[0]); 
    google::InstallFailureSignalHandler();
    // google::SetCommandLineOptionWithMode("FLAGS_minloglevel", "2");
    FLAGS_minloglevel = 0;
    FLAGS_colorlogtostderr = true;
    FLAGS_alsologtostderr = true;
    ros::init(argc, argv, "test_planer");
    ros::NodeHandle nh;
    testPlaner test_planer(nh);
    ros::AsyncSpinner spinner(nh.param("num_callback_threads", 2));  // Use n threads
    spinner.start();
    ros::waitForShutdown();
    return 0;
}

