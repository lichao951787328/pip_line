#include <grid_map_core/GridMap.hpp>
#include <grid_map_msgs/GridMap.h>
#include <grid_map_ros/grid_map_ros.hpp>
#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <AstarHierarchicalFootstepPlanner/AstarHierarchicalFootstepPlannerPropose.h>
#include <local_plannerBase.h>
#include <geometry_msgs/PoseStamped.h>
#include <iostream>
#include <tf/tf.h>
#include <visualization_msgs/MarkerArray.h>
#include <diy_msgs/robotState.h>
using namespace std;
Eigen::Vector3d goal;
bool get_goal = false;

void getgoal(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    // 
    double yaw = tf::getYaw(msg->pose.orientation);
    ROS_INFO("Received goal with yaw: %f", yaw);
    goal = Eigen::Vector3d(msg->pose.position.x, msg->pose.position.y, yaw);
    get_goal = true;
}
int main(int argc, char** argv) 
{
    ros::init(argc, argv, "real_step_map");
    ros::NodeHandle nh;
    ros::Publisher pub = nh.advertise<grid_map_msgs::GridMap>("/localmap", 1);
    ros::Publisher state_foot_pub = nh.advertise<visualization_msgs::MarkerArray>("/state_foot", 1);
    ros::Publisher steps_pub = nh.advertise<visualization_msgs::MarkerArray>("/steps", 1);
    ros::Subscriber goal_sub = nh.subscribe("/move_base_simple/goal", 1, &getgoal);

    rosbag::Bag bag;
    bag.open("/home/lichao/TCDS/src/pip_line/data/2025-03-04-23-15-31.bag", rosbag::bagmode::Read);

    std::vector<std::string> topics;
    topics.push_back(std::string("/localmap"));

    vector<grid_map::GridMap> maps;
    vector<grid_map_msgs::GridMap> msgs;

    for (rosbag::MessageInstance const m : rosbag::View(bag))
    {
        if (m.getTopic() == "/localmap")
        {
            grid_map_msgs::GridMap::ConstPtr grid_map_msg = m.instantiate<grid_map_msgs::GridMap>();
            if (grid_map_msg != nullptr)
            {
                cout << "Received a grid map message" << endl;
                grid_map::GridMap map;
                grid_map::GridMapRosConverter::fromMessage(*grid_map_msg, map);
                maps.emplace_back(map);
                msgs.emplace_back(*grid_map_msg);
            }
        }
    }
    bag.close();

    cout<<"maps.size():"<<maps.size()<<endl;
    cout<<"msgs.size():"<<msgs.size()<<endl;

    rosbag::Bag state_bag;
    state_bag.open("/home/lichao/TCDS/src/pip_line/data/2025-03-04-11-07-07.bag", rosbag::bagmode::Read);

    std::vector<std::string> state_topics;
    state_topics.push_back(std::string("/robot_state"));

    vector<diy_msgs::robotState> states;
    
    for (rosbag::MessageInstance const m : rosbag::View(state_bag))
    {
        if (m.getTopic() == "/robot_state")
        {
            // Assuming the message type is std_msgs::String for demonstration purposes
            diy_msgs::robotState::ConstPtr state_msg = m.instantiate<diy_msgs::robotState>();
            if (state_msg != nullptr)
            {
                cout << "Received a robot state message: "<< endl;
                states.emplace_back(*state_msg);
            }
        }
    }
    state_bag.close();
    LOG(INFO)<<"states: "<<states.size();
    FootParam foot_param;
    foot_param.x_upper = 0.156;
    foot_param.x_button = 0.1;
    foot_param.y_left = 0.08;
    foot_param.y_right = 0.08;
    foot_param.x_fore_button = 0.11;
    foot_param.x_hind_top = 0.01;
    double hip_width = 0.2;

    for (int i = 9; i < msgs.size(); i++)
    {
        grid_map::GridMap map = maps.at(i);
        grid_map_msgs::GridMap msg = msgs.at(i);
        diy_msgs::robotState robot_state = states.at(i);

        int nan_num = 0;
        for (int i = 0; i < map.getSize().x(); i++)
        {
            for (int j = 0; j < map.getSize().y(); j++)
            {
                grid_map::Index index(i, j);
                if (!map.isValid(index, "elevation"))
                {
                    nan_num++;
                }
            }
        }
        cout<<"nan_num: "<<nan_num<<endl;
        localPlannerBase local_planner_propose;
        // LOG(INFO)<<"set planer";
        // 声明自己想使用的规划器
        std::shared_ptr<AstarHierarchicalFootstepPlannerPropose> propose_planner_ptr = std::make_shared<AstarHierarchicalFootstepPlannerPropose>();
        local_planner_propose.setPlanner(propose_planner_ptr);
        // LOG(INFO)<<"setFootParam";
        local_planner_propose.setFootParam(foot_param);
        local_planner_propose.setHipWidth(hip_width);
        // LOG(INFO)<<"mapPrepare";
        local_planner_propose.mapPrepare(map);


        Eigen::Quaterniond qdl;
        qdl.x() = robot_state.left_foot.orientation.x;
        qdl.y() = robot_state.left_foot.orientation.y;
        qdl.z() = robot_state.left_foot.orientation.z;
        qdl.w() = robot_state.left_foot.orientation.w;
        Eigen::Vector3d v_x = qdl.toRotationMatrix() * Eigen::Vector3d::UnitX();
        double yaw = std::atan2(v_x.y(), v_x.x());
        Eigen::Vector3d start_left = Eigen::Vector3d(robot_state.left_foot.position.x, robot_state.left_foot.position.y, yaw);

        Eigen::Quaterniond qdr;
        qdr.x() = robot_state.right_foot.orientation.x;
        qdr.y() = robot_state.right_foot.orientation.y;
        qdr.z() = robot_state.right_foot.orientation.z;
        qdr.w() = robot_state.right_foot.orientation.w;
        v_x = qdr.toRotationMatrix() * Eigen::Vector3d::UnitX();
        yaw = std::atan2(v_x.y(), v_x.x());
        Eigen::Vector3d start_right = Eigen::Vector3d(robot_state.right_foot.position.x, robot_state.right_foot.position.y, yaw);

        if (i == 0)
        {
            start_left = Eigen::Vector3d(0, 0.1, 0);
            start_right = Eigen::Vector3d(0, -0.1, 0);
        }
        

        int support_flag;
        LOG(INFO)<<"foot_state: "<<robot_state.foot_state;
        // robot_state.foot_state == 1 定是左脚支撑
        // robot_state.foot_state == 2 双脚支撑或右脚支撑

        if (robot_state.foot_state == 2)
        {
            if (i == 0)
            {
                support_flag = 2; // 双脚支撑
                LOG(INFO)<<"double support";
            }
            else
            {
                support_flag = 0;// 右脚支撑
                LOG(INFO)<<"right support";
            }
        }
        else
        {
            support_flag = 1; // 左脚支撑
            LOG(INFO)<<"left support";
        }
        visualization_msgs::MarkerArray marker_array;
        visualization_msgs::Marker marker;
        marker.header.frame_id = "localmap";
        marker.header.stamp = ros::Time::now();
        marker.ns = "state";
        marker.type = visualization_msgs::Marker::ARROW;
        marker.action = visualization_msgs::Marker::ADD;
        marker.scale.x = 0.1;
        marker.scale.y = 0.02;
        marker.scale.z = 0.02;
        marker.color.a = 1.0;
        marker.color.r = 1.0;
        marker.color.g = 0.0;
        marker.color.b = 0.0;

        // Left foot marker
        
        
        marker.id = 0;
        marker.pose.position.x = robot_state.left_foot.position.x;
        marker.pose.position.y = robot_state.left_foot.position.y;
        marker.pose.position.z = robot_state.left_foot.position.z;
        marker.pose.orientation = robot_state.left_foot.orientation;

        if (i == 0)
        {
            marker.pose.position.x = 0;
            marker.pose.position.y = 0.1;
            marker.pose.position.z = 0;
            marker.pose.orientation.w = 1;
            marker.pose.orientation.x = 0;
            marker.pose.orientation.y = 0;
            marker.pose.orientation.z = 0;
        }
        marker_array.markers.push_back(marker);

        // Right foot marker
        marker.id = 1;
        marker.pose.position.x = robot_state.right_foot.position.x;
        marker.pose.position.y = robot_state.right_foot.position.y;
        marker.pose.position.z = robot_state.right_foot.position.z;
        marker.pose.orientation = robot_state.right_foot.orientation;
        if (i == 0)
        {
            marker.pose.position.x = 0;
            marker.pose.position.y = - 0.1;
            marker.pose.position.z = 0;
            marker.pose.orientation.w = 1;
            marker.pose.orientation.x = 0;
            marker.pose.orientation.y = 0;
            marker.pose.orientation.z = 0;
        }
        marker_array.markers.push_back(marker);

        
        // 要注意这个地方，支撑脚是右脚时，规划起始步是左脚，因为摆动周期内支撑脚为支撑脚，总以下一个双脚支撑期为规划起点
        // Eigen::Vector3d start(0.0, 0.1, 0.0);
        // Eigen::Vector3d pre_start(0.0, -0.1, 0.0);
        
        while (!get_goal)
        {
            sleep(1);
            state_foot_pub.publish(marker_array);
            pub.publish(msg);

            ros::spinOnce();
        }

        if (support_flag == 1)
        {
            local_planner_propose.initial(start_right, start_left, support_flag, goal);
        }
        else
        {
            local_planner_propose.initial(start_left, start_right, support_flag, goal);
        }
        // local_planner_propose.initial(start, pre_start, 0, goal);
        auto start_time = ros::Time::now();
        if (local_planner_propose.plan())
        {
            visualization_msgs::MarkerArray steps_markers;
            vector<Footstep> steps = local_planner_propose.getFootsteps();
            int index = 0;
            for(auto & step : steps)
            {
                cout<<step.x<<" "<<step.y<<" "<<step.z<<" "<<step.roll<<" "<<step.pitch<<" "<<step.yaw<<endl;
                visualization_msgs::Marker step_marker;
                step_marker.header.frame_id = "localmap";
                step_marker.header.stamp = ros::Time::now();
                step_marker.ns = "steps";
                step_marker.id = index++;
                step_marker.type = visualization_msgs::Marker::ARROW;
                step_marker.action = visualization_msgs::Marker::ADD;
                step_marker.scale.x = 0.1;
                step_marker.scale.y = 0.02;
                step_marker.scale.z = 0.02;
                step_marker.color.a = 1.0;
                step_marker.color.r = 0.0;
                step_marker.color.g = 1.0;
                step_marker.color.b = 0.0;
                step_marker.pose.position.x = step.x;
                step_marker.pose.position.y = step.y;
                step_marker.pose.position.z = step.z;
                tf::Quaternion q;
                q.setRPY(step.roll, step.pitch, step.yaw);
                tf::quaternionTFToMsg(q, step_marker.pose.orientation);
                steps_markers.markers.push_back(step_marker);
                
            }
            steps_pub.publish(steps_markers);
        }
        auto end_time = ros::Time::now();
        cout<<"time_consume: "<<(end_time - start_time).toSec()<<endl;
        cout<<"i = "<<i<<endl;
        // Wait for a key press to proceed to the next iteration
        cout << "Press Enter to proceed to the next iteration..." << endl;
        cin.ignore();
        get_goal = false;
    }
    return 0;
}
