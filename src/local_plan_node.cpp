#include <local_plan_node.h>
#include <ros/ros.h>
#include <grid_map_msgs/GridMap.h>
#include <grid_map_ros/GridMapRosConverter.hpp>
#include <Eigen/Core>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <tf2/exceptions.h>
localPlanNode::localPlanNode(ros::NodeHandle & n):nh(n)
{
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>();
    transform_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    nh.param("foot_param_x_upper", foot_param.x_upper, 0.05);
    nh.param("foot_param_x_button", foot_param.x_button, 0.05);
    nh.param("foot_param_y_left", foot_param.y_left, 0.05);
    nh.param("foot_param_y_right", foot_param.y_right, 0.05);
    nh.param("foot_param_x_fore_button", foot_param.x_fore_button, 0.05);
    nh.param("foot_param_x_hind_top", foot_param.x_hind_top, 0.05);
    local_planner.setFootParam(foot_param);

    nh.param("hip_width", hip_width, 0.05);
    local_planner.setHipWidth(hip_width);

    nh.param("global_path_topic", global_path_topic, string("/global_path"));
    sub_global_path = nh.subscribe(global_path_topic, 1, &localPlanNode::globalPathCallback, this);

    nh.param("foosteps_topic", foosteps_topic, string("/footsteps"));
    pub_footsteps = nh.advertise<diy_msgs::footSteps>(foosteps_topic, 1);

    nh.param("map_topic", map_topic, string("/local_map"));
    nh.param("robot_state_topic", robot_state_topic, string("/robot_state"));
    sub_map = nh.subscribe(map_topic, 1, &localPlanNode::mapCallback, this);
    sub_robot_state = nh.subscribe(robot_state_topic, 1, &localPlanNode::robotStateCallback, this);
    robot_state_received = false;

    nh.param("global_terrain_map_frame", global_terrain_map_frame, string("map"));
    nh.param("local_map_frame", local_map_frame, string("local_map"));
}

vector<tf2::Transform> localPlanNode::transformPose(tf2::Transform transform)
{
    vector<tf2::Transform> global_path_transformed;
    for (auto & poseS : global_path.poses)
    {
        tf2::Quaternion quat(poseS.pose.orientation.x, poseS.pose.orientation.y, poseS.pose.orientation.z, poseS.pose.orientation.w);
        tf2::Vector3 origin(poseS.pose.position.x, poseS.pose.position.y, poseS.pose.position.z);
        tf2::Transform transform_pose;
        transform_pose.setOrigin(origin);
        transform_pose.setRotation(quat);
        global_path_transformed.emplace_back(transform * transform_pose);
    }
    return global_path_transformed;
}

void localPlanNode::mapCallback(const grid_map_msgs::GridMap::ConstPtr& msg)
{
    grid_map::GridMap map;
    grid_map::GridMapRosConverter::fromMessage(*msg, map);

    local_planner.mapPrepare(map);
    
    // 为A*规划器设置初始值

    // 获取3d到localmap的变换矩阵
    geometry_msgs::TransformStamped transformStamped_T_localmap_globalmap;
    try
    {   
        // base到localmap的转换矩阵
        transformStamped_T_localmap_globalmap = tf_buffer_->lookupTransform(global_terrain_map_frame, local_map_frame, ros::Time(0));
    }
    catch(tf2::TransformException &ex)
    {
        std::cerr << ex.what() << '\n';
    }
    tf2::Transform transform_localmap_globalmap;
    tf2::fromMsg(transformStamped_T_localmap_globalmap.transform, transform_localmap_globalmap);
    // 将path 转到 localmap 坐标系下，转换后的点极有可能没有落在localmap内。在选择终点时，需要将其投影到localmap上
    vector<tf2::Transform> path_localmap = transformPose(transform_localmap_globalmap);
    // 获得最远的局部终点
    Eigen::Vector3d goal_localmap;
    std::reverse(path_localmap.begin(), path_localmap.end());
    for (auto & point : path_localmap)
    {
        if (map.isInside(grid_map::Position(point.getOrigin().x(), point.getOrigin().y())))
        {
            // 考虑到高程图总是在x-y平面上的，所以将终点的方向定义为3d方向在x-y投影的yaw角
            Eigen::Quaterniond qd;
            qd.x() = point.getRotation().x();
            qd.y() = point.getRotation().y();
            qd.z() = point.getRotation().z();
            qd.w() = point.getRotation().w();
            Eigen::Vector3d v_x = qd.toRotationMatrix() * Eigen::Vector3d::UnitX();
            double yaw = std::atan2(v_x.y(), v_x.x());

            if (local_planner.isGoalFeasible(Eigen::Vector3d(point.getOrigin().x(), point.getOrigin().y(), yaw)))
            {
                goal_localmap = Eigen::Vector3d(point.getOrigin().x(), point.getOrigin().y(), yaw);
                break;
            }
        }
    }
    // 如果终点的norm比较小，就是已经到达终点了，如果左右角不是并脚，就返回并脚。这些操作可以在planner里面写
    if (goal_localmap.head(2).norm() < 0.1)
    {
        // 不规划落脚点，因为可能已经到达中带哪
        // 发布空的落脚点
        footsteps.header = msg->info.header;
        footsteps.footsteps.clear();
        pub_footsteps.publish(footsteps);
    }       
    else
    {
        std::lock_guard<std::mutex> lock(received_mutex);
        if (robot_state_received)
        {
            // 由于base是与localmap是重合的，那么脚相对于base的想对位置即为脚相对于localmap的想对位置
            // tf2::Quaternion ql(robot_state.left_foot.orientation.x, robot_state.left_foot.orientation.y, robot_state.left_foot.orientation.z, robot_state.left_foot.orientation.w);
            // double yaw_left = tf2::getYaw(ql);
            // Eigen::Vector3d start_left(robot_state.left_foot.position.x, robot_state.left_foot.position.y, yaw_left);
            // tf2::Quaternion qr(robot_state.right_foot.orientation.x, robot_state.right_foot.orientation.y, robot_state.right_foot.orientation.z, robot_state.right_foot.orientation.w);
            // double yaw_right = tf2::getYaw(qr);
            // Eigen::Vector3d start_right(robot_state.right_foot.position.x, robot_state.right_foot.position.y, yaw_right);

            Eigen::Quaterniond qdl;
            qdl.x() = robot_state.left_foot.orientation.x;
            qdl.y() = robot_state.left_foot.orientation.y;
            qdl.z() = robot_state.left_foot.orientation.z;
            qdl.w() = robot_state.left_foot.orientation.w;
            Eigen::Vector3d v_x = qdl.toRotationMatrix() * Eigen::Vector3d::UnitX();
            double yaw = std::atan2(v_x.y(), v_x.x());
            Eigen::Vector3d start_left(robot_state.left_foot.position.x, robot_state.left_foot.position.y, yaw);

            Eigen::Quaterniond qdr;
            qdr.x() = robot_state.right_foot.orientation.x;
            qdr.y() = robot_state.right_foot.orientation.y;
            qdr.z() = robot_state.right_foot.orientation.z;
            qdr.w() = robot_state.right_foot.orientation.w;
            v_x = qdr.toRotationMatrix() * Eigen::Vector3d::UnitX();
            yaw = std::atan2(v_x.y(), v_x.x());
            Eigen::Vector3d start_right(robot_state.right_foot.position.x, robot_state.right_foot.position.y, yaw);
            
            // 解算局部终点
            local_planner.initial(start_left, start_right, robot_state.foot_state, goal_localmap);
            local_planner.plan();
            footsteps = local_planner.getResultSteps();
            footsteps.header = msg->info.header;
            pub_footsteps.publish(footsteps);
        }
    }
}

void localPlanNode::robotStateCallback(const diy_msgs::robotState::ConstPtr& msg)
{
    if (!global_path.poses.empty())
    {
        std::lock_guard<std::mutex> lock(received_mutex);
        ROS_INFO("robot state received");
        robot_state_received = true;
        robot_state = *msg;
    }    
}

void localPlanNode::globalPathCallback(const nav_msgs::Path::ConstPtr& msg)
{
    ROS_INFO("global path received");
    global_path = *msg;
}

localPlanNode::~localPlanNode()
{
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "local_plan_node");
    ros::NodeHandle n;
    localPlanNode local_plan_node(n);
    ros::AsyncSpinner spinner(n.param("num_callback_threads", 2));  // Use n threads
    spinner.start();
    ros::waitForShutdown();
    return 0;
}