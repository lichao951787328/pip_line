#include <local_plan_node.h>
#include <ros/ros.h>
#include <grid_map_msgs/GridMap.h>
#include <grid_map_ros/GridMapRosConverter.hpp>
#include <Eigen/Core>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <visualization_msgs/MarkerArray.h>
#include <tf2/exceptions.h>
#include <AstarHierarchicalFootstepPlanner/AstarHierarchicalFootstepPlannerPropose.h>
#include <glog/logging.h>
#include <chrono>
#include <future>
#include <thread>
#include <grid_map_cv/InpaintFilter.hpp>
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
    

    nh.param("hip_width", hip_width, 0.05);
    

    nh.param("global_path_topic", global_path_topic, string("/global_path"));
    sub_global_path = nh.subscribe(global_path_topic, 1, &localPlanNode::globalPathCallback, this);

    nh.param("foosteps_topic", foosteps_topic, string("/footsteps"));
    pub_footsteps = nh.advertise<diy_msgs::footSteps>(foosteps_topic, 1);
    pub_footsteps_visual = nh.advertise<visualization_msgs::MarkerArray>(foosteps_topic+"visual", 1);
    // pub_robotState_Feet = nh.advertise<visualization_msgs::MarkerArray>("/robotState_Feet", 1);

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

bool localPlanNode::getRobotState(uint32_t map_id, diy_msgs::robotState & robot_state)
{
    auto it = robot_states.begin();
    for (; it != robot_states.end(); it++)
    {
        if (it->header.seq == map_id)
        {
            LOG(INFO)<<"seq: "<<it->header.seq;
            robot_state = *it;
            robot_states.erase(robot_states.begin(), it);
            return true;
        }
    }
    return false;
}


void localPlanNode::publishFootsteps(diy_msgs::footSteps steps)
{   
    LOG(INFO)<<"steps size: "<<steps.footsteps.size();
    
    visualization_msgs::MarkerArray markerArray;
    

    for (int i = 0; i < steps.footsteps.size(); i++)
    {
        diy_msgs::footStep step = steps.footsteps.at(i);
        visualization_msgs::Marker marker;
        
        marker.header = steps.header;
        marker.header.frame_id = local_map_frame;
        marker.id = i;
        marker.type = visualization_msgs::Marker::CUBE;
        marker.action = visualization_msgs::Marker::ADD;
        marker.pose.position.x = step.x; // 中心点的x坐标
        marker.pose.position.y = step.y; // 中心点的y坐标
        marker.pose.position.z = step.z; // 中心点的z坐标
        Eigen::Quaterniond q_new;
        q_new = Eigen::AngleAxisd(step.yaw, Eigen::Vector3d::UnitZ()) *
            Eigen::AngleAxisd(step.pitch, Eigen::Vector3d::UnitY()) *
            Eigen::AngleAxisd(step.roll, Eigen::Vector3d::UnitX());
        marker.pose.orientation.x = q_new.x();
        marker.pose.orientation.y = q_new.y();
        marker.pose.orientation.z = q_new.z();
        marker.pose.orientation.w = q_new.w();
        marker.scale.x = 0.27; // 矩形的宽度
        marker.scale.y = 0.13; // 矩形的高度
        marker.scale.z = 0.05; // 矩形的厚度
        if (step.is_left)
        {
            marker.ns = "left_foot";
            // 设置颜色
            marker.color.r = 0.8f;
            marker.color.g = 1.0f;
            marker.color.b = 0.0f;
            marker.color.a = 0.5;
        }
        else
        {
            marker.ns = "right_foot";
            marker.color.r = 0.0f;
            marker.color.g = 0.8f;
            marker.color.b = 1.0f;
            marker.color.a = 0.5;
        }
        markerArray.markers.emplace_back(marker);
    }
    LOG(INFO)<<"markerArray.markers.size(): "<<markerArray.markers.size();
    pub_footsteps_visual.publish(markerArray);
    // return markerArray;
}


void localPlanNode::InPaintFilter(grid_map::GridMap & mapIn, grid_map::GridMap & mapOut)
{
    string outputLayer_ = "elevation";
    string inputLayer_ = "elevation";
    double radius_ = 0.5;
    mapOut = mapIn;
    
    
    //Convert elevation layer to OpenCV image to fill in holes.
    //Get the inpaint mask (nonzero pixels indicate where values need to be filled in).
    mapOut.add("inpaint_mask", 0.0);
    
    mapOut.setBasicLayers(std::vector<std::string>());
    for (grid_map::GridMapIterator iterator(mapOut); !iterator.isPastEnd(); ++iterator) {
        if (!mapOut.isValid(*iterator, inputLayer_)) {
        mapOut.at("inpaint_mask", *iterator) = 1.0;
        }
    }
    cv::Mat originalImage;
    cv::Mat mask;
    cv::Mat filledImage;
    const float minValue = mapOut.get(inputLayer_).minCoeffOfFinites();
    const float maxValue = mapOut.get(inputLayer_).maxCoeffOfFinites();
    
    grid_map::GridMapCvConverter::toImage<unsigned char, 3>(mapOut, inputLayer_, CV_8UC3, minValue, maxValue,
                                                            originalImage);
    grid_map::GridMapCvConverter::toImage<unsigned char, 1>(mapOut, "inpaint_mask", CV_8UC1, mask);
    
    const double radiusInPixels = radius_ / mapIn.getResolution();
    cv::inpaint(originalImage, mask, filledImage, radiusInPixels, cv::INPAINT_NS);
    
    mapOut.erase(outputLayer_);
    mapOut.add(outputLayer_);
    grid_map::GridMapCvConverter::addLayerFromImage<unsigned char, 3>(filledImage, outputLayer_, mapOut, minValue, maxValue);
    mapOut.erase("inpaint_mask");
}

// 怎么保证map与state同步?通过map与robotstate的id是否一致来判断。这要求在从控制的数据发送出来时，就会有一个id，这个id会在map与state中都有。
void localPlanNode::mapCallback(const grid_map_msgs::GridMap::ConstPtr& msg)
{
    auto start = std::chrono::high_resolution_clock::now();
        // LOG(INFO)<<"COST: "<<std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();
    LOG(INFO)<<"enter map callback";

    if (global_path.poses.empty())
    {
        LOG(ERROR)<<"global path is empty";
        return;
    }
    grid_map::GridMap map, tmpmap;

    // inpaint 一下
    
    grid_map::GridMapRosConverter::fromMessage(*msg, map);
    
    InPaintFilter(map, tmpmap);
    map = tmpmap;
    
    // 获取3d到localmap的变换矩阵
    geometry_msgs::TransformStamped transformStamped_T_localmap_globalmap;
    try
    {   
        // base到localmap的转换矩阵
        transformStamped_T_localmap_globalmap = tf_buffer_->lookupTransform(local_map_frame, global_terrain_map_frame, ros::Time(0));
    }
    catch(tf2::TransformException &ex)
    {
        std::cerr << ex.what() << '\n';
    }
    tf2::Transform transform_localmap_globalmap;
    tf2::fromMsg(transformStamped_T_localmap_globalmap.transform, transform_localmap_globalmap);
    // 将path 转到 localmap 坐标系下，转换后的点极有可能没有落在localmap内。在选择终点时，需要将其投影到localmap上
    // vector<tf2::Transform> path_localmap = transformPose(transform_localmap_globalmap);
    // 获得最远的局部终点
    // LOG(INFO)<<"set planer";

    // note：用同一个规划器来进行规划时，如果多次使用同一个规划器，会出现内存溢出的情况，可能时这个规划器没写好，
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
    Eigen::Vector3d goal_localmap;
    bool get_goal = false;
    if (global_path.poses.empty())
    {
        LOG(ERROR)<<"global path is empty";
        return;
    }
    vector<tf2::Transform> path_In_localmap = transformPose(transform_localmap_globalmap);
    std::reverse(path_In_localmap.begin(), path_In_localmap.end());
    
    // 由于收到的是稠密的path，所以只按照5cm一次进行检查。
    int resolution_search = (0.05/map.getResolution());
    for (int i = 0; i < path_In_localmap.size(); i=i+resolution_search)
    {
        auto point = path_In_localmap.at(i);
        // 还需要保证局部终点位于机器人的前方区域
        // 由于base坐标系与localmap坐标系重合，只需保证
        if (point.getOrigin().x() > 0)
        {
            Eigen::Quaterniond qd;
            qd.x() = point.getRotation().x();
            qd.y() = point.getRotation().y();
            qd.z() = point.getRotation().z();
            qd.w() = point.getRotation().w();
            Eigen::Vector3d v_x = qd.toRotationMatrix() * Eigen::Vector3d::UnitX();
            double yaw = std::atan2(v_x.y(), v_x.x());

            if (local_planner_propose.isGoalFeasible(Eigen::Vector3d(point.getOrigin().x(), point.getOrigin().y(), yaw)))
            {
                goal_localmap = Eigen::Vector3d(point.getOrigin().x(), point.getOrigin().y(), yaw);
                get_goal = true;
                break;
            }
        }
    }
    diy_msgs::footSteps footsteps;
    footsteps.header.frame_id = "localmap";
    // LOG(INFO)<<goal_localmap.transpose();
    // 如果没有找到终点，则发布空落脚点
    if (!get_goal)
    {
        LOG(INFO)<<"no feasible goal";
        footsteps.plan_success = false;
            // LOG(INFO)<<"can not plan footsteps or time out";
        pub_footsteps.publish(footsteps);
        return;
    }
    
    // 如果终点的norm比较小，就是已经到达终点了，如果左右角不是并脚，就返回并脚。这些操作可以在planner里面写
    LOG(INFO)<<"goal: "<<goal_localmap.transpose();
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
        diy_msgs::robotState robot_state;
        {
            std::lock_guard<std::mutex> lock(received_mutex);

            // 获取机器人和map id对应的robot state
            if (!robot_state_received)
            {
                LOG(INFO)<<"robot state not received";
                return;
            }
            if (!getRobotState(msg->info.header.seq, robot_state))
            {
                LOG(INFO)<<"can not find corresponding robot state";
                return;
            }
        }
        
        Eigen::Vector3d start_left, start_right;
        if (msg->info.header.seq == 0) // 初始时发来的数是0，0，0
        {
            start_left = Eigen::Vector3d(0, 0.1, 0);
            start_right = Eigen::Vector3d(0, -0.1, 0);
        }
        else
        {
            Eigen::Quaterniond qdl;
            qdl.x() = robot_state.left_foot.orientation.x;
            qdl.y() = robot_state.left_foot.orientation.y;
            qdl.z() = robot_state.left_foot.orientation.z;
            qdl.w() = robot_state.left_foot.orientation.w;
            Eigen::Vector3d v_x = qdl.toRotationMatrix() * Eigen::Vector3d::UnitX();
            double yaw = std::atan2(v_x.y(), v_x.x());
            start_left = Eigen::Vector3d(robot_state.left_foot.position.x, robot_state.left_foot.position.y, yaw);

            Eigen::Quaterniond qdr;
            qdr.x() = robot_state.right_foot.orientation.x;
            qdr.y() = robot_state.right_foot.orientation.y;
            qdr.z() = robot_state.right_foot.orientation.z;
            qdr.w() = robot_state.right_foot.orientation.w;
            v_x = qdr.toRotationMatrix() * Eigen::Vector3d::UnitX();
            yaw = std::atan2(v_x.y(), v_x.x());
            start_right = Eigen::Vector3d(robot_state.right_foot.position.x, robot_state.right_foot.position.y, yaw);
        }

        
        // 支撑脚转换
        int support_flag;
        LOG(INFO)<<"foot_state: "<<robot_state.foot_state;
        // robot_state.foot_state == 1 定是左脚支撑
        // robot_state.foot_state == 2 双脚支撑或右脚支撑


        if (robot_state.foot_state == 2)
        {
            if (msg->info.header.seq == 0)
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
        // 要注意这个地方，支撑脚是右脚时，规划起始步是左脚，因为摆动周期内支撑脚为支撑脚，总以下一个双脚支撑期为规划起点
        if (support_flag == 1)
        {
            local_planner_propose.initial(start_right, start_left, support_flag, goal_localmap);
        }
        else
        {
            local_planner_propose.initial(start_left, start_right, support_flag, goal_localmap);
        }
        
        LOG(INFO)<<"planner initial";

#ifdef PLANNING_TIMER_CHECK
        // 为了保证规划的时间不会太长，设置一个超时时间
        bool running_flag;
        // 使用 std::async 异步启动 plan()
        auto future = std::async(std::launch::async, [&local_planner_propose]() {
            return local_planner_propose.plan();
        });
        auto planning_start = std::chrono::high_resolution_clock::now();
        // 等待任务完成或超时
        if (future.wait_for(std::chrono::milliseconds(800)) == std::future_status::ready) {
            // 任务在超时时间内完成
            running_flag = future.get();
        } else {
            // 超时，取消任务
            LOG(INFO) << "Plan timeout, cancelling...";
            local_planner_propose.cancelPlanning(); // 设置取消标志位
            if (future.valid()) 
            {
                future.wait(); // 确保任务结束
            }
            running_flag = false;
        }
        auto planning_end = std::chrono::high_resolution_clock::now();
        LOG(INFO)<<"planning COST: "<<std::chrono::duration_cast<std::chrono::milliseconds>(planning_end - planning_start).count();
        if (running_flag)
        {
            vector<Footstep> steps = local_planner_propose.getFootsteps();
            for(auto & step : steps)
            {
                diy_msgs::footStep step_msg;
                step_msg.is_left = (step.robot_side == LEFT);
                step_msg.x = step.x;
                step_msg.y = step.y;
                // 这是一个规划的偏置
                step_msg.z = step.z;
                step_msg.roll = step.roll;
                step_msg.pitch = step.pitch;
                step_msg.yaw = step.yaw;
                footsteps.footsteps.emplace_back(step_msg);
            }
            footsteps.header = msg->info.header;
            publishFootsteps(footsteps);
            footsteps.plan_success = true;
            pub_footsteps.publish(footsteps);
        }
        else
        {
            footsteps.footsteps.clear();
            footsteps.plan_success = false;
            LOG(INFO)<<"can not plan footsteps or time out";
            pub_footsteps.publish(footsteps);
            return;
        }
#else
        auto planning_start = std::chrono::high_resolution_clock::now();
        
        if (local_planner_propose.plan())
        {
            auto planning_end = std::chrono::high_resolution_clock::now();
            LOG(INFO)<<"planning COST: "<<std::chrono::duration_cast<std::chrono::milliseconds>(planning_end - planning_start).count();
            vector<Footstep> steps = local_planner_propose.getFootsteps();
            for(auto & step : steps)
            {
                diy_msgs::footStep step_msg;
                step_msg.is_left = (step.robot_side == LEFT);
                step_msg.x = step.x;
                step_msg.y = step.y;
                step_msg.z = step.z;
                step_msg.roll = step.roll;
                step_msg.pitch = step.pitch;
                step_msg.yaw = step.yaw;
                footsteps.footsteps.emplace_back(step_msg);
            }
            footsteps.header = msg->info.header;
            footsteps.plan_success = true;
            publishFootsteps(footsteps);
            pub_footsteps.publish(footsteps);
        }
        else
        {
            auto planning_end = std::chrono::high_resolution_clock::now();
            LOG(INFO)<<"planning COST: "<<std::chrono::duration_cast<std::chrono::milliseconds>(planning_end - planning_start).count();
            LOG(INFO)<<"can not plan footsteps";
            footsteps.footsteps.clear();
            footsteps.plan_success = false;
            pub_footsteps.publish(footsteps);
            return;
        }
#endif
    }
    auto end = std::chrono::high_resolution_clock::now();
    LOG(INFO)<<"COST: "<<std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();
}

void localPlanNode::robotStateCallback(const diy_msgs::robotState::ConstPtr& msg)
{
    if (!global_path.poses.empty())
    {
        std::lock_guard<std::mutex> lock(received_mutex);
        ROS_INFO("robot state received");

        // LOG(INFO)<<msg->foot_state;
        // LOG(INFO)<<msg->currentBase.position.x<<" "<<msg->currentBase.position.y<<" "<<msg->currentBase.position.z;
        // LOG(INFO)<<msg->left_foot.position.x<<" "<<msg->left_foot.position.y<<" "<<msg->left_foot.position.z;
        // LOG(INFO)<<msg->right_foot.position.x<<" "<<msg->right_foot.position.y<<" "<<msg->right_foot.position.z;

        robot_state_received = true;
        robot_states.emplace_back(*msg);
        // robot_state = *msg;
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
    google::InitGoogleLogging(argv[0]); 
    google::InstallFailureSignalHandler();
    // google::SetCommandLineOptionWithMode("FLAGS_minloglevel", "2");
    FLAGS_minloglevel = 0;
    FLAGS_colorlogtostderr = true;
    FLAGS_alsologtostderr = true;
    ros::init(argc, argv, "local_plan_node");
    ros::NodeHandle n;
    localPlanNode local_plan_node(n);
    ros::AsyncSpinner spinner(n.param("num_callback_threads", 2));  // Use n threads
    spinner.start();
    ros::waitForShutdown();
    return 0;
}