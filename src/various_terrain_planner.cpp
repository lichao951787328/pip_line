#include <various_terrain_planner.h>
#include <grid_map_core/GridMap.hpp>
#include <grid_map_core/iterators/GridMapIterator.hpp>
#include <grid_map_core/iterators/SpiralIterator.hpp>
#include <grid_map_msgs/GetGridMap.h>
#include <AstarHierarchicalFootstepPlanner/AstarHierarchicalFootstepPlannerPropose.h>
#include <AstarHierarchicalFootstepPlanner/AstarHierarchicalFootstepPlannerTraditional.h>
#include <functional>
#include <boost/thread.hpp>
#include <boost/chrono.hpp>
#include <random>
#include <grid_map_ros/GridMapRosConverter.hpp>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <tf2/utils.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
// #define discontinuous_steps
#define WAVE
// #define flat_plane
// #define single_terrain_for_test
// #define Map_from_PCD
variousTerrainPlanner::variousTerrainPlanner(ros::NodeHandle nh_):nh(nh_)
{
    map_pub = nh.advertise<grid_map_msgs::GridMap>("map", 1, true);
    nh.param("foot_param_x_upper", foot_param.x_upper, 0.16);
    nh.param("foot_param_x_button", foot_param.x_button, 0.11);
    nh.param("foot_param_y_left", foot_param.y_left, 0.065);
    nh.param("foot_param_y_right", foot_param.y_right, 0.065);
    nh.param("foot_param_x_fore_button", foot_param.x_fore_button, 0.1);
    nh.param("foot_param_x_hind_top", foot_param.x_hind_top, 0.);
    nh.param("hip_width", hip_width, 0.2);
    nh.param("checkXupper", checkXupper, 0.13);
    nh.param("checkXButton", checkXButton, 0.04);
    file = std::ofstream(filename);
    if (!file.is_open()) 
    {
        std::cerr << "Failed to open the file: " << filename << std::endl;
        // return 1; // 返回错误码
    }

    // filetxt_ = std::ofstream(filetxt);
    // if (!filetxt.is_open())
    // {
    //     std::cerr << "Failed to open the file: " << filename << std::endl;
    // }
    

	grid_map::Position start(0.2, 0);
    start_points1.emplace_back(Eigen::Vector3d(start.x(), start.y(), 0));
    start_points1.emplace_back(Eigen::Vector3d(start.x(), start.y(), -45/57.3));
    start_points1.emplace_back(Eigen::Vector3d(start.x(), start.y(), 45/57.3));

    vector<Eigen::Vector2d> tmp_goal_points;
    tmp_goal_points.emplace_back(Eigen::Vector2d(4.7, 0));
    // tmp_goal_points.emplace_back(Eigen::Vector2d(3.5, 0));
    tmp_goal_points.emplace_back(Eigen::Vector2d(4.7, 2));
    tmp_goal_points.emplace_back(Eigen::Vector2d(4.7, -2));
    // tmp_goal_points.emplace_back(Eigen::Vector2d(3.4, 2));
    // tmp_goal_points.emplace_back(Eigen::Vector2d(3.4, -2));
    for (auto & tmp_goal_point : tmp_goal_points)
    {
        goal_points1.emplace_back(Eigen::Vector3d(tmp_goal_point.x(), tmp_goal_point.y(), 0));
        goal_points1.emplace_back(Eigen::Vector3d(tmp_goal_point.x(), tmp_goal_point.y(), -45/57.3));
        // goal_points1.emplace_back(Eigen::Vector3d(tmp_goal_point.x(), tmp_goal_point.y(), 60/57.3));
        // goal_points.emplace_back(Eigen::Vector3d(tmp_goal_point.x(), tmp_goal_point.y(), -60/57.3));
        goal_points1.emplace_back(Eigen::Vector3d(tmp_goal_point.x(), tmp_goal_point.y(), 45/57.3));
    }

    grid_map::Position3 start_(2.5, 2.3, -90/57.3);
    start_points2.emplace_back(Eigen::Vector3d(start_.x(), start_.y(), start_.z() + 0));
    start_points2.emplace_back(Eigen::Vector3d(start_.x(), start_.y(), start_.z() -45/57.3));
    start_points2.emplace_back(Eigen::Vector3d(start_.x(), start_.y(), start_.z() + 45/57.3));

    vector<Eigen::Vector3d> tmp_goal_points2;
    tmp_goal_points2.emplace_back(Eigen::Vector3d(2.5, -2.2, -90/57.3));
    tmp_goal_points2.emplace_back(Eigen::Vector3d(4.5, -2.2, -90/57.3));
    tmp_goal_points2.emplace_back(Eigen::Vector3d(0.5, -2.2, -90/57.3));
    for (auto & tmp_goal_point : tmp_goal_points2)
    {
        goal_points2.emplace_back(Eigen::Vector3d(tmp_goal_point.x(), tmp_goal_point.y(), tmp_goal_point.z()));
        goal_points2.emplace_back(Eigen::Vector3d(tmp_goal_point.x(), tmp_goal_point.y(), tmp_goal_point.z() - 45/57.3));
        goal_points2.emplace_back(Eigen::Vector3d(tmp_goal_point.x(), tmp_goal_point.y(), tmp_goal_point.z() + 45/57.3));
    }
}

bool variousTerrainPlanner::CheckFeasibleGoalTraditional(grid_map::GridMap & map, Eigen::Vector3d cand_goal, Eigen::Vector3d & goal)
{
    if (local_planner_traditional.getPlannerPtr())
    {
        grid_map::SpiralIterator iterator(map, cand_goal.head(2), (foot_param.x_upper + foot_param.x_button)/2 + 0.1);
        while (!iterator.isPastEnd())
        {
            grid_map::Position position;
            map.getPosition(*iterator, position);
            Eigen::Vector3d goal_point(position.x(), position.y(), cand_goal.z());
            if (local_planner_traditional.isGoalFeasible(goal_point))
            {
                goal = goal_point;
                return true;
            }
            ++iterator;
        }
        return false;
    }
    else
    {
        return false;
    }
}

bool variousTerrainPlanner::CheckFeasibleGoalPropose(grid_map::GridMap & map, Eigen::Vector3d cand_goal, Eigen::Vector3d & goal)
{
    if (local_planner_propose.getPlannerPtr())
    {
        grid_map::SpiralIterator iterator(map, cand_goal.head(2), (foot_param.x_upper + foot_param.x_button)/2 + 0.1);
        while (!iterator.isPastEnd())
        {
            grid_map::Position position;
            map.getPosition(*iterator, position);
            Eigen::Vector3d goal_point(position.x(), position.y(), cand_goal.z());
            if (local_planner_propose.isGoalFeasible(goal_point))
            {
                goal = goal_point;
                return true;
            }
            ++iterator;
        }
        return false;
    }
    else
    {
        return false;
    }
}

bool variousTerrainPlanner::checkFeasibleStartTraditional(grid_map::GridMap & map, Eigen::Vector3d & start, Eigen::Vector3d & left_foot_tra, Eigen::Vector3d & right_foot_tra)
{
    grid_map::SpiralIterator iterator(map, start.head(2), (foot_param.x_upper + foot_param.x_button)/2 + 0.1);
    while (!iterator.isPastEnd())
    {
        grid_map::Position position;
        map.getPosition(*iterator, position);
        // 两种情况均成立，才以此点为终点
        if (local_planner_traditional.isStartFeasible(Eigen::Vector3d(position.x(), position.y(), start.z()), left_foot_tra, right_foot_tra))
        {
            return true;
        }
        ++iterator;
    }
    return false;
}

bool variousTerrainPlanner::checkFeasibleStartPropose(grid_map::GridMap & map, Eigen::Vector3d & start, Eigen::Vector3d & propose_left_foot, Eigen::Vector3d & propose_right_foot)
{
    grid_map::SpiralIterator iterator(map, start.head(2), (foot_param.x_upper + foot_param.x_button)/2 + 0.1);
    while (!iterator.isPastEnd())
    {
        grid_map::Position position;
        map.getPosition(*iterator, position);
        // 两种情况均成立，才以此点为终点
        if (local_planner_propose.isStartFeasible(Eigen::Vector3d(position.x(), position.y(), start.z()), propose_left_foot, propose_right_foot))
        {
            return true;
        }
        ++iterator;
    }
    return false;
}


bool variousTerrainPlanner::getPlannerResultTraditional()
{
    vector<Footstep> steps = local_planner_traditional.getFootsteps();
    if (!steps.empty())
    {
        for (auto & step : steps)
        {
            std::cout << "step: " << step.x << " " << step.y << " " << step.z << " " << step.roll << " " << step.pitch << " " << step.yaw << std::endl;
            // file << "step: " << step.x << " " << step.y << " " << step.z << " " << step.roll << " " << step.pitch << " " << step.yaw << std::endl;
        }
        return true;
    }
    else
    {
        return false;
    }
}

bool variousTerrainPlanner::getPlannerResultPropose()
{
    vector<Footstep> steps = local_planner_propose.getFootsteps();
    if (!steps.empty())
    {
        for (auto & step : steps)
        {
            std::cout << "step: " << step.x << " " << step.y << " " << step.z << " " << step.roll << " " << step.pitch << " " << step.yaw << std::endl;
            // file << "step: " << step.x << " " << step.y << " " << step.z << " " << step.roll << " " << step.pitch << " " << step.yaw << std::endl;
        }
        return true;
    }
    else
    {
        return false;
    }
}


void variousTerrainPlanner::traditionalPlanner() 
{
    std::cout << "Starting traditionalPlanner...\n";
    // file<<"Starting traditionalPlanner..."<<endl;
	try 
	{
		// for (int i = 0; i < 60; ++i) 
		// {
		// 	std::cout << "traditional i = " << i << ", ";
		// 	boost::this_thread::sleep_for(boost::chrono::seconds(1)); // 模拟耗时操作
		// 	boost::this_thread::interruption_point(); // 检查是否被中断
		// }
        auto start = std::chrono::high_resolution_clock::now();
        if (local_planner_traditional.plan())
        {
            auto end = std::chrono::high_resolution_clock::now();
            string re = std::to_string((std::chrono::duration_cast<std::chrono::microseconds>(end - start).count())/1000.0);
            std::cout<<"planning: "<<re<<endl;
            // std::cout<<"planning: "<<re<<endl;
            
            // LOG(INFO)<<local_planner_traditional.timeConsumption();
            re += (" " + local_planner_traditional.time_consume);
            LOG(INFO)<<local_planner_traditional.time_consume;
            result_once.emplace_back(re);
            // filetxt_<<
        }
        else
        {
            // auto end = std::chrono::high_resolution_clock::now();
            // string re = std::to_string((std::chrono::duration_cast<std::chrono::microseconds>(end - start).count())/1000.0);
            // re += " " + local_planner_traditional.timeConsumption();
            // result_once.emplace_back(re);
            result_once.emplace_back("error planning");
        }
        
		
        // auto end = std::chrono::high_resolution_clock::now();
        // // file<<"planning: "<<(std::chrono::duration_cast<std::chrono::microseconds>(end - start).count())/1000.0<<endl;
        // // file<<(std::chrono::duration_cast<std::chrono::microseconds>(end - start).count())/1000.0<<endl;
        // string re = std::to_string((std::chrono::duration_cast<std::chrono::microseconds>(end - start).count())/1000.0);
        // re += " " + local_planner_traditional.timeConsumption();
        // result_once.emplace_back(re);
        // getPlannerResultTraditional();
        // result_once.emplace_back(local_planner_traditional.timeConsumption());
        // file <<local_planner_traditional.timeConsumption()<<endl;
	} 
	catch (const boost::thread_interrupted&) 
	{
		std::cout << "\ntraditionalPlanner interrupted!\n";
        // file << "traditionalPlanner interrupted!"<<endl;
        // file << "ERROR PLANNING"<<endl;
        result_once.emplace_back("error planning");
	}
	std::cout << "traditionalPlanner completed.\n";
    // file << "traditionalPlanner completed."<<endl;
}

void variousTerrainPlanner::proposePlanner() 
{
    std::cout << "Starting proposePlanner...\n";
    // file<<"Starting proposePlanner..."<<endl;
	try 
	{
		// for (int i = 0; i < 60; ++i) 
		// {
		// 	std::cout << "propose i = " << i << ", ";
		// 	boost::this_thread::sleep_for(boost::chrono::seconds(1)); // 模拟耗时操作
		// 	boost::this_thread::interruption_point(); // 检查是否被中断
		// }
        auto start = std::chrono::high_resolution_clock::now();
        if (local_planner_propose.plan())
        {
            auto end = std::chrono::high_resolution_clock::now();
            string re = std::to_string((std::chrono::duration_cast<std::chrono::microseconds>(end - start).count())/1000.0);
            std::cout<<"planning: "<<re<<endl;
            // LOG(INFO)<<local_planner_propose.timeConsumption();
            re += (" " + local_planner_propose.time_consume);
            LOG(INFO)<<local_planner_propose.time_consume;
            result_once.emplace_back(re);
        }
        else
        {
            result_once.emplace_back("error planning");
        }
        
        
		// local_planner_propose.plan();
        // auto end = std::chrono::high_resolution_clock::now();
        // string re = std::to_string((std::chrono::duration_cast<std::chrono::microseconds>(end - start).count())/1000.0);
        // re += " " + local_planner_traditional.timeConsumption();
        // result_once.emplace_back(re);
        // file<<"planning: "<<(std::chrono::duration_cast<std::chrono::microseconds>(end - start).count())/1000.0<<endl;
        // file<<(std::chrono::duration_cast<std::chrono::microseconds>(end - start).count())/1000.0<<endl;
        // getPlannerResultPropose();
        // file <<local_planner_propose.timeConsumption()<<endl;
	} 
	catch (const boost::thread_interrupted&) 
	{
		std::cout << "\nproposePlanner interrupted!\n";
        // file << "proposePlanner interrupted!"<<endl;
        // file << "ERROR PLANNING"<<endl;
        result_once.emplace_back("error planning");
	}
	std::cout << "proposePlanner completed.\n";
    // file << "proposePlanner completed."<<endl;
}

void variousTerrainPlanner::executeTaskWithTimeout(std::function<void()> task, int timeout) 
{
    boost::thread worker(task); // 创建一个线程来运行任务
    if (!worker.try_join_for(boost::chrono::seconds(timeout))) 
	{
        std::cout << "\nTask timed out, interrupting...\n";
        // file << "Task timed out, interrupting..."<<endl;
        worker.interrupt(); // 中断线程
        worker.join(); // 等待线程退出
        std::cout << "Task interrupted and skipped to the next task.\n";
        // file << "Task interrupted and skipped to the next task."<<endl;
    } 
	else 
	{
        std::cout << "Task completed within the time limit.\n";
        // file << "Task completed within the time limit."<<endl;
    }
}
void variousTerrainPlanner::execute()
{	
#ifdef flat_plane
    grid_map::GridMap map({"elevation"});
	map.setFrameId("map");
	map.setGeometry(grid_map::Length(5, 5), 0.02, grid_map::Position(2.5, 0.0));
	ROS_INFO("Created map with size %f x %f m (%i x %i cells).", map.getLength().x(), map.getLength().y(),map.getSize()(0), map.getSize()(1));
    for (grid_map::GridMapIterator it(map); !it.isPastEnd(); ++it) 
	{
		grid_map::Position position;
		map.getPosition(*it, position);
		map.at("elevation", *it) =0;
    }
    
for (size_t i = 0; i < 2; i++)
{
    // std::shared_ptr<AstarHierarchicalFootstepPlannerTraditional> traditional_planner_ptr = std::make_shared<AstarHierarchicalFootstepPlannerTraditional>();
    // traditional_planner_ptr->setCheckParam(checkXupper, checkXButton);
    // local_planner_traditional.setPlanner(traditional_planner_ptr);
    // local_planner_traditional.setFootParam(foot_param);
    // local_planner_traditional.setHipWidth(hip_width);

    

               
    

    std::shared_ptr<AstarHierarchicalFootstepPlannerPropose> propose_planner_ptr = std::make_shared<AstarHierarchicalFootstepPlannerPropose>();
    local_planner_propose.setPlanner(propose_planner_ptr);
    local_planner_propose.setFootParam(foot_param);
    local_planner_propose.setHipWidth(hip_width);

    // local_planner_traditional.mapPrepare(map);
                
    // local_planner_traditional.initial(Eigen::Vector3d(0.3, 0.1, 0), Eigen::Vector3d(0.3, -0.1, 0), 0, Eigen::Vector3d(4, 0, 0));
    const int TIME_LIMIT = 30;
    // executeTaskWithTimeout([this]() { traditionalPlanner(); }, TIME_LIMIT);


    local_planner_propose.mapPrepare(map);
    local_planner_propose.initial(Eigen::Vector3d(0.3, 0.1, 0), Eigen::Vector3d(0.3, -0.1, 0), 0, Eigen::Vector3d(4, 0, 0));
    executeTaskWithTimeout([this]() { proposePlanner(); }, TIME_LIMIT);
}

    
#endif

#ifdef stair
	grid_map::GridMap map({"elevation"});
	map.setFrameId("map");
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
#endif

#ifdef discontinuous_steps
    int test_index_num = 0;
	while (ros::ok() && test_index_num < 3)
	{
        double map_length = 5.0;
        double map_width = 5.0;
        double resolution = 0.01;
        grid_map::GridMap map({"elevation"});
        map.setFrameId("map");
        map.setGeometry(grid_map::Length(map_length, map_width), resolution, grid_map::Position(map_length/2.0, 0));

        // 输入多变的地形,台阶宽度0.06-0.27，间隙0.02-0.17
        std::random_device rd_step; // 随机数种子
        std::mt19937 gen_step(rd_step()); // 随机数生成器
        // std::uniform_int_distribution<> dist_int_step(19, 27);
        std::uniform_int_distribution<> dist_int_step(5, 27);
        // int step_width = dist_int_step(gen_step);

        std::random_device rd_gap; // 随机数种子
        std::mt19937 gen_gap(rd_gap()); // 随机数生成器
        std::uniform_int_distribution<> dist_int_gap(5, 14);
        // std::uniform_int_distribution<> dist_int_gap(2, 14);
        // int gap_width = dist_int_gap(gen_gap);

        double step_elevation = 0.1;
        double gap_elevation = 0.0;
        map.clearAll();
        int index_length = 0;
        vector<int> map_design;
        while (index_length < map.getSize().x())
        {
            int step_index_length = min(dist_int_step(gen_step), map.getSize().x() - index_length);
            
            for (int i_x = 0; i_x < step_index_length; i_x++)
            {
                for(int j_y = 0; j_y < map.getSize().y(); j_y++)
                {
                    if (i_x + index_length < map.getSize().x())
                    {
                        map["elevation"](i_x + index_length, j_y) = step_elevation;
                    }
                }
            }
            map_design.emplace_back(step_index_length);
            index_length = index_length + step_index_length;
            if (index_length >= map.getSize().x())
            {
                break;
            }
            int gap_index_length = min(dist_int_gap(gen_gap), map.getSize().x() - index_length);
            for (int i_x = 0; i_x < gap_index_length; i_x++)
            {
                for(int j_y = 0; j_y < map.getSize().y(); j_y++)
                {
                    if (i_x + index_length < map.getSize().x())
                    {
                        map["elevation"](i_x + index_length, j_y) = gap_elevation;
                    }
                }
            }
            map_design.emplace_back(gap_index_length);
            index_length = index_length + gap_index_length;
        }

        // int fill_step = 0.5/map.getResolution();
        // cv::Mat fill_image = cv::Mat::zeros(map.getSize()(1), map.getSize()(0), CV_8UC1);
        // 填充台阶，以保证起点和终点的可用性
        // for (int i = 0; i < fill_image.cols; i++)
        // {
        //     for (int j = 0; j < fill_step; j++)
        //     {
        //         fill_image.at<uchar>(j, i) = 255;
        //         fill_image.at<uchar>(map.getSize()(0) - j - 1, i) = 255;
        //     }
        // }
        // for (int i = 0; i < fill_image.rows; i++)
        // {
        //     for (int j = 0; j < fill_step; j++)
        //     {
        //         fill_image.at<uchar>(i, j) = 255;
        //         fill_image.at<uchar>(i, map.getSize()(1) - j - 1) = 255;
        //     }
        // }
        // // cv::imshow("fill_image", fill_image);
        // // cv::waitKey(0);
        // for (int i = 0; i < fill_image.rows; i++)
        // {
        //     for (int j = 0; j < fill_image.cols; j++)
        //     {
        //         if (fill_image.at<uchar>(i, j) == 255)
        //         {
        //             map["elevation"](i, j) = 0.1;
        //         }
        //     }
        // }

        grid_map_msgs::GridMap msg;
        grid_map::GridMapRosConverter::toMessage(map, msg);
        map_pub.publish(msg);

        string terrain_design = "map_design: ";
        for (auto i : map_design)
        {
            terrain_design += (std::to_string(i) + " ");
        }
        file<<terrain_design<<std::endl;
        vector<vector<string>> results;
        for (auto & start_point : start_points1)
        {
            for (auto & goal_point : goal_points1)
            {
                result_once.clear();
                // file<<"start point: "<<start_point.transpose()<<", goal_point: "<<goal_point.transpose()<<endl;
                // string start_goal = "start point: " + std::to_string(start_point.x()) + " " + std::to_string(start_point.y()) + ", goal_point: " + std::to_string(goal_point.x()) + " " + std::to_string(goal_point.y());
                std::ostringstream oss;
                oss << std::fixed << std::setprecision(3); // 固定小数点，保留三位小数
                oss << "start point: " << start_point.x() << " " << start_point.y()<<" "<<start_point.z() << ", goal point: " << goal_point.x() << " " << goal_point.y()<< " " << goal_point.z();
                // std::string start_goal = oss.str();
                // file<<oss.str()<<std::endl;
                result_once.emplace_back(oss.str());
                LOG(INFO)<<oss.str();
                std::shared_ptr<AstarHierarchicalFootstepPlannerTraditional> traditional_planner_ptr = std::make_shared<AstarHierarchicalFootstepPlannerTraditional>();
                traditional_planner_ptr->setCheckParam(checkXupper, checkXButton);
                local_planner_traditional.setPlanner(traditional_planner_ptr);
                local_planner_traditional.setFootParam(foot_param);
                local_planner_traditional.setHipWidth(hip_width);

                std::shared_ptr<AstarHierarchicalFootstepPlannerPropose> propose_planner_ptr = std::make_shared<AstarHierarchicalFootstepPlannerPropose>();
                local_planner_propose.setPlanner(propose_planner_ptr);
                local_planner_propose.setFootParam(foot_param);
                local_planner_propose.setHipWidth(hip_width);

                local_planner_traditional.mapPrepare(map);
                local_planner_propose.mapPrepare(map);

                LOG(INFO)<<"mapPrepare finish";

                Eigen::Vector3d goal_point_tra, goal_point_propose;
                Eigen::Vector3d left_foot_tra, right_foot_tra, propose_left_foot, propose_right_foot;

                if (checkFeasibleStartTraditional(map, start_point, left_foot_tra, right_foot_tra))
                {
                    LOG(INFO)<<"Traditional: start is feasible";
                    if (CheckFeasibleGoalTraditional(map, goal_point, goal_point_tra))
                    {
                        LOG(INFO)<<"Traditional: goal is feasible";
                        
                        // file<<"tra stat and goal: "<<left_foot_tra.transpose()<<", "<<right_foot_tra.transpose()<<", "<<goal_point_tra.transpose()<<endl;
                        local_planner_traditional.initial(left_foot_tra, right_foot_tra, 0, goal_point_tra);
                        const int TIME_LIMIT = 180;
                        executeTaskWithTimeout([this]() { traditionalPlanner(); }, TIME_LIMIT);
                    }
                    else
                    {
                        LOG(ERROR)<<"Traditional: goal is not feasible";
                        // file<<"error goal"<<endl;
                        result_once.emplace_back("error goal");
                        // file<<"Traditional: goal is not feasible"<<endl;
                    }
                }
                else
                {
                    LOG(ERROR)<<"Traditional: start is not feasible";
                    // file<<"error start"<<endl;
                    result_once.emplace_back("error start");
                    // file<<"Traditional: start is not feasible"<<endl;
                }
                

                if (checkFeasibleStartPropose(map, start_point, propose_left_foot, propose_right_foot))
                {
                    LOG(INFO)<<"Propose: start is feasible";
                    if (CheckFeasibleGoalPropose(map, goal_point, goal_point_propose))
                    {
                        LOG(INFO)<<"Propose: goal is feasible";
                        // file<<"pro stat and goal: "<<propose_left_foot.transpose()<<", "<<propose_right_foot.transpose()<<", "<<goal_point_propose.transpose()<<endl;
                        local_planner_propose.initial(propose_left_foot, propose_right_foot, 0, goal_point_propose);
                        const int TIME_LIMIT = 180;
                        executeTaskWithTimeout([this]() { proposePlanner(); }, TIME_LIMIT);
                    }
                    else
                    {
                        LOG(ERROR)<<"Propose: goal is not feasible";
                        // file<<"error goal"<<endl;
                        result_once.emplace_back("error goal");
                        // file<<"Propose: goal is not feasible"<<endl;
                    }
                }
                else
                {
                    LOG(ERROR)<<"Propose: start is not feasible";
                    result_once.emplace_back("error start");
                    // file<<"error start"<<endl;
                    // file<<"Propose: start is not feasible"<<endl;
                }
                results.emplace_back(result_once);
            }
        }
        
        
        for (auto & start_point : start_points2)
        {
            LOG(INFO)<<"start point: "<<start_point.transpose();
            for (auto & goal_point : goal_points2)
            {
                result_once.clear();
                // file<<"start point: "<<start_point.transpose()<<", goal_point: "<<goal_point.transpose()<<endl;
                std::ostringstream oss;
                oss << std::fixed << std::setprecision(3); // 固定小数点，保留三位小数
                oss << "start point: " << start_point.x() << " " << start_point.y() << " " << start_point.z() << ", goal point: " << goal_point.x() << " " << goal_point.y()<< " " << goal_point.z();
                // std::string start_goal = oss.str();
                // file<<oss.str()<<std::endl;
                result_once.emplace_back(oss.str());
                std::shared_ptr<AstarHierarchicalFootstepPlannerTraditional> traditional_planner_ptr = std::make_shared<AstarHierarchicalFootstepPlannerTraditional>();
                traditional_planner_ptr->setCheckParam(checkXupper, checkXButton);
                local_planner_traditional.setPlanner(traditional_planner_ptr);
                local_planner_traditional.setFootParam(foot_param);
                local_planner_traditional.setHipWidth(hip_width);

                std::shared_ptr<AstarHierarchicalFootstepPlannerPropose> propose_planner_ptr = std::make_shared<AstarHierarchicalFootstepPlannerPropose>();
                local_planner_propose.setPlanner(propose_planner_ptr);
                local_planner_propose.setFootParam(foot_param);
                local_planner_propose.setHipWidth(hip_width);

                local_planner_traditional.mapPrepare(map);
                local_planner_propose.mapPrepare(map);
                LOG(INFO)<<"mapPrepare finish";

                Eigen::Vector3d goal_point_tra, goal_point_propose;
                Eigen::Vector3d left_foot_tra, right_foot_tra, propose_left_foot, propose_right_foot;

                if (checkFeasibleStartTraditional(map, start_point, left_foot_tra, right_foot_tra))
                {
                    LOG(INFO)<<"Traditional: start is feasible";
                    if (CheckFeasibleGoalTraditional(map, goal_point, goal_point_tra))
                    {
                        LOG(INFO)<<"Traditional: goal is feasible";
                        // file<<"tra stat and goal: "<<left_foot_tra.transpose()<<", "<<right_foot_tra.transpose()<<", "<<goal_point_tra.transpose()<<endl;
                        local_planner_traditional.initial(left_foot_tra, right_foot_tra, 0, goal_point_tra);
                        const int TIME_LIMIT = 180;
                        executeTaskWithTimeout([this]() { traditionalPlanner(); }, TIME_LIMIT);
                    }
                    else
                    {
                        LOG(ERROR)<<"Traditional: goal is not feasible";
                        // file<<"error goal"<<endl;
                        result_once.emplace_back("error goal");
                        // file<<"Traditional: goal is not feasible"<<endl;
                    }
                }
                else
                {
                    LOG(ERROR)<<"Traditional: start is not feasible";
                    // file<<"error start"<<endl;
                    result_once.emplace_back("error start");
                    // file<<"Traditional: start is not feasible"<<endl;
                }
                

                if (checkFeasibleStartPropose(map, start_point, propose_left_foot, propose_right_foot))
                {
                    LOG(INFO)<<"Propose: start is feasible";
                    if (CheckFeasibleGoalPropose(map, goal_point, goal_point_propose))
                    {
                        LOG(INFO)<<"Propose: goal is feasible";
                        // file<<"pro stat and goal: "<<propose_left_foot.transpose()<<", "<<propose_right_foot.transpose()<<", "<<goal_point_propose.transpose()<<endl;
                        local_planner_propose.initial(propose_left_foot, propose_right_foot, 0, goal_point_propose);
                        const int TIME_LIMIT = 180;
                        executeTaskWithTimeout([this]() { proposePlanner(); }, TIME_LIMIT);
                    }
                    else
                    {
                        LOG(ERROR)<<"Propose: goal is not feasible";
                        // file<<"Propose: goal is not feasible"<<endl;
                        // file<<"error goal"<<endl;
                        result_once.emplace_back("error goal");
                    }
                }
                else
                {
                    LOG(ERROR)<<"Propose: start is not feasible";
                    result_once.emplace_back("error start");
                    // file<<"error start"<<endl;
                    // file<<"Propose: start is not feasible"<<endl;
                }
                results.emplace_back(result_once);
            }
        }
        
        LOG(INFO)<<"finish one test";
        const size_t chunk_size = 9;
        while (!results.empty())
        {
            std::vector<vector<string>> sub_vector;

            // 将最多 chunk_size 个元素从 original 移动到 sub_vector
            for (size_t i = 0; i < chunk_size && !results.empty(); ++i) 
            {
                sub_vector.push_back(results.front());
                results.erase(results.begin());
            }
            int cols = sub_vector.at(0).size();

            for (int i = 0; i < cols; i++)
            {
                for (const auto& element : sub_vector)
                {
                    file<<element.at(i)<<"\t";
                }
                file<<endl;
            }
        }
        file<<endl;


        test_index_num++;
		
	}
#endif

#ifdef single_terrain_for_test
// single terrain for test
// terrain: 19 11 20 2 22 8 26 7 23 5 23 10 25 6 27 13 24 4 26 4 25 3 23 12 20 9 21 5 22 13 27 6 25
// start left: 0.275 0.105     0,  0.275 -0.095      0, 3.455 0.005     0
    double step_elevation = 0.1;
    double gap_elevation = 0.0;
    double map_length = 5.0;
    double map_width = 5.0;
	double resolution = 0.01;
	grid_map::GridMap map({"elevation"});
    map.setFrameId("map");
    map.setGeometry(grid_map::Length(map_length, map_width), resolution, grid_map::Position(map_length/2.0, 0));
    vector<int> map_design = {5, 13, 12, 6, 24, 14, 22, 5, 13, 5, 20, 8, 17, 13, 21, 8, 16, 11, 10, 10, 14, 13, 8, 6, 26, 9, 10, 12, 24, 9, 25, 9, 14, 11, 27, 12, 18 };
    int insert_index = 0;
    for (int i = 0; i < map_design.size(); i++)
    {
        if (i%2 == 0)
        {
            for (int i_x = 0; i_x < map_design.at(i); i_x++)
            {
                for(int j_y = 0; j_y < map.getSize().y(); j_y++)
                {
                    map["elevation"](i_x + insert_index, j_y) = step_elevation;
                }
            }
        }
        else
        {
            for (int i_x = 0; i_x < map_design.at(i); i_x++)
            {
                for(int j_y = 0; j_y < map.getSize().y(); j_y++)
                {
                    map["elevation"](i_x + insert_index, j_y) = gap_elevation;
                }
            }
        }
        insert_index = insert_index + map_design.at(i);
    }
    grid_map_msgs::GridMap msg;
    grid_map::GridMapRosConverter::toMessage(map, msg);

    ros::Rate loop_rate(10);
    while (ros::ok())
    {
        map_pub.publish(msg);
        loop_rate.sleep();
    }
    

    
    // std::shared_ptr<AstarHierarchicalFootstepPlannerTraditional> traditional_planner_ptr = std::make_shared<AstarHierarchicalFootstepPlannerTraditional>();
    // traditional_planner_ptr->setCheckParam(checkXupper, checkXButton);
    // local_planner_traditional.setPlanner(traditional_planner_ptr);
    // local_planner_traditional.setFootParam(foot_param);
    // local_planner_traditional.setHipWidth(hip_width);

    // local_planner_traditional.mapPrepare(map);
    // Eigen::Vector3d left_foot_tra(0.275, 0.105, 0);
    // Eigen::Vector3d right_foot_tra(0.275, -0.095, 0);
    // Eigen::Vector3d goal_point_tra(3.455, 0.005, 0);
    // local_planner_traditional.initial(left_foot_tra, right_foot_tra, 0, goal_point_tra); 
    // local_planner_traditional.plan();

    // std::shared_ptr<AstarHierarchicalFootstepPlannerPropose> propose_planner_ptr = std::make_shared<AstarHierarchicalFootstepPlannerPropose>();
    // local_planner_propose.setPlanner(propose_planner_ptr);
    // local_planner_propose.setFootParam(foot_param);
    // local_planner_propose.setHipWidth(hip_width);
    // local_planner_propose.mapPrepare(map);
    // local_planner_propose.initial(Eigen::Vector3d(0.245, 0.085, 0.1), Eigen::Vector3d(0.245, -0.105, 0.1), 0, Eigen::Vector3d(4.495, 0, 0));
    // const int TIME_LIMIT = 30;
    // executeTaskWithTimeout([this]() { proposePlanner(); }, TIME_LIMIT);
#endif

#ifdef WAVE
    // test_index_num = 0;
    int test_index_num = 0;
	while (ros::ok() && test_index_num < 10)
	{
        double map_length = 5.0;
        double map_width = 5.0;
        double resolution = 0.01;
        grid_map::GridMap map({"elevation"});
        map.setFrameId("map");
        map.setGeometry(grid_map::Length(map_length, map_width), resolution, grid_map::Position(map_length/2.0, 0));

        double step_elevation = 0.1;
        double gap_elevation = 0.0;
        cv::Mat image = cv::Mat::zeros(map.getSize().x(), map.getSize().y(), CV_8UC1);
        int amplitude = 20;  // 振幅（波的高度）
        int frequency = 100;  // 波浪的频率（每个周期内的点数）
        image = cv::Mat::zeros(map.getSize().x(), map.getSize().y(), CV_8UC1);


        vector<int> map_design;  
        int offsetY = 0;
        while (offsetY < map.getSize().x())
        {
            std::random_device rd_step; // 随机数种子
            std::mt19937 gen_step(rd_step()); // 随机数生成器
            // std::uniform_int_distribution<> dist_int_step(19, 27);
            std::uniform_int_distribution<> dist_int_step(5, 27);
            // int step_width = dist_int_step(gen_step);

            std::random_device rd_gap; // 随机数种子
            std::mt19937 gen_gap(rd_gap()); // 随机数生成器
            std::uniform_int_distribution<> dist_int_gap(5, 14);

            int step_index_length = dist_int_step(gen_step);
            int gap_index_length = dist_int_gap(gen_gap);

            int lineGap = step_index_length + gap_index_length;      // 每条波浪线之间的垂直间隔
            int thickness = step_index_length;  

            std::random_device rd; // 随机数种子
            std::mt19937 gen(rd()); // 随机数生成器

            // 整数范围
            std::uniform_int_distribution<> dist_int(0, frequency/2); // 1到10
            int xOffset = dist_int(gen); 

            for (int x = 0; x < map.getSize().y(); ++x)
            {
                int y = static_cast<int>(offsetY + amplitude * std::sin(2 * CV_PI * (x + xOffset) / frequency));
                if (x > 0) 
                {
                    // 连接相邻的点，生成连续的波浪线
                    cv::line(image, 
                            cv::Point(x - 1, static_cast<int>(offsetY + amplitude * std::sin(2 * CV_PI * ((x - 1) + xOffset) / frequency))),
                            cv::Point(x, y), 
                            cv::Scalar(255), // 白色波浪线
                            thickness);               // 波浪线的厚度
                }
            }
            offsetY += lineGap;

            map_design.emplace_back(step_index_length);
            map_design.emplace_back(gap_index_length);
            map_design.emplace_back(xOffset);
        }


        for (int x_i = 0; x_i < image.rows; x_i++)
        {
            for (int y_j = 0; y_j < image.cols; y_j++)
            {
                if (image.at<uchar>(x_i, y_j) == 255)
                {
                    map["elevation"](x_i, y_j) = step_elevation;
                }
                else
                {
                    map["elevation"](x_i, y_j) = gap_elevation;
                }
            }
        }

        // int fill_step = 0.5/map.getResolution();
        // cv::Mat fill_image = cv::Mat::zeros(map.getSize()(1), map.getSize()(0), CV_8UC1);
        // // 填充台阶，以保证起点和终点的可用性
        // for (int i = 0; i < fill_image.cols; i++)
        // {
        //     for (int j = 0; j < fill_step; j++)
        //     {
        //         fill_image.at<uchar>(j, i) = 255;
        //         fill_image.at<uchar>(map.getSize()(0) - j - 1, i) = 255;
        //     }
        // }
        // for (int i = 0; i < fill_image.rows; i++)
        // {
        //     for (int j = 0; j < fill_step; j++)
        //     {
        //         fill_image.at<uchar>(i, j) = 255;
        //         fill_image.at<uchar>(i, map.getSize()(1) - j - 1) = 255;
        //     }
        // }
        // // cv::imshow("fill_image", fill_image);
        // // cv::waitKey(0);
        // for (int i = 0; i < fill_image.rows; i++)
        // {
        //     for (int j = 0; j < fill_image.cols; j++)
        //     {
        //         if (fill_image.at<uchar>(i, j) == 255)
        //         {
        //             map["elevation"](i, j) = 0.1;
        //         }
        //     }
        // }
        grid_map_msgs::GridMap msg;
            grid_map::GridMapRosConverter::toMessage(map, msg);
            map_pub.publish(msg);
        
        
        

        

        string terrain_design = "map_design: ";
        for (auto i : map_design)
        {
            terrain_design += (std::to_string(i) + " ");
        }
        file<<terrain_design<<std::endl;

        vector<vector<string>> results;
        for (auto & start_point : start_points1)
        {
            for (auto & goal_point : goal_points1)
            {
                result_once.clear();
                // file<<"start point: "<<start_point.transpose()<<", goal_point: "<<goal_point.transpose()<<endl;
                // string start_goal = "start point: " + std::to_string(start_point.x()) + " " + std::to_string(start_point.y()) + ", goal_point: " + std::to_string(goal_point.x()) + " " + std::to_string(goal_point.y());
                std::ostringstream oss;
                oss << std::fixed << std::setprecision(3); // 固定小数点，保留三位小数
                oss << "start point: " << start_point.x() << " " << start_point.y()<<" "<<start_point.z() << ", goal point: " << goal_point.x() << " " << goal_point.y()<< " " << goal_point.z();
                // std::string start_goal = oss.str();
                // file<<oss.str()<<std::endl;
                result_once.emplace_back(oss.str());
                LOG(INFO)<<oss.str();
                std::shared_ptr<AstarHierarchicalFootstepPlannerTraditional> traditional_planner_ptr = std::make_shared<AstarHierarchicalFootstepPlannerTraditional>();
                traditional_planner_ptr->setCheckParam(checkXupper, checkXButton);
                local_planner_traditional.setPlanner(traditional_planner_ptr);
                local_planner_traditional.setFootParam(foot_param);
                local_planner_traditional.setHipWidth(hip_width);

                std::shared_ptr<AstarHierarchicalFootstepPlannerPropose> propose_planner_ptr = std::make_shared<AstarHierarchicalFootstepPlannerPropose>();
                local_planner_propose.setPlanner(propose_planner_ptr);
                local_planner_propose.setFootParam(foot_param);
                local_planner_propose.setHipWidth(hip_width);

                local_planner_traditional.mapPrepare(map);
                local_planner_propose.mapPrepare(map);

                LOG(INFO)<<"mapPrepare finish";

                Eigen::Vector3d goal_point_tra, goal_point_propose;
                Eigen::Vector3d left_foot_tra, right_foot_tra, propose_left_foot, propose_right_foot;

                if (checkFeasibleStartTraditional(map, start_point, left_foot_tra, right_foot_tra))
                {
                    LOG(INFO)<<"Traditional: start is feasible";
                    if (CheckFeasibleGoalTraditional(map, goal_point, goal_point_tra))
                    {
                        LOG(INFO)<<"Traditional: goal is feasible";
                        
                        // file<<"tra stat and goal: "<<left_foot_tra.transpose()<<", "<<right_foot_tra.transpose()<<", "<<goal_point_tra.transpose()<<endl;
                        local_planner_traditional.initial(left_foot_tra, right_foot_tra, 0, goal_point_tra);
                        const int TIME_LIMIT = 180;
                        executeTaskWithTimeout([this]() { traditionalPlanner(); }, TIME_LIMIT);
                    }
                    else
                    {
                        LOG(ERROR)<<"Traditional: goal is not feasible";
                        // file<<"error goal"<<endl;
                        result_once.emplace_back("error goal");
                        // file<<"Traditional: goal is not feasible"<<endl;
                    }
                }
                else
                {
                    LOG(ERROR)<<"Traditional: start is not feasible";
                    // file<<"error start"<<endl;
                    result_once.emplace_back("error start");
                    // file<<"Traditional: start is not feasible"<<endl;
                }
                

                if (checkFeasibleStartPropose(map, start_point, propose_left_foot, propose_right_foot))
                {
                    LOG(INFO)<<"Propose: start is feasible";
                    if (CheckFeasibleGoalPropose(map, goal_point, goal_point_propose))
                    {
                        LOG(INFO)<<"Propose: goal is feasible";
                        // file<<"pro stat and goal: "<<propose_left_foot.transpose()<<", "<<propose_right_foot.transpose()<<", "<<goal_point_propose.transpose()<<endl;
                        local_planner_propose.initial(propose_left_foot, propose_right_foot, 0, goal_point_propose);
                        const int TIME_LIMIT = 180;
                        executeTaskWithTimeout([this]() { proposePlanner(); }, TIME_LIMIT);
                    }
                    else
                    {
                        LOG(ERROR)<<"Propose: goal is not feasible";
                        // file<<"error goal"<<endl;
                        result_once.emplace_back("error goal");
                        // file<<"Propose: goal is not feasible"<<endl;
                    }
                }
                else
                {
                    LOG(ERROR)<<"Propose: start is not feasible";
                    result_once.emplace_back("error start");
                    // file<<"error start"<<endl;
                    // file<<"Propose: start is not feasible"<<endl;
                }
                results.emplace_back(result_once);
            }
        }
        
        
        for (auto & start_point : start_points2)
        {
            LOG(INFO)<<"start point: "<<start_point.transpose();
            for (auto & goal_point : goal_points2)
            {
                result_once.clear();
                // file<<"start point: "<<start_point.transpose()<<", goal_point: "<<goal_point.transpose()<<endl;
                std::ostringstream oss;
                oss << std::fixed << std::setprecision(3); // 固定小数点，保留三位小数
                oss << "start point: " << start_point.x() << " " << start_point.y() << " " << start_point.z() << ", goal point: " << goal_point.x() << " " << goal_point.y()<< " " << goal_point.z();
                // std::string start_goal = oss.str();
                // file<<oss.str()<<std::endl;
                result_once.emplace_back(oss.str());
                std::shared_ptr<AstarHierarchicalFootstepPlannerTraditional> traditional_planner_ptr = std::make_shared<AstarHierarchicalFootstepPlannerTraditional>();
                traditional_planner_ptr->setCheckParam(checkXupper, checkXButton);
                local_planner_traditional.setPlanner(traditional_planner_ptr);
                local_planner_traditional.setFootParam(foot_param);
                local_planner_traditional.setHipWidth(hip_width);

                std::shared_ptr<AstarHierarchicalFootstepPlannerPropose> propose_planner_ptr = std::make_shared<AstarHierarchicalFootstepPlannerPropose>();
                local_planner_propose.setPlanner(propose_planner_ptr);
                local_planner_propose.setFootParam(foot_param);
                local_planner_propose.setHipWidth(hip_width);

                local_planner_traditional.mapPrepare(map);
                local_planner_propose.mapPrepare(map);
                LOG(INFO)<<"mapPrepare finish";

                Eigen::Vector3d goal_point_tra, goal_point_propose;
                Eigen::Vector3d left_foot_tra, right_foot_tra, propose_left_foot, propose_right_foot;

                if (checkFeasibleStartTraditional(map, start_point, left_foot_tra, right_foot_tra))
                {
                    LOG(INFO)<<"Traditional: start is feasible";
                    if (CheckFeasibleGoalTraditional(map, goal_point, goal_point_tra))
                    {
                        LOG(INFO)<<"Traditional: goal is feasible";
                        // file<<"tra stat and goal: "<<left_foot_tra.transpose()<<", "<<right_foot_tra.transpose()<<", "<<goal_point_tra.transpose()<<endl;
                        local_planner_traditional.initial(left_foot_tra, right_foot_tra, 0, goal_point_tra);
                        const int TIME_LIMIT = 180;
                        executeTaskWithTimeout([this]() { traditionalPlanner(); }, TIME_LIMIT);
                    }
                    else
                    {
                        LOG(ERROR)<<"Traditional: goal is not feasible";
                        // file<<"error goal"<<endl;
                        result_once.emplace_back("error goal");
                        // file<<"Traditional: goal is not feasible"<<endl;
                    }
                }
                else
                {
                    LOG(ERROR)<<"Traditional: start is not feasible";
                    // file<<"error start"<<endl;
                    result_once.emplace_back("error start");
                    // file<<"Traditional: start is not feasible"<<endl;
                }
                

                if (checkFeasibleStartPropose(map, start_point, propose_left_foot, propose_right_foot))
                {
                    LOG(INFO)<<"Propose: start is feasible";
                    if (CheckFeasibleGoalPropose(map, goal_point, goal_point_propose))
                    {
                        LOG(INFO)<<"Propose: goal is feasible";
                        // file<<"pro stat and goal: "<<propose_left_foot.transpose()<<", "<<propose_right_foot.transpose()<<", "<<goal_point_propose.transpose()<<endl;
                        local_planner_propose.initial(propose_left_foot, propose_right_foot, 0, goal_point_propose);
                        const int TIME_LIMIT = 180;
                        executeTaskWithTimeout([this]() { proposePlanner(); }, TIME_LIMIT);
                    }
                    else
                    {
                        LOG(ERROR)<<"Propose: goal is not feasible";
                        // file<<"Propose: goal is not feasible"<<endl;
                        // file<<"error goal"<<endl;
                        result_once.emplace_back("error goal");
                    }
                }
                else
                {
                    LOG(ERROR)<<"Propose: start is not feasible";
                    result_once.emplace_back("error start");
                    // file<<"error start"<<endl;
                    // file<<"Propose: start is not feasible"<<endl;
                }
                results.emplace_back(result_once);
            }
        }
        
        LOG(INFO)<<"finish one test";
        const size_t chunk_size = 9;
        while (!results.empty())
        {
            std::vector<vector<string>> sub_vector;

            // 将最多 chunk_size 个元素从 original 移动到 sub_vector
            for (size_t i = 0; i < chunk_size && !results.empty(); ++i) 
            {
                sub_vector.push_back(results.front());
                results.erase(results.begin());
            }
            int cols = sub_vector.at(0).size();

            for (int i = 0; i < cols; i++)
            {
                for (const auto& element : sub_vector)
                {
                    file<<element.at(i)<<"\t";
                }
                file<<endl;
            }
        }
        file<<endl;


        test_index_num++;
	}
#endif

#ifdef Map_from_PCD
    pcl::PointCloud<pcl::PointXYZ> cloud;
    pcl::io::loadPCDFile("/home/lichao/TCDS/src/bag_to_pcd/data/cloud_0.pcd", cloud);
    LOG(INFO)<<"load pcd file"<< cloud.size();

    pcl::VoxelGrid<pcl::PointXYZ> voxel_filter;
    voxel_filter.setInputCloud(cloud.makeShared());

    float leaf_size = 0.005f; // 体素尺寸（单位：米）
    voxel_filter.setLeafSize(leaf_size, leaf_size, leaf_size);

    pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    voxel_filter.filter(*filtered_cloud);
    LOG(INFO)<<"filter pcd file"<< filtered_cloud->size();

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
    sor.setInputCloud(filtered_cloud);                 // 设置输入点云
    sor.setMeanK(10);                         // 设置每个点的邻居点数
    sor.setStddevMulThresh(1.0);              // 设置标准差倍数的阈值
    sor.filter(*cloud_filtered);  

    pcl::io::savePCDFileASCII("/home/lichao/TCDS/src/pip_line/data/cloud.pcd", *cloud_filtered);
    Eigen::Matrix4d T_install_depth = Eigen::Matrix4d::Identity();
    T_install_depth(1, 3) = -0.001;
    T_install_depth(2, 3) = 0.026 - 0.0045;
    Eigen::Matrix4d T_hole_install = Eigen::Matrix4d::Identity();
    Eigen::Matrix3d R;
    R<<- 0.669130606, 0, 0.743144825,
        0, 1, 0,
        - 0.743144825, 0, -0.669130606;
    T_hole_install.block<3,3>(0,0) = R;
    T_hole_install(0, 3) = 0.07025;
    T_hole_install(2, 3) = 0.00424;
    Eigen::Matrix4d T_base_hole = Eigen::Matrix4d::Identity();
    T_base_hole(0, 3) = 0.05675;
    T_base_hole(2, 3) = 0.49123;
    Eigen::Matrix4d T_world_base = Eigen::Matrix4d::Identity();
    // 一定要注意根据实际高度调节，控制端反馈
    T_world_base.block<3,1>(0,3) = Eigen::Vector3d(0, 0, 0.67 + 0.08);
    Eigen::Matrix4d T_world_camera = T_world_base * T_base_hole * T_hole_install * T_install_depth;

    pcl::PointCloud<pcl::PointXYZ> need_points;
    for (auto & p : cloud_filtered->points)
    {
        // if (p.x > -1.5 && p.x < 3.5)
        // {
        //     if (p.y > -3 && p.y < 9)
        //     {
        //         // need_points.emplace_back(pcl::PointXYZ(p.x + 1.5, p.y - 3, 9 - p.z));
        //         need_points.emplace_back(p);
        //     }
        // }
        Eigen::Vector3d p_world(p.x, p.y, p.z);
        Eigen::Vector3d p_camera = T_world_camera.block<3,3>(0,0) * p_world + T_world_camera.block<3,1>(0,3);
        need_points.emplace_back(pcl::PointXYZ(p_camera(0), p_camera(1), p_camera(2)));
        // if (p_camera(2) > 0.0)
        // {
        //     need_points.emplace_back(pcl::PointXYZ(p_camera(0), p_camera(1), p_camera(2)));
        // }
    }
    LOG(INFO)<<"filter pcd file"<< need_points.size();

    double map_length = 1.6;
    double map_width = 2.0;
    double resolution =0.01;
    grid_map::GridMap map({"elevation"});
    map.setFrameId("map");
    map.setGeometry(grid_map::Length(map_length, map_width), resolution, grid_map::Position(map_length/2.0, 0));

    for (auto & p : need_points.points)
    {
        if (map.isInside(grid_map::Position(p.x, p.y)))
        {
            grid_map::Index index;
            map.getIndex(grid_map::Position(p.x, p.y), index);
            map.at("elevation", index) = p.z;
        }
    }

    for (int i = 118; i < map.getSize().x(); i++)
    {
        for (int j = 50; j < map.getSize().y()-50; j++)
        {
            map.at("elevation", grid_map::Index(i, j)) = 0.05;
        }
    }
    

    // start: position: 
    //   x: 2.2664873600006104
    //   y: -0.0054368977434933186
    //   z: 0.0
    // orientation: 
    //   x: 0.0
    //   y: 0.0
    //   z: -0.6860311125736688
    //   w: 0.7275722043762627

    // goal1 : 
//     position: 
//     x: 3.45186185836792
//     y: -4.003772258758545
//     z: 0.0
//   orientation: 
//     x: 0.0
//     y: 0.0
//     z: -0.5775723631826717
//     w: 0.816339491441878

// goal2 :
// position: 
//     x: 1.5601766109466553
//     y: -4.574963092803955
//     z: 0.0
//   orientation: 
//     x: 0.0
//     y: 0.0
//     z: -0.8506509125327667
//     w: 0.5257309435511393
    // Eigen::Vector3d start(2.2664873600006104, -0.0054368977434933186, 0.0);
    // Eigen::Vector3d goal1(3.45186185836792, -4.003772258758545, 0.0);
    // Eigen::Vector3d goal2(1.5601766109466553, -4.574963092803955, 0.0);
    // tf2::Quaternion quaternion_start(0.0, 0.0, -0.6860311125736688, 0.7275722043762627);
    // tf2::Quaternion quaternion_goal1(0.0, 0.0, -0.5775723631826717, 0.816339491441878);
    // tf2::Quaternion quaternion_goal2(0.0, 0.0, -0.8506509125327667, 0.5257309435511393);
    // start.z() = tf2::getYaw(quaternion_start);
    // goal1.z() = tf2::getYaw(quaternion_goal1);
    // goal2.z() = tf2::getYaw(quaternion_goal2);
    
    std::shared_ptr<AstarHierarchicalFootstepPlannerTraditional> traditional_planner_ptr = std::make_shared<AstarHierarchicalFootstepPlannerTraditional>();
    traditional_planner_ptr->setCheckParam(checkXupper, checkXButton);
    local_planner_traditional.setPlanner(traditional_planner_ptr);
    local_planner_traditional.setFootParam(foot_param);
    local_planner_traditional.setHipWidth(hip_width);

    std::shared_ptr<AstarHierarchicalFootstepPlannerPropose> propose_planner_ptr = std::make_shared<AstarHierarchicalFootstepPlannerPropose>();
    local_planner_propose.setPlanner(propose_planner_ptr);
    local_planner_propose.setFootParam(foot_param);
    local_planner_propose.setHipWidth(hip_width);

    local_planner_traditional.mapPrepare(map);
    local_planner_propose.mapPrepare(map);

    // Eigen::Vector3d left_foot_tra, right_foot_tra;
    // if (checkFeasibleStartTraditional(map, start, left_foot_tra, right_foot_tra))
    // {
    //     local_planner_traditional.initial(left_foot_tra, right_foot_tra, 0, goal1);
    //     if (local_planner_traditional.plan())
    //     {
    //         LOG(INFO)<<"traditional plan success";
    //     }
    // }
    
    
    ros::Rate loop_rate(2);
    while (ros::ok())
    {
        grid_map_msgs::GridMap msg;
        grid_map::GridMapRosConverter::toMessage(map, msg);
        map_pub.publish(msg);
        loop_rate.sleep(); 
    }
    
    

    // pcl::PassThrough<pcl::PointXYZ> pass_filter;
    // pass_filter.setInputCloud(filtered_cloud);
    // // pass_filter.setFilterFieldName("x");        // 选择 z 轴
    // // pass_filter.setFilterLimits(-1.5, 3.5);      // 设置 z 值范围在 [0.0, 1.0]

    // pass_filter.setFilterFieldName("y");
    // pass_filter.setFilterLimits(-2.5, 9);

    // // 可选：保留范围外的点（默认为 false）
    // // pass_filter.getFilterLimitsNegative(false);
    // pass_filter.setNegative(false);

    // // 执行过滤
    // pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud2(new pcl::PointCloud<pcl::PointXYZ>);
    // pass_filter.filter(*filtered_cloud2);
    // LOG(INFO)<<"pass through pcd file"<< filtered_cloud2->size();

    
#endif

}



variousTerrainPlanner::~variousTerrainPlanner()
{
    file.close();
}