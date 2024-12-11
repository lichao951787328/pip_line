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
// #define discontinuous_steps
#define WAVE
// #define flat_plane
// #define single_terrain_for_test
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
    

	grid_map::Position start(0.4, 0);
    start_points1.emplace_back(Eigen::Vector3d(start.x(), start.y(), 0));
    start_points1.emplace_back(Eigen::Vector3d(start.x(), start.y(), -45/57.3));
    start_points1.emplace_back(Eigen::Vector3d(start.x(), start.y(), 45/57.3));

    vector<Eigen::Vector2d> tmp_goal_points;
    tmp_goal_points.emplace_back(Eigen::Vector2d(4.5, 0));
    // tmp_goal_points.emplace_back(Eigen::Vector2d(3.5, 0));
    tmp_goal_points.emplace_back(Eigen::Vector2d(4.4, 2));
    tmp_goal_points.emplace_back(Eigen::Vector2d(4.4, -2));
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

    grid_map::Position3 start_(2.5, 2.0, -90/57.3);
    start_points2.emplace_back(Eigen::Vector3d(start_.x(), start_.y(), start_.z() + 0));
    start_points2.emplace_back(Eigen::Vector3d(start_.x(), start_.y(), start_.z() -45/57.3));
    start_points2.emplace_back(Eigen::Vector3d(start_.x(), start_.y(), start_.z() + 45/57.3));

    vector<Eigen::Vector3d> tmp_goal_points2;
    tmp_goal_points2.emplace_back(Eigen::Vector3d(2.5, -2, -90/57.3));
    tmp_goal_points2.emplace_back(Eigen::Vector3d(4.5, -2, -90/57.3));
    tmp_goal_points2.emplace_back(Eigen::Vector3d(0.5, -2, -90/57.3));
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
        grid_map::SpiralIterator iterator(map, cand_goal.head(2), (foot_param.x_upper + foot_param.x_button)/2);
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
        grid_map::SpiralIterator iterator(map, cand_goal.head(2), (foot_param.x_upper + foot_param.x_button)/2);
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
    grid_map::SpiralIterator iterator(map, start.head(2), (foot_param.x_upper + foot_param.x_button)/2);
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
    grid_map::SpiralIterator iterator(map, start.head(2), (foot_param.x_upper + foot_param.x_button)/2);
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
	while (ros::ok() && test_index_num < 6)
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
        std::uniform_int_distribution<> dist_int_step(20, 30);
        // int step_width = dist_int_step(gen_step);

        std::random_device rd_gap; // 随机数种子
        std::mt19937 gen_gap(rd_gap()); // 随机数生成器
        std::uniform_int_distribution<> dist_int_gap(5, 12);
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
        grid_map_msgs::GridMap msg;
        grid_map::GridMapRosConverter::toMessage(map, msg);
        map_pub.publish(msg);

        // file<<"map_design: ";
        // for (auto i : map_design)
        // {
            // file<<i<<" ";
        // }
        // file<<endl;

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
		// if (checkFeasibleStart(map, left_foot_tra, left_right_tra, propose_left_foot, propose_right_foot))
		// {
		// 	if (CheckFeasibleGoal(map, goal_points))
		// 	{
		// 		LOG(INFO)<<left_foot_tra.transpose();
		// 		LOG(INFO)<<left_right_tra.transpose();
		// 		LOG(INFO)<<propose_left_foot.transpose();
		// 		LOG(INFO)<<propose_right_foot.transpose();
		// 		local_planner_traditional.initial(left_foot_tra, left_right_tra, 0, goal_points.at(0));
		// 		local_planner_propose.initial(propose_left_foot, propose_right_foot, 0, goal_points.at(0));
		// 		const int TIME_LIMIT = 60;
		// 		executeTaskWithTimeout([this]() { proposePlanner(); }, TIME_LIMIT);
		// 		executeTaskWithTimeout([this]() { traditionalPlanner(); }, TIME_LIMIT);
		// 	}
		// 	else
		// 	{
		// 		LOG(ERROR)<<"goal point is not feasible";
		// 	}
		// }
		// else
		// {
		// 	LOG(ERROR)<<"start point is not feasible";
		// }
		
		// break;
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

        
        grid_map_msgs::GridMap msg;
        grid_map::GridMapRosConverter::toMessage(map, msg);
        map_pub.publish(msg);

        // file<<"map_design: ";
        // for (auto i : map_design)
        // {
            // file<<i<<" ";
        // }
        // file<<endl;

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
}



variousTerrainPlanner::~variousTerrainPlanner()
{
    file.close();
}