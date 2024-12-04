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
#define discontinuous_steps
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
}

bool variousTerrainPlanner::CheckFeasibleGoalTraditional(grid_map::GridMap & map, Eigen::Vector2d cand_goal, Eigen::Vector3d & goal)
{
    if (local_planner_traditional.getPlannerPtr())
    {
        grid_map::SpiralIterator iterator(map, cand_goal, (foot_param.x_upper + foot_param.x_button)/2);
        while (!iterator.isPastEnd())
        {
            grid_map::Position position;
            map.getPosition(*iterator, position);
            vector<Eigen::Vector3d> iter_goal_points;
            
            // 0度
            iter_goal_points.emplace_back(Eigen::Vector3d(position.x(), position.y(), 0));

            // 5度
            iter_goal_points.emplace_back(Eigen::Vector3d(position.x(), position.y(), 5/57.3));

            // -5度
            iter_goal_points.emplace_back(Eigen::Vector3d(position.x(), position.y(), -5/57.3));

            // 10度
            iter_goal_points.emplace_back(Eigen::Vector3d(position.x(), position.y(), 10/57.3));

            // -10度
            iter_goal_points.emplace_back(Eigen::Vector3d(position.x(), position.y(), -10/57.3));

            for (auto & goal_point : iter_goal_points)
            {
                if (local_planner_traditional.isGoalFeasible(goal_point))
                {
                    goal = goal_point;
                    return true;
                }
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

bool variousTerrainPlanner::CheckFeasibleGoalTraditional(grid_map::GridMap & map, vector<Eigen::Vector3d> & goal_points_final)
{
    if (local_planner_traditional.getPlannerPtr())
    {
        vector<Eigen::Vector2d> goal_points;
        goal_points.emplace_back(Eigen::Vector2d(4.5, 0));
        goal_points.emplace_back(Eigen::Vector2d(3.5, 0));
        goal_points.emplace_back(Eigen::Vector2d(4.4, 2));
        goal_points.emplace_back(Eigen::Vector2d(4.4, -2));
        goal_points.emplace_back(Eigen::Vector2d(3.4, 1.5));
        goal_points.emplace_back(Eigen::Vector2d(3.4, -1.5));
        for (auto & tmp_goal_point : goal_points)
        {
            bool goal_flag = false;
            grid_map::SpiralIterator iterator(map, tmp_goal_point, (foot_param.x_upper + foot_param.x_button)/2);
            while (!iterator.isPastEnd())
            {
                grid_map::Position position;
                map.getPosition(*iterator, position);
                vector<Eigen::Vector3d> iter_goal_points;
                
                // 0度
                iter_goal_points.emplace_back(Eigen::Vector3d(position.x(), position.y(), 0));

                // 5度
                iter_goal_points.emplace_back(Eigen::Vector3d(position.x(), position.y(), 5/57.3));

                // -5度
                iter_goal_points.emplace_back(Eigen::Vector3d(position.x(), position.y(), -5/57.3));

                // 10度
                iter_goal_points.emplace_back(Eigen::Vector3d(position.x(), position.y(), 10/57.3));

                // -10度
                iter_goal_points.emplace_back(Eigen::Vector3d(position.x(), position.y(), -10/57.3));

                for (auto & goal_point : iter_goal_points)
                {
                    if (local_planner_traditional.isGoalFeasible(goal_point))
                    {
                        goal_points_final.emplace_back(goal_point);
                        goal_flag = true;
                        break; 
                    }
                }
                if (goal_flag)
                {
                    break;
                }
                ++iterator;
            }
        }
        if (goal_points_final.empty())
        {
			ROS_ERROR("No goal points found!");
            return false;
        }
        else
        {
            return true;
        }
    }
    else
    {
        return false;
    }
}

bool variousTerrainPlanner::CheckFeasibleGoalPropose(grid_map::GridMap & map, Eigen::Vector2d cand_goal, Eigen::Vector3d & goal)
{
    if (local_planner_propose.getPlannerPtr())
    {
        grid_map::SpiralIterator iterator(map, cand_goal, (foot_param.x_upper + foot_param.x_button)/2);
        while (!iterator.isPastEnd())
        {
            grid_map::Position position;
            map.getPosition(*iterator, position);
            vector<Eigen::Vector3d> iter_goal_points;
            
            // 0度
            iter_goal_points.emplace_back(Eigen::Vector3d(position.x(), position.y(), 0));

            // 5度
            iter_goal_points.emplace_back(Eigen::Vector3d(position.x(), position.y(), 5/57.3));

            // -5度
            iter_goal_points.emplace_back(Eigen::Vector3d(position.x(), position.y(), -5/57.3));

            // 10度
            iter_goal_points.emplace_back(Eigen::Vector3d(position.x(), position.y(), 10/57.3));

            // -10度
            iter_goal_points.emplace_back(Eigen::Vector3d(position.x(), position.y(), -10/57.3));

            for (auto & goal_point : iter_goal_points)
            {
                if (local_planner_propose.isGoalFeasible(goal_point))
                {
                    goal = goal_point;
                    return true;
                }
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

bool variousTerrainPlanner::CheckFeasibleGoalPropose(grid_map::GridMap & map, vector<Eigen::Vector3d> & goal_points_final)
{
	if (local_planner_propose.getPlannerPtr())
    {
        vector<Eigen::Vector2d> goal_points;
        goal_points.emplace_back(Eigen::Vector2d(4.5, 0));
        goal_points.emplace_back(Eigen::Vector2d(3.5, 0));
        goal_points.emplace_back(Eigen::Vector2d(4.4, 2));
        goal_points.emplace_back(Eigen::Vector2d(4.4, -2));
        goal_points.emplace_back(Eigen::Vector2d(3.4, 2));
        goal_points.emplace_back(Eigen::Vector2d(3.4, -2));
        for (auto & tmp_goal_point : goal_points)
        {
            bool goal_flag = false;
            grid_map::SpiralIterator iterator(map, tmp_goal_point, (foot_param.x_upper + foot_param.x_button)/2);
            while (!iterator.isPastEnd())
            {
                grid_map::Position position;
                map.getPosition(*iterator, position);
                vector<Eigen::Vector3d> iter_goal_points;
                
                // 0度
                iter_goal_points.emplace_back(Eigen::Vector3d(position.x(), position.y(), 0));

                // 5度
                iter_goal_points.emplace_back(Eigen::Vector3d(position.x(), position.y(), 5/57.3));

                // -5度
                iter_goal_points.emplace_back(Eigen::Vector3d(position.x(), position.y(), -5/57.3));

                // 10度
                iter_goal_points.emplace_back(Eigen::Vector3d(position.x(), position.y(), 10/57.3));

                // -10度
                iter_goal_points.emplace_back(Eigen::Vector3d(position.x(), position.y(), -10/57.3));

                for (auto & goal_point : iter_goal_points)
                {
                    if (local_planner_propose.isGoalFeasible(goal_point))
                    {
                        goal_points_final.emplace_back(goal_point);
                        goal_flag = true;
                        break;
                    }
                }
                if (goal_flag)
                {
                    break;
                }
                ++iterator;
            }
        }
        if (goal_points_final.empty())
        {
			ROS_ERROR("No goal points found!");
            return false;
        }
        else
        {
            return true;
        }
        
    }
    else
    {
        return false;
    }
}

bool variousTerrainPlanner::checkFeasibleStartTraditional(grid_map::GridMap & map, Eigen::Vector3d & left_foot_tra, Eigen::Vector3d & right_foot_tra)
{
    grid_map::Position start(0.3, 0);
    grid_map::SpiralIterator iterator(map, start, (foot_param.x_upper + foot_param.x_button)/2);
    while (!iterator.isPastEnd())
    {
        grid_map::Position position;
        map.getPosition(*iterator, position);
        vector<Eigen::Vector3d> start_points;
        
        // 0度
        start_points.emplace_back(Eigen::Vector3d(position.x(), position.y(), 0));

        // 5度
        start_points.emplace_back(Eigen::Vector3d(position.x(), position.y(), 5/57.3));

        // -5度
        start_points.emplace_back(Eigen::Vector3d(position.x(), position.y(), -5/57.3));

        // 10度
        start_points.emplace_back(Eigen::Vector3d(position.x(), position.y(), 10/57.3));

        // -10度
        start_points.emplace_back(Eigen::Vector3d(position.x(), position.y(), -10/57.3));

        for (auto & tmp_start_point : start_points)
        {
            // 两种情况均成立，才以此点为终点
			if (local_planner_traditional.isStartFeasible(tmp_start_point, left_foot_tra, right_foot_tra))
			{
				return true;
			}
        }
        ++iterator;
    }
    return false;
}

bool variousTerrainPlanner::checkFeasibleStartPropose(grid_map::GridMap & map, Eigen::Vector3d & propose_left_foot, Eigen::Vector3d & propose_right_foot)
{
	grid_map::Position start(0.3, 0);
    grid_map::SpiralIterator iterator(map, start, (foot_param.x_upper + foot_param.x_button)/2);
    while (!iterator.isPastEnd())
    {
        grid_map::Position position;
        map.getPosition(*iterator, position);
        vector<Eigen::Vector3d> start_points;
        
        // 0度
        start_points.emplace_back(Eigen::Vector3d(position.x(), position.y(), 0));

        // 5度
        start_points.emplace_back(Eigen::Vector3d(position.x(), position.y(), 5/57.3));

        // -5度
        start_points.emplace_back(Eigen::Vector3d(position.x(), position.y(), -5/57.3));

        // 10度
        start_points.emplace_back(Eigen::Vector3d(position.x(), position.y(), 10/57.3));

        // -10度
        start_points.emplace_back(Eigen::Vector3d(position.x(), position.y(), -10/57.3));

        for (auto & tmp_start_point : start_points)
        {
            // 两种情况均成立，才以此点为终点
			if (local_planner_propose.isStartFeasible(tmp_start_point, propose_left_foot, propose_right_foot))
			{
				return true;
			}
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
            file << "step: " << step.x << " " << step.y << " " << step.z << " " << step.roll << " " << step.pitch << " " << step.yaw << std::endl;
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
            file << "step: " << step.x << " " << step.y << " " << step.z << " " << step.roll << " " << step.pitch << " " << step.yaw << std::endl;
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
    file<<"Starting traditionalPlanner..."<<endl;
	try 
	{
		// for (int i = 0; i < 60; ++i) 
		// {
		// 	std::cout << "traditional i = " << i << ", ";
		// 	boost::this_thread::sleep_for(boost::chrono::seconds(1)); // 模拟耗时操作
		// 	boost::this_thread::interruption_point(); // 检查是否被中断
		// }
		local_planner_traditional.plan();
        getPlannerResultTraditional();
        file <<local_planner_traditional.timeConsumption()<<endl;
	} 
	catch (const boost::thread_interrupted&) 
	{
		std::cout << "\ntraditionalPlanner interrupted!\n";
        file << "traditionalPlanner interrupted!"<<endl;
	}
	std::cout << "traditionalPlanner completed.\n";
    file << "traditionalPlanner completed."<<endl;
}

void variousTerrainPlanner::proposePlanner() 
{
    std::cout << "Starting proposePlanner...\n";
    file<<"Starting proposePlanner..."<<endl;
	try 
	{
		// for (int i = 0; i < 60; ++i) 
		// {
		// 	std::cout << "propose i = " << i << ", ";
		// 	boost::this_thread::sleep_for(boost::chrono::seconds(1)); // 模拟耗时操作
		// 	boost::this_thread::interruption_point(); // 检查是否被中断
		// }
		local_planner_propose.plan();
        getPlannerResultPropose();
        file <<local_planner_propose.timeConsumption()<<endl;
	} 
	catch (const boost::thread_interrupted&) 
	{
		std::cout << "\nproposePlanner interrupted!\n";
        file << "proposePlanner interrupted!"<<endl;
	}
	std::cout << "proposePlanner completed.\n";
    file << "proposePlanner completed."<<endl;
}

void variousTerrainPlanner::executeTaskWithTimeout(std::function<void()> task, int timeout) 
{
    boost::thread worker(task); // 创建一个线程来运行任务
    if (!worker.try_join_for(boost::chrono::seconds(timeout))) 
	{
        std::cout << "\nTask timed out, interrupting...\n";
        file << "Task timed out, interrupting..."<<endl;
        worker.interrupt(); // 中断线程
        worker.join(); // 等待线程退出
        std::cout << "Task interrupted and skipped to the next task.\n";
        file << "Task interrupted and skipped to the next task."<<endl;
    } 
	else 
	{
        std::cout << "Task completed within the time limit.\n";
        file << "Task completed within the time limit."<<endl;
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
	double map_length = 5.0;
    double map_width = 5.0;
	double resolution = 0.01;
	grid_map::GridMap map({"elevation"});
    map.setFrameId("map");
    map.setGeometry(grid_map::Length(map_length, map_width), resolution, grid_map::Position(map_length/2.0, 0));

	while (ros::ok())
	{
        vector<Eigen::Vector2d> goal_points;
        goal_points.emplace_back(Eigen::Vector2d(4.5, 0));
        goal_points.emplace_back(Eigen::Vector2d(3.5, 0));
        goal_points.emplace_back(Eigen::Vector2d(4.4, 2));
        goal_points.emplace_back(Eigen::Vector2d(4.4, -2));
        goal_points.emplace_back(Eigen::Vector2d(3.4, 2));
        goal_points.emplace_back(Eigen::Vector2d(3.4, -2));

        for (auto & goal : goal_points)
		{
            std::shared_ptr<AstarHierarchicalFootstepPlannerTraditional> traditional_planner_ptr = std::make_shared<AstarHierarchicalFootstepPlannerTraditional>();
            traditional_planner_ptr->setCheckParam(checkXupper, checkXButton);
            local_planner_traditional.setPlanner(traditional_planner_ptr);
            local_planner_traditional.setFootParam(foot_param);
            local_planner_traditional.setHipWidth(hip_width);

            std::shared_ptr<AstarHierarchicalFootstepPlannerPropose> propose_planner_ptr = std::make_shared<AstarHierarchicalFootstepPlannerPropose>();
            local_planner_propose.setPlanner(propose_planner_ptr);
            local_planner_propose.setFootParam(foot_param);
            local_planner_propose.setHipWidth(hip_width);
		
            // 输入多变的地形,台阶宽度0.06-0.27，间隙0.02-0.17
            std::random_device rd_step; // 随机数种子
            std::mt19937 gen_step(rd_step()); // 随机数生成器
            std::uniform_int_distribution<> dist_int_step(6, 27);
            // int step_width = dist_int_step(gen_step);

            std::random_device rd_gap; // 随机数种子
            std::mt19937 gen_gap(rd_gap()); // 随机数生成器
            std::uniform_int_distribution<> dist_int_gap(2, 17);
            // int gap_width = dist_int_gap(gen_gap);

            double step_elevation = 0.1;
            double gap_elevation = 0.0;
            map.clearAll();
            int index_length = 0;
            vector<int> map_design;
            while (index_length < map.getSize().x())
            {
                int step_index_length = dist_int_step(gen_step);
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
                int gap_index_length = dist_int_gap(gen_gap);
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

            file<<"map_design: ";
            for (auto i : map_design)
            {
                file<<i<<" ";
            }
            file<<endl;
            local_planner_traditional.mapPrepare(map);
            local_planner_propose.mapPrepare(map);

		    LOG(INFO)<<"mapPrepare finish";

            Eigen::Vector3d goal_point_tra, goal_point_propose;
            Eigen::Vector3d left_foot_tra, right_foot_tra, propose_left_foot, propose_right_foot;

            if (checkFeasibleStartTraditional(map, left_foot_tra, right_foot_tra))
            {
                LOG(INFO)<<"Traditional: start is feasible";
                if (CheckFeasibleGoalTraditional(map, goal, goal_point_tra))
                {
                    LOG(INFO)<<"Traditional: goal is feasible";
                    
                    file<<"tra stat and goal: "<<left_foot_tra.transpose()<<", "<<right_foot_tra.transpose()<<", "<<goal_point_tra.transpose()<<endl;
                    local_planner_traditional.initial(left_foot_tra, right_foot_tra, 0, goal_point_tra);
                    const int TIME_LIMIT = 60;
                    executeTaskWithTimeout([this]() { traditionalPlanner(); }, TIME_LIMIT);
                    
                }
                else
                {
                    LOG(ERROR)<<"Traditional: goal is not feasible";
                    file<<"Traditional: goal is not feasible"<<endl;
                }
            }
            else
            {
                LOG(ERROR)<<"Traditional: start is not feasible";
                file<<"Traditional: start is not feasible"<<endl;
            }
            

            if (checkFeasibleStartPropose(map, propose_left_foot, propose_right_foot))
            {
                LOG(INFO)<<"Propose: start is feasible";
                if (CheckFeasibleGoalPropose(map, goal, goal_point_propose))
                {
                    LOG(INFO)<<"Propose: goal is feasible";
                    file<<"pro stat and goal: "<<propose_left_foot.transpose()<<", "<<propose_right_foot.transpose()<<", "<<goal_point_propose.transpose()<<endl;
                    local_planner_propose.initial(propose_left_foot, propose_right_foot, 0, goal_point_propose);
                    const int TIME_LIMIT = 60;
                    executeTaskWithTimeout([this]() { proposePlanner(); }, TIME_LIMIT);
                }
                else
                {
                    LOG(ERROR)<<"Propose: goal is not feasible";
                }
            }
            else
            {
                LOG(ERROR)<<"Propose: start is not feasible";
            }
        }

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
}

variousTerrainPlanner::~variousTerrainPlanner()
{
    file.close();
}