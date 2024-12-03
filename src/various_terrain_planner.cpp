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
}

bool variousTerrainPlanner::CheckFeasibleGoal(grid_map::GridMap & map, int radius, vector<Eigen::Vector3d> & goal_points_final)
{
    if (local_planner_traditional.getPlannerPtr() && local_planner_propose.getPlannerPtr())
    {
        vector<Eigen::Vector2d> goal_points;
        goal_points.emplace_back(Eigen::Vector2d(4.5, 0));
        // goal_points.emplace_back(Eigen::Vector2d(3.8, 0));
        // goal_points.emplace_back(Eigen::Vector2d(4.4, 2));
        // goal_points.emplace_back(Eigen::Vector2d(4.4, -2));
        // goal_points.emplace_back(Eigen::Vector2d(3.4, 2));
        // goal_points.emplace_back(Eigen::Vector2d(3.4, -2));
        for (auto & tmp_goal_point : goal_points)
        {
            bool goal_flag = false;
            grid_map::SpiralIterator iterator(map, tmp_goal_point, radius);
            while (!iterator.isPastEnd())
            {
                grid_map::Position position;
                map.getPosition(*iterator, position);
                vector<Eigen::Vector3d> goal_points;
                
                // 0度
                goal_points.emplace_back(Eigen::Vector3d(position.x(), position.y(), 0));

                // 5度
                goal_points.emplace_back(Eigen::Vector3d(position.x(), position.y(), 5/57.3));

                // -5度
                goal_points.emplace_back(Eigen::Vector3d(position.x(), position.y(), -5/57.3));

                // 10度
                goal_points.emplace_back(Eigen::Vector3d(position.x(), position.y(), 10/57.3));

                // -10度
                goal_points.emplace_back(Eigen::Vector3d(position.x(), position.y(), -10/57.3));

                for (auto & goal_point : goal_points)
                {
                    if (local_planner_traditional.isGoalFeasible(goal_point) && local_planner_propose.isGoalFeasible(goal_point))
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

bool variousTerrainPlanner::checkFeasibleStart(grid_map::GridMap & map, Eigen::Vector3d & left_foot_tra, Eigen::Vector3d & left_right_tra, Eigen::Vector3d & propose_left_foot, Eigen::Vector3d & propose_right_foot)
{
    grid_map::Position start(0.2, 0);
    grid_map::SpiralIterator iterator(map, start, 0.15);
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
			if (local_planner_traditional.isStartFeasible(tmp_start_point, left_foot_tra, left_right_tra))
			{
				if (local_planner_propose.isStartFeasible(tmp_start_point, propose_left_foot, propose_right_foot))
				{
					return true;
				}
				else
				{
					LOG(INFO)<<"PROPOSE : start point is not feasible"<<endl;
					return false;
				}
			}
			else
			{
					LOG(INFO)<<"TRADITIONAL : start point is not feasible"<<endl;
					return false;
			}
        }
        ++iterator;
    }
    return false;
}

void variousTerrainPlanner::traditionalPlanner() 
{
    std::cout << "Starting traditionalPlanner...\n";
	try 
	{
		// for (int i = 0; i < 60; ++i) 
		// {
		// 	std::cout << "traditional i = " << i << ", ";
		// 	boost::this_thread::sleep_for(boost::chrono::seconds(1)); // 模拟耗时操作
		// 	boost::this_thread::interruption_point(); // 检查是否被中断
		// }
		local_planner_traditional.plan();
	} 
	catch (const boost::thread_interrupted&) 
	{
		std::cout << "\ntraditionalPlanner interrupted!\n";
	}
	std::cout << "traditionalPlanner completed.\n";
}

void variousTerrainPlanner::proposePlanner() 
{
    std::cout << "Starting proposePlanner...\n";
	try 
	{
		// for (int i = 0; i < 60; ++i) 
		// {
		// 	std::cout << "propose i = " << i << ", ";
		// 	boost::this_thread::sleep_for(boost::chrono::seconds(1)); // 模拟耗时操作
		// 	boost::this_thread::interruption_point(); // 检查是否被中断
		// }
		local_planner_propose.plan();
	} 
	catch (const boost::thread_interrupted&) 
	{
		std::cout << "\nproposePlanner interrupted!\n";
	}
	std::cout << "proposePlanner completed.\n";
}

void variousTerrainPlanner::executeTaskWithTimeout(std::function<void()> task, int timeout) 
{
    boost::thread worker(task); // 创建一个线程来运行任务
    if (!worker.try_join_for(boost::chrono::seconds(timeout))) 
	{
        std::cout << "\nTask timed out, interrupting...\n";
        worker.interrupt(); // 中断线程
        worker.join(); // 等待线程退出
        std::cout << "Task interrupted and skipped to the next task.\n";
    } 
	else 
	{
        std::cout << "Task completed within the time limit.\n";
    }
}
void variousTerrainPlanner::execute()
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
	double resolution = 0.02;
	grid_map::GridMap map({"elevation"});
    map.setFrameId("map");
    map.setGeometry(grid_map::Length(map_length, map_width), 0.02, grid_map::Position(map_length/2.0, 0));

	while (ros::ok())
	{
		// for (int i = 0; i <= 9; i++) //9
		// {
		// 	for (int j = 0; j <= 8; j++) //8
		// 	{
		int i = 8;
		int j = 8;
				cout<<"i: "<<i<<" j: "<<j<<endl;
				double step_width = 0.02 + resolution*i;// 0.06+0.16 = 0.24
				double gap_width = 0.02 + resolution*j; // 0.02+ 0.08 = 0.1
				LOG(INFO)<<"step_width: "<<step_width<<" gap_width: "<<gap_width;
				int step_index_length = step_width/resolution;
				int gap_index_length = gap_width/resolution;
				LOG(INFO)<<"step_index_length: "<<step_index_length<<" gap_index_length: "<<gap_index_length;
				double step_elevation = 0.1;
				double gap_elevation = 0.0;
				map.clearAll();
				cout<<step_width<<" "<<gap_width<<endl;
				int index_length = 0;
				while (index_length < map.getSize().x())
				{
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
					index_length = index_length + step_index_length;
					if (index_length >= map.getSize().x())
					{
						break;
					}
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
					index_length = index_length + gap_index_length;
				}
			
				local_planner_traditional.mapPrepare(map);

				local_planner_propose.mapPrepare(map);

				vector<Eigen::Vector3d> goal_points;
				Eigen::Vector3d left_foot_tra, left_right_tra, propose_left_foot, propose_right_foot;
				if (CheckFeasibleGoal(map, 0.2, goal_points))
				{
					if (checkFeasibleStart(map, left_foot_tra, left_right_tra, propose_left_foot, propose_right_foot))
					{
#ifdef DEBUG
						LOG(INFO)<<"goal points size: "<<goal_points.size();
						LOG(INFO)<<"goal: "<<goal_points.at(0).transpose();
						LOG(INFO)<<left_foot_tra.transpose();
						LOG(INFO)<<left_right_tra.transpose();
						LOG(INFO)<<propose_left_foot.transpose();
						LOG(INFO)<<propose_right_foot.transpose();
#endif
						local_planner_traditional.initial(left_foot_tra, left_right_tra, 0, goal_points.at(0));
						local_planner_propose.initial(propose_left_foot, propose_right_foot, 0, goal_points.at(0));
						const int TIME_LIMIT = 60;

						// 执行两个任务，分别设置超时时间
						executeTaskWithTimeout([this]() { traditionalPlanner(); }, TIME_LIMIT);
						executeTaskWithTimeout([this]() { proposePlanner(); }, TIME_LIMIT);
					}
					else
					{
						LOG(INFO)<<"not suitable start point";
					}
				}
				else
				{
					LOG(INFO)<<"not suitable goal point";
				}
		// 	}
		// }
	
		break;
	
	}
#endif
}

variousTerrainPlanner::~variousTerrainPlanner()
{
}