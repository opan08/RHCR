#include "SortingSystem.h"
#include <stdlib.h>
#include "PBS.h"
#include <boost/tokenizer.hpp>
#include "WHCAStar.h"
#include "ECBS.h"
#include "LRAStar.h"

// 这里初始化c=8，表示当induct被一个agent占用，代价为8
SortingSystem::SortingSystem(const SortingGrid& G, MAPFSolver& solver): BasicSystem(G, solver), c(8), G(G) {}


SortingSystem::~SortingSystem() {}

// 初始化起始位置，随机位置，不能与之前的重复，不是障碍物即可
void SortingSystem::initialize_start_locations()
{
    int N = G.size();
    std::vector<bool> used(N, false);

    // Choose random start locations
    // Any non-obstacle locations can be start locations
    // Start locations should be unique
	for (int k = 0; k < num_of_drives;)
	{
		int loc = rand() % N;
		if (G.types[loc] != "Obstacle" && !used[loc])
		{
			int orientation = -1;
			if (consider_rotation)
			{
				orientation = rand() % 4;
			}
			starts[k] = State(loc, 0, orientation);
			paths[k].emplace_back(starts[k]);
			used[loc] = true;
			finished_tasks[k].emplace_back(loc, 0);
			k++;
		}
	}
}

// 初始化目标位置，奇数的agent分配eject，偶数的agent分配induct
// 寻找的是最靠近当前位置的induct或者eject
// 初始化goal_locations
void SortingSystem::initialize_goal_locations()
{
	if (hold_endpoints || useDummyPaths)
		return;
    // Choose random goal locations
    // a close induct location can be a goal location, or
    // any eject locations can be goal locations
    // Goal locations are not necessarily unique
    for (int k = 0; k < num_of_drives; k++)
    {
		int goal;
		if (k % 2 == 0) // to induction
		{
			goal = assign_induct_station(starts[k].location);
			drives_in_induct_stations[goal]++;
		}
		else // to ejection
		{
			goal = assign_eject_station();
		}
		goal_locations[k].emplace_back(goal, 0);
    }
}

// 如果在重新规划之前完成任务，则重新给agent分配一个新的任务
void SortingSystem::update_goal_locations()
{
	for (int k = 0; k < num_of_drives; k++)
	{
		pair<int, int> curr(paths[k][timestep].location, timestep); // current location 获取agent当前timestep的位置,first是location,second是timestep

		pair<int, int> goal; // The last goal location,first是location,second是timestep
		if (goal_locations[k].empty())
		{
			// 如果agent的目标点列表为空，则使用当前位置作为目标点
			goal = curr;
		}
		else
		{
			goal = goal_locations[k].back();//获取agent的最后一个目标位置
		}
		// 这里通过曼哈顿距离近似timestep,每个timestep移动一个栅格（节点
		int min_timesteps;
		min_timesteps = G.get_Manhattan_distance(curr.first, goal.first);// 获取到达目标点所需的时间 // cannot use h values, because graph edges may have weights  // G.heuristics.at(goal)[curr]; 
		min_timesteps = max(min_timesteps, goal.second);
		while (min_timesteps <= simulation_window)
			// The agent might finish its tasks during the next planning horizon 如果能在重新规划之前完成任务
		{
			// RHCR论文中提到，d>=h的情况
			// assign a new task
			int next;
			if (G.types[goal.first] == "Induct")
			{
				// 如果到达了induct的位置，则随机选择一个eject作为新的目标
				next = assign_eject_station();
			}
			else if (G.types[goal.first] == "Eject")
			{
				// 如果已经到达了eject的位置，则重新选择一个induct作为新的目标
				next = assign_induct_station(curr.first);
				drives_in_induct_stations[next]++; // the drive will go to the next induct station
			}
			else
			{
				std::cout << "ERROR in update_goal_function()" << std::endl;
				std::cout << "The fiducial type should not be " << G.types[curr.first] << std::endl;
				exit(-1);
			}
			goal_locations[k].emplace_back(next, 0);//添加新的目标位置
			min_timesteps += G.get_Manhattan_distance(next, goal.first); // G.heuristics.at(next)[goal];
			min_timesteps = max(min_timesteps, goal.second);
			goal = make_pair(next, 0);
		}
	}
}

// 获取距离当前位置curr代价最小的induct
int SortingSystem::assign_induct_station(int curr) const
{
    int assigned_loc;
	double min_cost = DBL_MAX;
	for (auto induct : drives_in_induct_stations)
	{
		// G.heuristics.at(induct.first)[curr]表示当前位置到induct station的代价
		// induct.second表示induct已经被多少agent占用
		// 这个代价权衡了距离和induct的被占用情况
		double cost = G.heuristics.at(induct.first)[curr] + c * induct.second;
		if (cost < min_cost)
		{
			min_cost = cost;
			assigned_loc = induct.first;
		}
	}
    return assigned_loc;
}

// 从ejects中随机选择一个位置作为eject station
int SortingSystem::assign_eject_station() const
{
	int n = rand() % G.ejects.size();
	boost::unordered_map<std::string, std::list<int> >::const_iterator it = G.ejects.begin();
	std::advance(it, n);
	int p = rand() % it->second.size();
	auto it2 = it->second.begin();
	std::advance(it2, p);
	return *it2;
}

// 进行仿真，计算的主函数
void SortingSystem::simulate(int simulation_time)
{
    std::cout << "*** Simulating " << seed << " ***" << std::endl;
    this->simulation_time = simulation_time;
    initialize();
	
	// 这是大的循环，模拟每次重规划，只有当模拟达到次数的时候，才会停止，因为任务会在agent到达目标点的时候马上分配新任务
	for (; timestep < simulation_time; timestep += simulation_window)
	{
		std::cout << "Timestep " << timestep << std::endl;

		update_start_locations();//获取agent的当前位置
		update_goal_locations();//更新agent的目标位置
		solve();//通过带窗口的mapf算法求解（如LRA, PBS, WHCA, ECBS），主要是得到新的路径

		// move drives
		auto new_finished_tasks = move();//更新每个timestep的机器人的位置，时间段为[timestep, timestep + simulation_window]
		std::cout << new_finished_tasks.size() << " tasks has been finished" << std::endl << std::endl;

		// update tasks更新induct的占用情况
		for (auto task : new_finished_tasks)
		{
			int id, loc, t;
			std::tie(id, loc, t) = task;//<agent_id, location, timestep>
			finished_tasks[id].emplace_back(loc, t);
			num_of_tasks++;
			if (G.types[loc] == "Induct")
			{
				// 如果已经到达induct的位置，则induct的占用次数减1
				drives_in_induct_stations[loc]--; // the drive will leave the current induct station
			}
		}
		
		

		if (congested())
		{
			cout << "***** Too many traffic jams ***" << endl;
			break;
		}
	}

    update_start_locations();
    std::cout << std::endl << "Done!" << std::endl;
    save_results();
}

// 系统初始化
void SortingSystem::initialize()
{
	initialize_solvers();

	starts.resize(num_of_drives);
	goal_locations.resize(num_of_drives);
	paths.resize(num_of_drives);
	finished_tasks.resize(num_of_drives);

	// 初始化induct不被agent占用
	for (const auto induct : G.inducts)
	{
		drives_in_induct_stations[induct.second] = 0;
	}

	bool succ = load_records(); // continue simulating from the records
	if (!succ)
	{
		timestep = 0;
		succ = load_locations();
		if (!succ)
		{
			cout << "Randomly generating initial locations" << endl;
			initialize_start_locations();
			initialize_goal_locations();// 初始化goal_locations
		}
	}

	// initialize induct station counter
	for (int k = 0; k < num_of_drives; k++)
	{
		// goals
		int goal = goal_locations[k].back().first;
		if (G.types[goal] == "Induct")
		{
			drives_in_induct_stations[goal]++;
		}
		else if (G.types[goal] != "Eject")
		{
			// 这个条件限制了初始的目标要不是induct，要不是eject，不能是其他位置
			std::cout << "ERROR in the type of goal locations" << std::endl;
			std::cout << "The fiducial type of the goal of agent " << k << " is " << G.types[goal] << std::endl;
			exit(-1);
		}
	}
}
