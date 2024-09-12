#pragma once
#include "BasicSystem.h"
#include "SortingGraph.h"


class SortingSystem :
	public BasicSystem
{
public:
    int c; // param for induct assignment induct被一个agent占用所产生的代价

	SortingSystem(const SortingGrid& G, MAPFSolver& solver);
    ~SortingSystem();

    void simulate(int simulation_time);

private:
	const SortingGrid& G;

    // record usage of induct stations
    // induct location + #drives that intends to go to this induct station
    // 记录每个induct当前被多少agent占用，first是induct的序号，second是这个indect被多少agent占用
    boost::unordered_map<int, int> drives_in_induct_stations; 

	void initialize();

    // assign tasks
    // 初始化起始位置，随机位置，不能与之前的重复，不是障碍物即可
    void initialize_start_locations();
    // 初始化目标位置，奇数的agent分配eject，偶数的agent分配induct
    void initialize_goal_locations();
    
	void update_goal_locations();

    int assign_induct_station(int curr) const;
    int assign_eject_station() const;
};

