#pragma once
#include "States.h"
#include "BasicGraph.h"

// 保留表
class ReservationTable
{
public:
    size_t map_size;//栅格的数量
    int num_of_agents;//机器人的数量
    int k_robust;//k_robust是SIPP上使用的安全区间
    int window;//这是RHCR算法的规划时间窗口h
    bool use_cat; // use conflict avoidance table
	bool hold_endpoints = false;//到达目标点后是否保持，true为保持，false为到达目标点后消失

    bool prioritize_start;
    double runtime;

    void clear() {sit.clear(); ct.clear(); cat.clear(); }
	void copy(const ReservationTable& other) {sit = other.sit; ct = other.ct; cat = other.cat; }
    void build(const vector<Path*>& paths,
               const list< tuple<int, int, int> >& initial_constraints,
               const unordered_set<int>& high_priority_agents, int current_agent, int start_location);
    void build(const vector<Path>& paths,
               const list< tuple<int, int, int> >& initial_constraints,
               int current_agent);
    void build(const vector<Path*>& paths,
               const list< tuple<int, int, int> >& initial_constraints,
               const list< Constraint >& constraints, int current_agent);
	void insertPath2CT(const Path& path); // insert the path to the constraint table
	void print() const;
    void printCT(size_t location) const;

    // functions  for SIPP
    list<Interval> getSafeIntervals(int location, int lower_bound, int upper_bound);
	list<Interval> getSafeIntervals(int from, int to, int lower_bound, int upper_bound);
	int getHoldingTimeFromSIT(int location);
    Interval getFirstSafeInterval(int location);
    bool findSafeInterval(Interval& interval, int location, int t_min);

	// functions for state-time A*
	bool isConstrained(int curr_id, int next_id, int next_timestep) const;
	bool isConflicting(int curr_id, int next_id, int next_timestep) const;
	int getHoldingTimeFromCT(int location) const;
    set<int> getConstrainedTimesteps(int location) const;

	ReservationTable(const BasicGraph& G): G(G) {}
private:
	const BasicGraph& G;
	// Constraint Table (CT)约束表，硬约束
    // location/edge -> time range first为点或者边，second为时间范围标识了不能使用的时间段
	unordered_map<size_t, list<pair<int, int> > > ct; 
	// Conflict Avoidance Table (CAT) 软约束
	vector<vector<bool> > cat; //  (timestep, location) ->  have conflicts or not
	// Safe Interval Table (SIT)
	unordered_map<size_t, list<Interval > > sit; // location/edge -> [t_min, t_max), num_of_collisions

	void updateSIT(size_t location); // update SIT at the gvien location
	void mergeIntervals(list<Interval >& intervals) const; //merge successive safe intervals with the same number of conflicts.



    void insertConstraint2SIT(int location, int t_min, int t_max);
    void insertSoftConstraint2SIT(int location, int t_min, int t_max);
    void insertConstraints4starts(const vector<Path*>& paths, int current_agent, int start_location);	
	void insertPath2CAT(const Path& path); //  insert the path to the conflict avoidance table
	void addInitialConstraints(const list< tuple<int, int, int> >& initial_constraints, int current_agent);
	inline int getEdgeIndex(int from, int to) const {return (from + 1) * map_size + to; }
	inline pair<int, int> getEdge(int index) const {return make_pair(index / map_size - 1, index % map_size); }

};