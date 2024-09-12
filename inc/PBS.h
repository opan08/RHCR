#pragma once
#include "PBSNode.h"
#include "MAPFSolver.h"
#include <ctime>

// TODO: add topological sorting

class PBS:
	public MAPFSolver
{
public:
    bool lazyPriority;//默认是false
    bool prioritize_start = true;//设置是否在起点等待的优先级高于正在移动的agent的优先级,true表示运动的agent需要让在起点的agent

	 // runtime breakdown
    double runtime_rt = 0;
    double runtime_plan_paths = 0;
    double runtime_get_higher_priority_agents = 0;
    double runtime_copy_priorities = 0;
    double runtime_detect_conflicts = 0;
    double runtime_copy_conflicts = 0;
    double runtime_choose_conflict = 0;
    double runtime_find_consistent_paths = 0;
    double runtime_find_replan_agents = 0;


	PBSNode* dummy_start = nullptr;
	PBSNode* best_node;

	uint64_t HL_num_expanded = 0;//记录高层搜索扩展的节点数目
	uint64_t HL_num_generated = 0;//记录高层搜索产生的节点数目
	uint64_t LL_num_expanded = 0;//记录低层搜索扩展的节点数目
	uint64_t LL_num_generated = 0;//记录低层搜索产生的节点数目


	double min_f_val = 0;


	// Runs the algorithm until the problem is solved or time is exhausted 
    bool run(const vector<State>& starts,
            const vector< vector<pair<int, int> > >& goal_locations, // an ordered list of pairs of <location, release time>
            int time_limit);


    PBS(const BasicGraph& G, SingleAgentSolver& path_planner);
	~PBS();

    void update_paths(PBSNode* curr);
	// Save results
	void save_results(const std::string &fileName, const std::string &instanceName) const;
	void save_search_tree(const std::string &fileName) const;
	void save_constraints_in_goal_node(const std::string &fileName) const;

	string get_name() const {return "PBS"; }

	void clear();

	void setRT(bool use_cat, bool prioritize_start)
	{
		rt.use_cat = use_cat;
		rt.prioritize_start = prioritize_start;
	}

private:

    std::vector< Path* > paths;
    list<PBSNode*> allNodes_table;
    list<PBSNode*> dfs;//待搜索的节点队列

   //  vector<State> starts;
    // vector< vector<int> > goal_locations;

    std::clock_t start = 0;

	// double focal_w = 1.0;
    unordered_set<pair<int, int>> nogood;

    // SingleAgentICBS astar;


    bool generate_root_node();
    void push_node(PBSNode* node);
    PBSNode* pop_node();

    // high level search
    // 规划agent的路径，node负责提供约束
    // 规划结果保存到node->paths中
	bool find_path(PBSNode*  node, int ag);
    bool find_consistent_paths(PBSNode* node, int a); // find paths consistent with priorities
    static void resolve_conflict(const Conflict& conflict, PBSNode* n1, PBSNode* n2);
	bool generate_child(PBSNode* child, PBSNode* curr);

	// conflicts
    void remove_conflicts(list<Conflict>& conflicts, int excluded_agent);
    void find_conflicts(const list<Conflict>& old_conflicts, list<Conflict> & new_conflicts, int new_agent);
	void find_conflicts(list<Conflict> & conflicts, int a1, int a2);
    void find_conflicts(list<Conflict> & new_conflicts, int new_agent);
    void find_conflicts(list<Conflict> & new_conflicts);

	void choose_conflict(PBSNode &parent);
	void copy_conflicts(const list<Conflict>& conflicts, list<Conflict>& copy, int excluded_agent);
    void copy_conflicts(const list<Conflict>& conflicts,
                       list<Conflict>& copy, const vector<bool>& excluded_agents);

    double get_path_cost(const Path& path) const;
	
    // update information
    //void collect_constraints(const boost::unordered_set<int>& agents, int current_agent);
    void get_solution();

    void update_CAT(int ex_ag); // update conflict avoidance table
	void update_focal_list();
	inline void release_closed_list();

    // 如果best_node的最早冲突发生的时间早于当前节点node的最早冲突发生的时间
    // 或者时间相同，但是node的f_val更小，则更新best_node为node
    void update_best_node(PBSNode* node);

	// print and save
	void print_paths() const;
	void print_results() const;
	static void print_conflicts(const PBSNode &curr) ;


	// validate
	bool validate_solution();
    static bool validate_consistence(const list<Conflict>& conflicts, const PriorityGraph &G) ;


    // tools
    static bool wait_at_start(const Path& path, int start_location, int timestep) ;
    void find_replan_agents(PBSNode* node, const list<Conflict>& conflicts,
            unordered_set<int>& replan);
};

