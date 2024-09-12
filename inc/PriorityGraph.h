#pragma once
#include <boost/graph/graph_traits.hpp>
#include <boost/graph/adjacency_list.hpp>
#include "common.h"


class PriorityGraph
{
public:
    double runtime;

    void clear();
    bool empty() const {return G.empty(); }
    void copy(const PriorityGraph& other);
    void copy(const PriorityGraph& other, const vector<bool>& excluded_nodes);
    // 往G中添加一条边，from是父节点，to是子节点，from的优先级低于to
    void add(int from, int to); // from is lower than to
    // 从G中删除一条边，from是父节点，to是子节点，from的优先级低于to
    void remove(int from, int to); // from is lower than to
    // 判断from和to是否连通，能否从from找到路径到to
    bool connected(int from, int to) const;
    // 获取root能够到达的所有节点，也就是说获取优先级比root高的所有节点
    boost::unordered_set<int> get_reachable_nodes(int root);

    void save_as_digraph(std::string fname) const;
    typedef boost::unordered_map<int, boost::unordered_set<int> > PGraph_t;

    void update_number_of_lower_nodes(vector<int>& lower_nodes, int node) const;

    PGraph_t G;//first是父节点，second是子节点（或称邻居节点），父节点的优先级低于子节点

    // TODO:  connected components
};

