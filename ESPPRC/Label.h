#ifndef LABEL_H
#define LABEL_H

#include <vector>
#include <unordered_set>
#include "Graph.h"
#include "Edge.h"
#include <gurobi_c++.h>
#include <map>
#include <memory>

//#include "MIP1.h"

class Graph;
class Edge;

enum class LabelStatus {
    NEW_OPEN,
    NEW_CLOSED,
    OPEN,
    CLOSED,
    DOMINATED
};

enum class DominanceStatus {
    DOMINATES,
    DOMINATED,
    INCOMPARABLE
};



class Label {
public:
    long long id = -1;
    int vertex;
    std::vector<int> path;
    double cost;
    std::vector<double> resources, rc;
    std::vector<bool> reachable;
    // Farzane: a vector of edges visited by the label
    /*std::vector<Edge> edges;*/
    // 
    double LB;
    LabelStatus status;
    std::shared_ptr<GRBModel> model=nullptr;
    std::map<std::pair<int, int>, double> min_res;

    Label(Graph& graph);
    Label(const Label& parent, Graph& graph, const Edge* edge, const double UB);

    void UpdateReachable(Graph& graph, const double UB);
    bool reachHalfPoint(const std::vector<double>& res_max, int num_nodes);
    void display() const;
    DominanceStatus DominanceCheck(const Label& rival) const;
    bool isConcatenable(const Label& bw_label, const std::vector<double>& r_max) const;
	bool LBImprove(Graph& graph);
	void getUpdateMinRes(Graph& graph);
	bool isInPath(int node) const;
};

#endif // LABEL_H