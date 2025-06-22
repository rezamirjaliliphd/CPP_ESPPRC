#ifndef LABEL_H
#define LABEL_H

#include <vector>
#include <unordered_set>
#include "Graph.h"
#include "Edge.h"
#include <map>
#include <memory>
#include <cstdint>


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
    bool open = true;
    std::vector<double> resources;
    uint64_t visited;
    std::unordered_set<long long> checked_with;
    bool direction;
    bool deleted = false;

    double LB = -1000;
    LabelStatus status;

    Label(Graph& graph,bool dir);
    Label(const std::shared_ptr<Label>& parent_ptr, Graph& graph, const Edge* edge, const double UB);

    void UpdateReachable(Graph& graph);
    bool reachHalfPoint(const std::vector<double>& res_max, int num_nodes);
    void display(Graph& graph) const;
    DominanceStatus DominanceCheck(const std::shared_ptr<Label>& rival_ptr) const;
    bool isConcatenable(const std::shared_ptr<Label>& partner_ptr, const std::vector<double>& r_max) const;
    void UpdateLB(Graph& graph);
};

#endif // LABEL_H