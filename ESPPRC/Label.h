#ifndef LABEL_H
#define LABEL_H

#include <vector>
#include <unordered_set>
#include "Graph.h"

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
    bool direction;
    double LB;
    LabelStatus status;

    Label(Graph& graph, bool dir);
    Label(const Label& parent, const Graph& graph, const Edge* edge, const double UB);

    void UpdateReachable(const Graph& graph, const double UB);
    bool reachHalfPoint(const std::vector<double>& res_max, int num_nodes);
    void display() const;
    DominanceStatus DominanceCheck(const Label& rival) const;
    bool isConcatenable(const Label& bw_label, const std::vector<double>& r_max) const;
};

#endif // LABEL_H