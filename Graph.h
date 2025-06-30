#ifndef GRAPH_H
#define GRAPH_H


#include <vector>
#include "Edge.h"
#include "Label.h"
#include <memory>
#include <map>
#include <cmath>
#include <algorithm>

#define ROUND(value, places) (std::round((value) * std::pow(10.0, (places))) / std::pow(10.0, (places)))


class Graph {
public:
    std::vector<std::vector<std::shared_ptr<Edge>>> OutList;
	std::vector<std::vector<std::shared_ptr<Edge>>> InList;
    std::vector<std::shared_ptr<Edge>> edges;
    std::vector<std::vector<bool>> predecessor;
    int num_nodes;
    int num_res;
    int num_edges = 0;
    int phi_ID = 2;
    std::vector<double> Pi;
    int max_nodes = 1000;
    std::vector<double> max_value;
    std::vector<double> res_max;

    Graph(int n, std::vector<double> r_max, int phi_ID, std::vector<double>& pi);

    void addEdge(int from, int to, double cost, const std::vector<double>& resources);
    const std::vector<std::shared_ptr<Edge>>& getNeighbors(int node, bool dir) const;
	void deleteEdge(int from, int to);
    void display() const;
    bool is_neighbor(const int from, const int to) const;
	Edge& getEdge(int from, int to) const;

};


#endif // GRAPH_H