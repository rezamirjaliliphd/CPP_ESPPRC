#ifndef GRAPH_H
#define GRAPH_H


#include <vector>
#include "Edge.h"
#include "Label.h"
#include <memory>


class Graph {
public:
    std::vector<std::vector<std::shared_ptr<Edge>>> OutList;
	std::vector<std::vector<std::shared_ptr<Edge>>> InList;
    std::vector<std::shared_ptr<Edge>> edges;
    std::vector<std::vector<bool>> predecessor;
    int num_nodes;
    int num_res;
    int num_edges = 0;
    std::vector<std::vector<double>> min_weight;
    std::vector<double> max_value;
    std::vector<double> res_max;

    Graph(int n, int m, std::vector<double> r_max);

    void addEdge(int from, int to, double cost, const std::vector<double>& resources);
    const std::vector<std::shared_ptr<Edge>> getNeighbors(int node, bool dir) const;
	void deleteEdge(int from, int to);
    void display() const;
    bool is_neighbor(const int from, const int to) const;
    std::vector<std::vector<double>> getMinWeights();
	Edge& getEdge(int from, int to) const;
    void getMaxValue();
};


#endif // GRAPH_H