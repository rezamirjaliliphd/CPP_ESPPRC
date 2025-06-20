// Graph.cpp

#include "Graph.h"
#include <iostream>

// Constructor
Graph::Graph(int n, std::vector<double> r_max)
    : OutList(n), InList(n), num_nodes(n), num_res(r_max.size()), res_max(r_max) {
	OutList.resize(n);
    InList.resize(n);
    predecessor = std::vector<std::vector<bool>>(n, std::vector<bool>(n, false));
}

// Method to add an edge to the graph
void Graph::addEdge(int tail, int head, double cost, const std::vector<double>& resources) {
	auto edge = std::make_shared<Edge>(tail,head, cost, resources);
    OutList[tail].push_back(edge);
    InList[head].push_back(edge);
	edges.push_back(edge);
    num_edges += 1;
	predecessor[tail][head] = true;
}

// Method to get neighbors of a node
const std::vector<std::shared_ptr<Edge>>& Graph::getNeighbors(int node, bool dir) const {
    return dir? OutList[node] : InList[node];
}

// Method to display the graph
void Graph::display() const {
    for (int i = 0; i < num_nodes; ++i) {
        std::cout << "Node " << i << ":\n";
        for (const auto& edge : OutList[i]) {
            std::cout << "  -> " << edge->head << " (Cost: " << edge->cost << ", Resources: [";
            for (size_t j = 0; j < edge->resources.size(); ++j) {
                std::cout << edge->resources[j] << (j + 1 < edge->resources.size() ? ", " : "");
            }
            std::cout << "])\n";
        }
    }
}

void Graph::deleteEdge(int tail, int head) {
    // Remove (v, w) from OutList[u]

    OutList[tail].erase(
        std::remove_if(OutList[tail].begin(), OutList[tail].end(),
            [head](std::shared_ptr<Edge> e) { return head == e->head; }),
        OutList[tail].end()
    );

    // Remove (u, w) from InList[v]
    InList[head].erase(
        std::remove_if(InList[head].begin(), InList[head].end(),
            [tail](std::shared_ptr<Edge> edge) { return edge->tail == tail; }),
        InList[head].end()
    );
	predecessor[tail][head] = false;

}

bool Graph::is_neighbor(const int tail, const int head) const {
	return predecessor[tail][head];
}

Edge& Graph::getEdge(int tail, int head) const {
	for (const auto& edge : OutList[tail]) {
		if (edge->head == head) {
			return *edge;
		}
	}
}

