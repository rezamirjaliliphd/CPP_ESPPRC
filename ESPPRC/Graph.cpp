// Graph.cpp

#include "Graph.h"
#include <iostream>

// Constructor
Graph::Graph(int n, int m, std::vector<double> r_max)
    : OutList(n), InList(n), num_nodes(n), num_res(m),
    min_weight(n, std::vector<double>(m, 100.0)),
    max_value(n, 100.0), res_max(r_max) {
	OutList.resize(n); InList.resize(n);
    predecessor = std::vector<std::vector<bool>>(n, std::vector<bool>(n, false));
}

// Method to add an edge to the graph
void Graph::addEdge(int from, int to, double cost, const std::vector<double>& resources) {
	auto edge = make_shared<Edge>(from, to, cost, resources);
    OutList[from].push_back(edge);
    InList[to].push_back(edge);
	edges.push_back(edge);
    num_edges += 1;
	predecessor[from][to] = true;
}

// Method to get neighbors of a node
const std::vector<std::shared_ptr<Edge>> Graph::getNeighbors(int node, bool dir) const {
    return dir? OutList[node] : InList[node];
}

// Method to display the graph
void Graph::display() const {
    for (int i = 0; i < num_nodes; ++i) {
        std::cout << "Node " << i << ":\n";
        for (const auto edge : OutList[i]) {
            std::cout << "  -> " << edge->to << " (Cost: " << edge->cost << ", Resources: [";
            for (size_t j = 0; j < edge->resources.size(); ++j) {
                std::cout << edge->resources[j] << (j + 1 < edge->resources.size() ? ", " : "");
            }
            std::cout << "])\n";
        }
    }
}

// Method to get minimum weights
std::vector<std::vector<double>> Graph::getMinWeights() {
    for (int i = 0; i < num_nodes; i++) {
        for (const auto edge : OutList[i]) {
            for (int k = 0; k < edge->resources.size(); k++) {
                if (min_weight[i][k] > edge->resources[k]) {
                    min_weight[i][k] = edge->resources[k];
                }
            }
        }
    }
    return min_weight;
}
void Graph::deleteEdge(int from, int to ) {
    // Remove (v, w) from OutList[u]

    OutList[from].erase(
        std::remove_if(OutList[from].begin(), OutList[from].end(),
            [to](std::shared_ptr<Edge> e) { return to ==e->to; }),
        OutList[from].end()
    );

    // Remove (u, w) from InList[v]
    InList[to].erase(
        std::remove_if(InList[to].begin(), InList[to].end(),
            [from](std::shared_ptr<Edge> edge) { return edge->from == from; }),
        InList[to].end()
    );
	predecessor[from][to] = false;

}
// Method to get maximum values
void Graph::getMaxValue() {
    for (int i = 0; i < num_nodes; i++) {
        for (const auto edge : OutList[i]) {
            if (max_value[i] > edge->cost) {
                max_value[i] = edge->cost;
            }
        }
		//std::cout << "Node " << i << " Min Value: " << max_value[i] << std::endl;
    }
}
bool Graph::is_neighbor(const int from, const int to) const {
	return predecessor[from][to];
}

Edge& Graph::getEdge(int from, int to) const {
	for (const auto edge : OutList[from]) {
		if (edge->to == to) {
			return *edge;
		}
	}
}
