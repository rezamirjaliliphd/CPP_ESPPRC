#include "Label.h"
#include <iostream>
#include <cmath>
#include <algorithm>

Label::Label(Graph& graph, bool dir)
    : vertex(0), path({ 0 }), cost(0),
    resources(graph.num_res, 0),
    reachable(graph.num_nodes, true),
    direction(dir), LB(0), id(0), rc(graph.num_edges, 0) { // Farzane: initialized "edges()"
    status = LabelStatus::NEW_OPEN;
    reachable[0] = false;
    /*LB = cost;
    for (int i = 0; i < reachable.size(); i++) {
        if (reachable[i] && graph.max_value[i] < 0) {
            LB += graph.max_value[i];
        }
    }*/
	std::cout << "checking undreachable nodes" << std::endl;
    UpdateReachable(graph, 0);
    
}

// Farzane: passed pointer of MIP to the label
Label::Label(const Label& parent, Graph& graph, const Edge* edge, const double UB)
    : path(parent.path), cost(parent.cost),
    resources(parent.resources), reachable(parent.reachable),
    direction(parent.direction) {
	vertex = direction ? edge->to : edge->from;
    cost = parent.cost + edge->cost;

    //Farzane: add the edge data to visited edges by the label
    //edges.push_back(*edge);
    //

    if (direction) {
        path.push_back(vertex);
    } else {
        path.insert(path.begin(), vertex);
    }
    reachable[vertex] = false;
    reachable[0] = false;
    for (size_t i = 0; i < resources.size(); ++i) {
        resources[i] += edge->resources[i];
    }
    UpdateReachable(graph, UB);

    // Farzane: get LB and update it
    //LB = mip->solve_with(edges);
	

    // Farzane: commented out
    /*LB = cost+graph.max_value[vertex];
	for (int i = 0; i < reachable.size(); i++) {
		if (reachable[i]&& graph.max_value[i]<0) {
			LB+=graph.max_value[i];
		}
	}*/

    if (reachHalfPoint(graph.res_max, graph.num_nodes)) {
        status = LabelStatus::NEW_CLOSED;
    } else {
        status = LabelStatus::NEW_OPEN;
    }
    
    if (LB > UB) {
        status = LabelStatus::DOMINATED;
		//std::cout << "Pruned" << std::endl;
    }

}

void Label::UpdateReachable(Graph& graph, const double UB) {
	std::map<std::pair<int, int>, double> rc;
    double obj = 0;
	//std::cout << "Calculating RC" << std::endl;
	std::pair<std::map<std::pair<int, int>, double>, double> result = graph.getRCLabel(path);
	rc = result.first;
	obj = result.second;
    LB = obj;
    for (const auto e : graph.getNeighbors(vertex, direction)) {
        int neighbor = direction? e->to : e->from;
        if (reachable[neighbor] && neighbor != 0 ) {
            if (rc[{e->from, e->to}] > UB - LB) {
				//std::cout << "edge[" << e->from<<", "<< e->to <<"] is unreachable" << std::endl;
				reachable[neighbor] = false;
				continue;
            }
            for (size_t i = 0; i < resources.size(); ++i) {
                if (resources[i] + e->resources[i] > graph.res_max[i]) {
                    reachable[neighbor] = false;
                    break;
                }
            }
        }
    }
    

}

bool Label::reachHalfPoint(const std::vector<double>& res_max, int num_nodes) {
    if (path.size() >= static_cast<double>(num_nodes) / 2) {
        return true;
    }
    for (size_t i = 0; i < resources.size(); ++i) {
        if (resources[i] >= res_max[i] / 2) {
            return true;
        }
    }
    return false;
}


void Label::display() const {
    std::cout << "=========================\n";
    std::cout << "Direction: " << (direction? "Forward" : "Backward") << std::endl;
    std::cout << "Path: ";
    for (int node : path) {
        std::cout << node << " ";
    }
    std::cout << ", LB: " << LB << " Cost: " << cost << ", Resources : [";
    for (size_t i = 0; i < resources.size(); ++i) {
        std::cout << resources[i] << (i + 1 < resources.size() ? ", " : "");
    }
    std::cout << "]\n";
    for (size_t i = 0; i < reachable.size(); ++i) {
        if (reachable[i]) {
            std::cout << "Node " << i << " is reachable\n";
        }
    }
    std::cout << "Status: ";
    switch (status) {
    case LabelStatus::NEW_OPEN:    std::cout << "NEW_OPEN\n"; break;
    case LabelStatus::NEW_CLOSED:  std::cout << "NEW_CLOSED\n"; break;
    case LabelStatus::OPEN:        std::cout << "OPEN\n"; break;
    case LabelStatus::CLOSED:      std::cout << "CLOSED\n"; break;
    case LabelStatus::DOMINATED:   std::cout << "DOMINATED\n"; break;
    }
    std::cout << "ID (vertex, id): ( " << vertex << " , " << id << " )" << std::endl;
    std::cout << "=========================\n\n";
}

DominanceStatus Label::DominanceCheck(const Label& rival) const {
    bool dominates = true, dominated = true;

    for (size_t i = 0; i < resources.size(); ++i) {
        if (resources[i] > rival.resources[i]) dominates = false;
        if (resources[i] < rival.resources[i]) dominated = false;
        if (!dominates && !dominated) return DominanceStatus::INCOMPARABLE;
    }

    for (size_t i = 0; i < reachable.size(); ++i) {
        if (reachable[i] < rival.reachable[i]) dominates = false;
        if (reachable[i] > rival.reachable[i]) dominated = false;
        if (!dominates && !dominated) return DominanceStatus::INCOMPARABLE;
    }

    if (dominates && cost <= rival.cost) return DominanceStatus::DOMINATES;
    if (dominated && cost >= rival.cost) return DominanceStatus::DOMINATED;
    return DominanceStatus::INCOMPARABLE;
}

bool Label::isConcatenable(const Label& bw_label, const std::vector<double>& r_max) const {
    for (size_t i = 0; i < resources.size(); ++i) {
        if (resources[i] + bw_label.resources[i] > r_max[i]) {
            return false;
        }
    }

    std::unordered_set<int> first_set(path.begin() + 1, path.end());
    return std::none_of(bw_label.path.begin() + 1, bw_label.path.end() - 1,
        [&first_set](const int& x) { return first_set.count(x) > 0; });
}
