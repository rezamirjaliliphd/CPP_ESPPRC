#include "Label.h"
#include <iostream>
#include <cmath>
#include <algorithm>

Label::Label(Graph& graph, bool dir)
    : vertex(0), path({ 0 }), cost(0),
    resources(graph.num_res, 0),
    visited(0), LB(0), id(0) { 
    status = LabelStatus::OPEN;
    id = -1;
    direction = dir;
    visited = 1ULL << vertex;
    UpdateReachable(graph);
    UpdateLB(graph);
    if (reachHalfPoint(graph.res_max, graph.num_nodes)) {
        status = LabelStatus::NEW_CLOSED;
    }
    else {
        status = LabelStatus::NEW_OPEN;
    }

    if (LB > 100) {
        status = LabelStatus::DOMINATED;
        //std::cout << "Pruned" << std::endl;
    }
}

// Farzane: passed pointer of MIP to the label
Label::Label(const Label& parent, Graph& graph, const Edge* edge, const double UB)
    : path(parent.path), direction(parent.direction), cost(parent.cost),
    resources(parent.resources), visited(0),checked_with(parent.checked_with) {
    vertex = direction? edge->head:edge->tail;
    cost = parent.cost + edge->cost;
    visited = parent.visited|(1ULL << vertex);

    if (direction) {
        path.push_back(vertex);
    }
    else {
        path.insert(path.begin(), vertex);
    }

    for (size_t i = 0; i < resources.size(); ++i) {
        resources[i] += edge->resources[i];
    }
    //LBImprove(graph);
    UpdateReachable(graph);
    UpdateLB(graph);
    

    if (reachHalfPoint(graph.res_max, graph.num_nodes)) {
        status = LabelStatus::NEW_CLOSED;
    }
    else {
        status = LabelStatus::NEW_OPEN;
    }

    if (LB > UB) {
        status = LabelStatus::DOMINATED;
        //std::cout << "Pruned" << std::endl;
    }

}


void Label::UpdateReachable(Graph& graph) {

    for (const auto& e : graph.getNeighbors(vertex, direction)) {
        int neighbor = direction? e->head:e->tail;
        if (visited & (1ULL << neighbor)) continue;
            for (int k = 0; k < graph.num_res; k++) {
                if (resources[k] + e->resources[k] > graph.res_max[k]) {
                    visited |= (1ULL << neighbor);
                    break;
                }
        }
    }
}


bool Label::reachHalfPoint(const std::vector<double>& res_max, int num_nodes) {
    for (size_t i = 0; i < resources.size(); ++i) {
        if (resources[i] >= res_max[i] / 2) {
            return true;
        }
    }
    return false;
}



void Label::UpdateLB(Graph& graph) {
    LB = cost;
    double lb = 0;
    int neighbor = 0;
    for (int i = 0; i < graph.num_nodes; i++) {
        if (!(visited & (1ULL << i))||(i == vertex)) {
            lb = 0;
            for (const auto& e : direction? graph.OutList[i]: graph.InList[i]) {
                neighbor = direction? e->head:e->tail;
                if ( lb>e->cost && !(visited & (1ULL << neighbor))) lb = e->cost;
            }
            if (lb < 0) LB += lb;

        }
    }
}



void Label::display(Graph& graph) const {
    std::cout << "=========================\n";
    std::cout << "Path: ";
    for (int node : path) {
        std::cout << node << " ";
    }
    std::cout << ", LB: " << LB << " Cost: " << cost << ", Resources : [";
    for (size_t i = 0; i < resources.size(); ++i) {
        std::cout << resources[i] << (i + 1 < resources.size() ? ", " : "");
    }
    std::cout << "] ";
    std::cout << "Reachable: [";
    for (int i = 0; i < graph.num_nodes; ++i) {
        if (!(visited & (1ULL << i))) {
            std::cout << i << " ";
        }
    }
    std::cout << "] ";
    std::cout << "Status: ";
    switch (status) {
    case LabelStatus::NEW_OPEN:    std::cout << "NEW_OPEN "; break;
    case LabelStatus::NEW_CLOSED:  std::cout << "NEW_CLOSED "; break;
    case LabelStatus::OPEN:        std::cout << "OPEN "; break;
    case LabelStatus::CLOSED:      std::cout << "CLOSED "; break;
    case LabelStatus::DOMINATED:   std::cout << "DOMINATED "; break;
    }
    std::cout << "ID (vertex, id): ( " << vertex << " , " << id << " )" << std::endl;
    std::cout << "=========================\n\n";
}

DominanceStatus Label::DominanceCheck(const Label& rival) const {
    if (vertex != rival.vertex|| direction != rival.direction) return DominanceStatus::INCOMPARABLE;
    bool new_label_dominates = true, new_label_lose = true;
    for (size_t i = 0; i < resources.size(); ++i) {
        if (resources[i] > rival.resources[i]) new_label_dominates = false;
        if (resources[i] < rival.resources[i]) new_label_lose = false;
        if (!new_label_dominates && !new_label_lose) return DominanceStatus::INCOMPARABLE;
    }

    if ((visited&rival.visited)!=visited) new_label_dominates = false;
    if ((visited&rival.visited)!=rival.visited) new_label_lose = false;

    if (!new_label_dominates && !new_label_lose) return DominanceStatus::INCOMPARABLE;
    
    if (new_label_dominates && cost <= rival.cost) return DominanceStatus::DOMINATES;
    if (new_label_lose && cost >= rival.cost) return DominanceStatus::DOMINATED;
    return DominanceStatus::INCOMPARABLE;
}

bool Label::isConcatenable(const Label& label, const std::vector<double>& r_max) const {
    if ((vertex != label.vertex) || (direction == label.direction)|| (checked_with.find(label.id) != checked_with.end())) {
        return false;
    }
    for (size_t i = 0; i < resources.size(); ++i) {
        if (resources[i] + label.resources[i] > r_max[i]) {
            return false;
        }
    }

    uint64_t common = (visited & label.visited)&~(1ULL << vertex) &~(1ULL<<0);
    return common == 0;
}
