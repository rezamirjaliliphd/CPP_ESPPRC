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
        status = LabelStatus::CLOSED;
        open = false;
    }
    else {
        status = LabelStatus::OPEN;
    }

    if (LB > 100) {
        status = LabelStatus::DOMINATED;
        //std::cout << "Pruned" << std::endl;
    }
}


Label::Label(const std::shared_ptr<Label>& parent_ptr, Graph& graph, const Edge* edge, const double UB)
    : path({0}), direction(parent_ptr->direction), cost(parent_ptr->cost+edge->cost),
    resources(parent_ptr->resources), visited(parent_ptr->visited),checked_with(parent_ptr->checked_with) {
    vertex = direction? edge->head:edge->tail;
    path = parent_ptr->path;
    if (direction) {path.push_back(vertex);}else{path.insert(path.begin(), vertex);}
    
    visited |= (1ULL << vertex);

    for (size_t i = 0; i < resources.size(); ++i) resources[i] += edge->resources[i];
    UpdateReachable(graph);
    UpdateLB(graph);
    

    if (reachHalfPoint(graph.res_max, graph.num_nodes)) {
        status = LabelStatus::CLOSED;
        open = false;}
    else {
        status = LabelStatus::OPEN;
    }

    if (LB > UB) {
        status = LabelStatus::DOMINATED;
        deleted = true;
        open = false;
    }
        //std::cout << "Pruned" << std::endl;
}


void Label::UpdateReachable(Graph& graph) {
    int next_vertex= -1;
    for (const auto& e : graph.getNeighbors(vertex, direction)) {
        next_vertex = direction? e->head:e->tail;
        if (visited & (1ULL << next_vertex)) continue;
        for (int k = 0; k < graph.num_res; k++) {
            if (resources[k] + e->resources[k] > graph.res_max[k]) {
                visited |= (1ULL << next_vertex);
                break;
            }
        }
    }
}


bool Label::reachHalfPoint(const std::vector<double>& res_max, int num_nodes) {
    // if (static_cast<double>(path.size())>= static_cast<double>(num_nodes+2) / 2+0.5) return true;
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
    int neighbor = -1;
    uint64_t visited_ = visited &(~(1ULL << vertex)); // Exclude current vertex from visited set
    for (int i = 0; i < graph.num_nodes; i++) {
        if (!(visited_ & (1ULL << i))) {
            lb = 0;
            for (const auto& e : direction? graph.OutList[i]: graph.InList[i]) {
                neighbor = direction? e->head:e->tail;
                if ( lb>e->cost && !(visited_ & (1ULL << neighbor))) lb = e->cost;
            }
            if (lb < 0) LB += lb;

        }
    }
    LB = -1000;
    
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
    std::cout << "Open: " << (open ? "true" : "false") << std::endl;
    std::cout << "ID (vertex, id): ( " << vertex << " , " << id << " )" << std::endl;
    std::cout << "=========================\n\n";
}

DominanceStatus Label::DominanceCheck(const std::shared_ptr<Label>& rival_ptr) const {
    // if (vertex != rival_ptr->vertex || direction != rival_ptr->direction) 
    //     return DominanceStatus::INCOMPARABLE;
    
    // Check resource dominance
    bool this_win = true;
    bool rival_win = true;
    
    for (size_t i = 0; i < resources.size(); ++i) {
        if (resources[i] > rival_ptr->resources[i]) this_win = false;
        if (rival_ptr->resources[i] > resources[i]) rival_win = false;
        if (!this_win && !rival_win) {
            return DominanceStatus::INCOMPARABLE; // Neither dominates
        }
    }
    
    // Check visited set dominance (subset relationship)
    if (this_win && cost<=rival_ptr->cost && (visited & rival_ptr->visited) == visited ) return DominanceStatus::DOMINATES;  // this ⊆ rival
    else if (rival_win && rival_ptr->cost <= cost && (rival_ptr->visited & visited) == rival_ptr->visited) return DominanceStatus::DOMINATED; // rival ⊆ this   
    return DominanceStatus::INCOMPARABLE;
}

bool Label::isConcatenable(const std::shared_ptr<Label>& partner_ptr, const std::vector<double>& r_max) const {

    for (size_t i = 0; i < resources.size(); ++i) {
        if (resources[i] + partner_ptr->resources[i] > r_max[i]) {
            return false;
        }
    }

    uint64_t common = (visited & partner_ptr->visited)&~(1ULL << vertex) &~(1ULL<<0);
    return common == 0;
}
