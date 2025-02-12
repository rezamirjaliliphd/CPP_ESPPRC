#include "LabelManager.h"
#include <iostream>
#include <memory>

LabelManager::LabelManager(int num_nodes, int num_res, Graph& graph) {
    //std::cout << "Create Labels at source and sink" << std::endl;
    initializeLabels(num_nodes, num_res, graph);
}

void LabelManager::initializeLabels(int num_nodes, int num_res, Graph& graph) {

	Labels[0].emplace(Label(graph));
	Labels[0].begin()->display();
    best_open_label = 0;
}


void LabelManager::DominanceCheckInsert(Label& label) {
    int v = label.vertex;
   
    bool isDominated = false;
	if (Labels.find(v) == Labels.end()) {
		Labels[v].emplace(std::move(label));
		if (label.LB < Labels[best_open_label].begin()->LB) {
			best_open_label = v;
		}
		return;
	}
    for (auto it = Labels[v].begin(); it != Labels[v].end() && it->status != LabelStatus::CLOSED; ) {
		DominanceStatus status = it->DominanceCheck(label); // existing_label.DominanceCheck(new_label)
        if (status == DominanceStatus::DOMINATED) it = Labels[v].erase(it);
        else if (status == DominanceStatus::DOMINATES) {
            isDominated = true;
            return;
        }
        else {
            ++it;
        }
    }
    if (!isDominated) {
        label.id = find_ID(v);
		label.display();
        label.status = (label.status == LabelStatus::NEW_CLOSED) ? LabelStatus::CLOSED : LabelStatus::OPEN;
        Labels[v].emplace(std::move(label));
		if (label.LB < Labels[best_open_label].begin()->LB) {
			best_open_label = v;
		}

    }
    else {
        label.status = LabelStatus::DOMINATED;
    }
}


bool LabelManager::isIDDuplicate(const int vertex, const long long fw_id, const long long bw_id) const {
   
    return IDs.find(std::make_tuple(vertex,fw_id,bw_id))!=IDs.end();
}


long long LabelManager::find_ID(const int vertex) const {
    // Check if the vertex exists in the map
    if (Labels.find(vertex) == Labels.end())  return 1;  // If no labels exist for this vertex, return 1 as the first ID
    // Use rbegin() to access the label with the highest ID
    return Labels.at(vertex).rbegin()->id + 1;
}


void LabelManager::displayLabels() const {
    for (const auto& pair : Labels) {
        for (const Label& label : pair.second) label.display();
    }
}


void LabelManager::concatenateLabels(const Graph& graph) {
    std::vector<int> path;
    #pragma omp parallel for private(path)
    for (int v = 1; v < graph.num_nodes; v++) {
        for (auto fit = Labels[v].begin(); fit != Labels[v].end(); ++fit) {
            for (auto bit = std::next(fit); bit != Labels[v].end(); ++bit) {
                
                if (isIDDuplicate(v, fit->id, bit->id) || !fit->isConcatenable(*bit, graph.res_max)) continue;

                // Concatenate paths efficiently
                path = fit->path;
                path.insert(path.end(), bit->path.rbegin() + 1, bit->path.rend());

                // Calculate the new cost
                double cost = fit->cost + bit->cost;
				/*std::cout << "Cost: " << cost << std::endl;
				print_vector(path);*/
                // Check and update UB
                #pragma omp critical
                if (cost < UB) {
                    solutions.emplace_back(Solution(path, cost, { v,fit->id, bit->id }));
                    UB = cost;
                    std::cout << "New UB: " << UB << std::endl;
                }
            }
        }
    }
}


void LabelManager::displaySolutions() const {
    for (const Solution& solution : solutions) {
        solution.display();
    }
}



void LabelManager::Propagate(Graph& graph) {    
	int v = best_open_label;
	std::cout << "Propagating from best label's vertex " << v << std::endl;
    for (auto it = Labels[v].begin(); it != Labels[v].end();) {		
        if (it->status == LabelStatus::OPEN) {
            for (const auto edge : graph.OutList[it->vertex]) {
				std::cout << "adding edge from " << it->vertex << " to " << edge->to << std::endl;
                if (it->reachable[edge->to]) {
                    Label new_label(*it, graph, edge.get(), UB);
					new_label.display();
					if (new_label.status != LabelStatus::DOMINATED) DominanceCheckInsert(new_label);

                }
            }
        Labels[v].erase(it);
        break;           
		}
		else  ++it;                        
    }   
  }


bool LabelManager::Terminate(Graph& graph) {
    for (int v = 0; v < graph.num_nodes; v++) {
        if (Labels[v].empty()) continue;
        for (const Label& label : Labels[v]) {
            if (label.status == LabelStatus::OPEN) return false;
        }
    }
    return true;
}


void LabelManager::Run(Graph& graph) {
    while (!Terminate(graph)) {
        Propagate(graph);
        concatenateLabels(graph);
    }
    std::cout << "Solutions: " << std::endl;
    displaySolutions();
}



