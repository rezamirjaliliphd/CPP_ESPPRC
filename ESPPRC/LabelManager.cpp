#include "LabelManager.h"
#include <iostream>
#include <memory>

LabelManager::LabelManager(int num_nodes, int num_res, Graph& graph) {
    //std::cout << "Create Labels at source and sink" << std::endl;
    initializeLabels(num_nodes, num_res, graph);
}

void LabelManager::initializeLabels(int num_nodes, int num_res, Graph& graph) {

    labelQueue.push(const_cast<Label*>(&(*Labels[0].emplace(Label(graph)).first)));

}


void LabelManager::DominanceCheckInsert(Label& label) {
    int v = label.vertex;
    bool isDominated = false;
	if (Labels.find(v) == Labels.end()) {
        label.id = find_ID(v);
		label.status = LabelStatus::OPEN;
        labelQueue.push(const_cast<Label*>(&(*Labels[v].emplace(std::move(label)).first)));
		return;
	}
	//std::cout << "Line 34" << std::endl;
    for (auto it = Labels[v].begin(); it != Labels[v].end() && it->status != LabelStatus::CLOSED; ) {
		DominanceStatus status = it->DominanceCheck(label); // existing_label.DominanceCheck(new_label)
        if (status == DominanceStatus::DOMINATED)
        {
			//std::cout << "Existing Label is dominated" << std::endl;
            //std::cout << " Line 32" << std::endl;
            invalidLabels.insert(const_cast<Label*>(&(*it)));
            //std::cout << " Line 33" << std::endl;
            it = Labels[v].erase(it);
        }
        else if (status == DominanceStatus::DOMINATES) {
            isDominated = true;
			//std::cout << "New Label is dominated" << std::endl;
            return;
        }
        else {
			//std::cout << " Line 43" << std::endl;
            ++it;
        }
    }
    if (!isDominated) {
        //std::cout << " Line 46" << std::endl;
        label.id = find_ID(v);
        //std::cout << " Line 47" << std::endl;
        label.status = (label.status == LabelStatus::NEW_CLOSED) ? LabelStatus::CLOSED : LabelStatus::OPEN;
		//label.display(); std::cout << " Line 48" << std::endl;
        if (label.status == LabelStatus::OPEN) {
            labelQueue.push(const_cast<Label*>(&(*Labels[v].emplace(std::move(label)).first)));
            //std::cout << " Line 50" << std::endl;
		}
		else if (label.status == LabelStatus::CLOSED) {
			Labels[v].emplace(std::move(label)); 
            //std::cout << " Line 53" << std::endl;
		}
    }
    else {
        label.status = LabelStatus::DOMINATED; 
        //std::cout << " Line 57" << std::endl;
    }
}


bool LabelManager::isIDDuplicate(const int vertex, const long long fw_id, const long long bw_id) const {
   
    return IDs.find(std::make_tuple(vertex,fw_id,bw_id))!=IDs.end(); 
    //std::cout << " Line 64" << std::endl;
}


long long LabelManager::find_ID(const int vertex) const {
    //std::cout << " Line 70" << std::endl;

    auto it = Labels.find(vertex);
    if (it == Labels.end() || it->second.empty()) {
        return 1;  // If no labels exist for this vertex, return 1 as the first ID
    }

    //std::cout << " Line 73" << std::endl;
    return it->second.rbegin()->id + 1;  // Get the highest ID + 1
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
                    //std::cout << "New UB: " << UB << std::endl;
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
	while (!labelQueue.empty() && invalidLabels.count(labelQueue.top())) {
		//std::cout << "Line 122" << std::endl;
        labelQueue.pop();  // Remove invalid labels
    }
    
    //std::cout << "Line 127" << std::endl;
    if (labelQueue.empty()) return;  // No valid labels left

    // Get the top valid label
    //std::cout << "Line 131" << std::endl;
    Label* parentLabel = labelQueue.top();
    int vertex = parentLabel->vertex;
    labelQueue.pop(); //std::cout << " Line 135" << std::endl; // Remove from queue (but still exists in Labels)
    if (parentLabel->LB <= UB) {
        //std::cout << "Line 135" << std::endl;
        
        for (const auto edge : graph.OutList[parentLabel->vertex]) {
            //std::cout << "Line 140" << std::endl;
            if (parentLabel->reachable[edge->to]) {
                //std::cout << "adding edge (" << edge->from << ", " << edge->to << ")" << std::endl;
                Label new_label(*parentLabel, graph, edge.get(), UB);

                //std::cout << " Line 135" << std::endl;
                if (new_label.status != LabelStatus::DOMINATED) {
                    DominanceCheckInsert(new_label);
                }
                //std::cout << "Size of queue: " << labelQueue.size() << std::endl;
            }
        }
    }
    // Mark parentLabel as removed
    invalidLabels.insert(parentLabel);

    // Remove from Labels[v]
    Labels[vertex].erase(*parentLabel);
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
    while (!labelQueue.empty()) {
        Propagate(graph);
        concatenateLabels(graph);

    }
    //std::cout << "Solutions: " << std::endl;
    //displaySolutions();
}



