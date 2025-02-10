#include "LabelManager.h"
#include <iostream>
#include <memory>

LabelManager::LabelManager(int num_nodes, int num_res, Graph& graph) {
    //std::cout << "Create Labels at source and sink" << std::endl;
    initializeLabels(num_nodes, num_res, graph);
}

void LabelManager::initializeLabels(int num_nodes, int num_res, Graph& graph) {
    auto initialize = [&](bool dir, std::map<int, std::set<Label, CompareLabel>>& labels) {
        std::cout << "Initialize Labels" << std::endl;
        Label initial_label(graph, dir);
        initial_label.status = LabelStatus::OPEN;
        initial_label.id = 0;
        labels[0].insert(initial_label);
        };
    initialize(true, fw_labels);
    initialize(false, bw_labels);
}


void LabelManager::DominanceCheckInsert(Label& label) {
    int v = label.vertex;
    auto& labels = label.direction? fw_labels[v] : bw_labels[v];

    bool isDominated = false;
    for (auto it = labels.begin(); it != labels.end(); ) {
		DominanceStatus status = it->DominanceCheck(label); // existing_label.DominanceCheck(new_label)

        if (status == DominanceStatus::DOMINATED) {
            it = labels.erase(it);
        }
        else if (status == DominanceStatus::DOMINATES) {
            isDominated = true;
            return;
        }
        else {
            ++it;
        }
    }
    if (!isDominated) {
        label.id = find_ID(label.direction, label.vertex);
		//label.display();
        label.status = (label.status == LabelStatus::NEW_CLOSED) ? LabelStatus::CLOSED : LabelStatus::OPEN;
        labels.emplace(std::move(label));

    }
    else {
        label.status = LabelStatus::DOMINATED;
    }
}


bool LabelManager::isIDDuplicate(const int vertex, const long long fw_id, const long long bw_id) const {
    for (const Solution& solution : solutions) {
        if (solution.ID == std::make_pair(vertex, std::make_pair(fw_id, bw_id))) {
            return true;
        }
    }
    return false;
}


long long LabelManager::find_ID(bool direction, const int vertex) const {
    const auto& label_map = direction ? fw_labels : bw_labels;

    // Check if the vertex exists in the map
    auto it = label_map.find(vertex);
    if (it == label_map.end()) {
        return 1;  // If no labels exist for this vertex, return 1 as the first ID
    }

    const auto& labels = it->second;

    // If the set is empty, return 1 as the first ID
    if (labels.empty()) {
        return 1;
    }

    // Use rbegin() to access the label with the highest ID
    return labels.rbegin()->id + 1;
}


void LabelManager::displayLabels() const {
    for (const auto& pair : fw_labels) {
        for (const Label& label : pair.second) {
            label.display();
        }
    }
    for (const auto& pair : bw_labels) {
        for (const Label& label : pair.second) {
            label.display();
        }
    }
}


void LabelManager::concatenateLabels(const std::vector<double>& res_max) {
    std::vector<int> path;
    #pragma omp parallel for private(path)
    for (auto& [vertex, f_labels] : fw_labels) {
        if (vertex == 0) continue;

        // Find the corresponding backward labels for this vertex
        auto bw_it = bw_labels.find(vertex);
        if (bw_it == bw_labels.end()) continue;

        auto& b_labels = bw_it->second;

        for (auto fit = f_labels.begin(); fit != f_labels.end(); ++fit) {
            /*if (fit->status != LabelStatus::OPEN) continue;*/

            for (auto bit = b_labels.begin(); bit != b_labels.end(); ++bit) {
                //if (bit->status != LabelStatus::OPEN) continue;

                // Skip if IDs are duplicate or labels are not concatenable
                if (isIDDuplicate(vertex, fit->id, bit->id) || !fit->isConcatenable(*bit, res_max)) continue;

                // Concatenate paths efficiently
                path = fit->path;
                path.insert(path.end(), bit->path.begin() + 1, bit->path.end());

                // Calculate the new cost
                double cost = fit->cost + bit->cost;
				/*std::cout << "Cost: " << cost << std::endl;
				print_vector(path);*/
                // Check and update UB
                #pragma omp critical
                if (cost < UB) {
                    solutions.emplace_back(Solution(path, cost, std::make_pair(vertex, std::make_pair(fit->id, bit->id))));
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



void LabelManager::Propagate(Graph& graph, const std::vector<double>& res_max) { // Farzane: passed mip pointer
      //std::cout << "Line 130" << std::endl;

      auto propagateDirection = [&](std::map<int, std::set<Label, CompareLabel>>& label_map,
          bool direction) {
              
			  //std::cout << "Line 135" << std::endl;
              for (auto& [vertex, labels] : label_map) {
				  //std::cout << "Vertex: " << vertex << std::endl;   
                  for (auto it = labels.begin(); it != labels.end();  ++it) {
						  //std::cout << "Label: " << it->id << " vertex " << it->vertex << std::endl;
						  /*if (direction) { std::cout << "Direction: Forward" << std::endl; }
						  else { std::cout << "Direction: Backward" << std::endl; }*/
                      if (it->status == LabelStatus::OPEN) {
                          for (const auto edge : graph.getNeighbors(it->vertex, direction)) {
                              //std::cout << "Edge: " << edge.from << " " << edge.to << std::endl;
                              if ((direction && it->reachable[edge->to]) ||
                                  (!direction && it->reachable[edge->from])) {
                                  Label new_label(*it, graph, edge.get(), UB); // Farzane: passed mip pointer
                                  
								  if (new_label.status != LabelStatus::DOMINATED) DominanceCheckInsert(new_label);

                              }
                          }

                          // Remove and re-insert to update the status
                          /*Label modified_label = *it;
                          modified_label.status = LabelStatus::CLOSED;
                          it = labels.erase(it);
                          labels.insert(modified_label);*/
                          const_cast<Label&>(*it).status = LabelStatus::CLOSED;
                          //const_cast<Label&>(*it).model = nullptr;
                      }
                      
                      
                  }
              }
              
          };

      propagateDirection(fw_labels, true);
      propagateDirection(bw_labels, false);

      
  }


bool LabelManager::Terminate() {
    for (auto& [vertex, labels] : fw_labels) {
        for (auto& label : labels) {
            if (label.status == LabelStatus::OPEN) {
                return false;
            }
        }
    }
    for (auto& [vertex, labels] : bw_labels) {
        for (auto& label : labels) {
            if (label.status == LabelStatus::OPEN) {
                return false;
            }
        }
    }
    return true;
}


void LabelManager::Run(Graph& graph, const std::vector<double>& res_max) {
    while (!Terminate()) {
        Propagate(graph, res_max);
        concatenateLabels(res_max);
    }
    std::cout << "Solutions: " << std::endl;
    displaySolutions();
}



