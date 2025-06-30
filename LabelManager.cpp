#include "LabelManager.h"
#include <iostream>
#include <memory>

LabelManager::LabelManager(Graph& graph) {
    HuristicUB(graph);
    // std::cout << "Huristic UB: " << UB << std::endl;
    for (int i = 0; i < graph.num_nodes; ++i) {
        F_set[i] = std::set<std::shared_ptr<Label>, LabelPtrSetComparator>();
        B_set[i] = std::set<std::shared_ptr<Label>, LabelPtrSetComparator>();
    }
    // std::cout << "Create Labels at source and sink" << std::endl;
    std::shared_ptr<Label> source = std::make_shared<Label>(graph,true);
    ID++;
    source->id = ID;
    F_set.at(0).insert(source);
    F_Open.push_back(source);
    std::push_heap(F_Open.begin(), F_Open.end(), LabelPtrHeapComparator());
    std::shared_ptr<Label> sink = std::make_shared<Label>(graph,false);
    ID++;
    sink->id = ID;
    B_set.at(0).insert(sink);
    B_Open.push_back(sink);
    std::push_heap(B_Open.begin(), B_Open.end(), LabelPtrHeapComparator());
    // std::cout<< "Sink and source labels created" << std::endl;
    // std::cout << "Forward Open Labels: " << F_Open.size() << std::endl;
    // std::cout << "Backward Open Labels: " << B_Open.size() << std::endl;
}

void LabelManager::HuristicUB(Graph& graph) {
    
    bool add_to_UB = false;
    for (int i = 1; i < graph.num_nodes; ++i) {
        add_to_UB = true;
        for (size_t j = 0; j < graph.num_res; ++j) {
            if (graph.edges[0,i]->resources[j]+graph.edges[i,0]->resources[j] > graph.res_max[j]) {
                add_to_UB = false;
                break;
            }
        }
        if (graph.edges[0,i]->cost+graph.edges[i,0]->cost < UB && add_to_UB) {
            UB = graph.edges[0,i]->cost + graph.edges[i,0]->cost;
            solutions.push_back(Solution({0, i, 0}, UB));
        }
        for (int k= 1; k < graph.num_nodes; ++k) {
            if (i == k || !graph.predecessor[i][k]) continue;
            add_to_UB = true;
            for (size_t j = 0; j < graph.num_res; ++j) {
                if (graph.edges[0,i]->resources[j] + graph.edges[i,k]->resources[j] + graph.edges[k,0]->resources[j] > graph.res_max[j]) {
                    add_to_UB = false;
                    break;
                }
            }
            if (graph.edges[0,i]->cost + graph.edges[i,k]->cost + graph.edges[k,0]->cost < UB && add_to_UB) {
                UB = graph.edges[0,i]->cost + graph.edges[i,k]->cost + graph.edges[k,0]->cost;
                solutions.push_back(Solution({0, i, k, 0}, UB));
            }
        }
    }

    // std::cout << "Huristic UB: " << UB << std::endl;
}


void LabelManager::Propagate(Graph& graph) {

    //if (labelHeap.front().status != LabelStatus::OPEN) return;
	int neighbor;
    int vertex = -1;
    DominanceStatus status;
    std::shared_ptr<Label> parenet_ptr , newLabel;
    for (bool dir : {true, false}) {
        std::vector<std::shared_ptr<Label>>& labelHeap= dir ? F_Open : B_Open;    
        while (labelHeap.size()<=1000 && !labelHeap.empty()) {
            std::pop_heap(labelHeap.begin(), labelHeap.end(), LabelPtrHeapComparator());  // Move best label to end
            parenet_ptr = labelHeap.back();  // Store the best label
            vertex = parenet_ptr->vertex;
            labelHeap.pop_back();  // Remove from heap
            for (const auto& edge : dir?graph.OutList[parenet_ptr->vertex]: graph.InList[parenet_ptr->vertex]) {
                neighbor = dir ? edge->head : edge->tail;
                if (parenet_ptr->visited & (1ULL << neighbor)) continue;  // Skip if already visited
                newLabel= std::make_shared<Label>(parenet_ptr, graph, edge.get(), UB);  // Create new label
                if (newLabel->deleted) continue;;  // Store in temporary heap
                std::set<std::shared_ptr<Label>, LabelPtrSetComparator>& labelSet = dir ? F_set[neighbor] : B_set[neighbor];
                if (labelSet.empty()){
                    ID++;
                    newLabel->id = ID;  // Assign new ID
                    if (dir){
                        F_concat_need[neighbor] = true;  // Mark as needing concatenation
                    }else{
                        B_concat_needed[neighbor] = true;  // Mark as needing concatenation
                    }
                    if (newLabel->open) labelHeap.push_back(newLabel);  // Add to heap if open
                    labelSet.insert(newLabel);  // Insert into the set
                }else{
                    for (auto it=labelSet.begin(); it != labelSet.end(); ) {
                        status = newLabel->DominanceCheck(*it);  // Check dominance
                        if (status == DominanceStatus::DOMINATED) {
                            newLabel->status = LabelStatus::DOMINATED;  // Mark as dominated
                            newLabel->deleted = true;  // Mark as deleted
                            break;  // Exit loop if dominated
                        } else if (status == DominanceStatus::DOMINATES) { // New label dominates existing label
                            if ((*it)->open) {
                                // Remove dominated label from the heap
                                labelHeap.erase(std::remove_if(labelHeap.begin(), labelHeap.end(),
                                    [&](const std::shared_ptr<Label>& lptr) {
                                        return lptr == (*it);
                                    }), labelHeap.end());
                            }
                            it = labelSet.erase(it); // Remove dominated label from the set
                            // std::cout << "New Label dominates label " << (*it)->id << std::endl;
                        } else if (status == DominanceStatus::INCOMPARABLE) {
                            ++it; // Keep the label in the set
                        }
                    }
                    if (newLabel->deleted) continue;  // Skip if deleted
                    ID++;
                    newLabel->id = ID;  // Assign new ID
                    newLabel->status = (newLabel->open) ? LabelStatus::OPEN : LabelStatus::CLOSED;  // Set status
                    labelSet.insert(newLabel);  // Insert into the set
                    if (dir){
                        F_concat_need[neighbor] = true;  // Mark as needing concatenation
                    }else{
                        B_concat_needed[neighbor] = true;  // Mark as needing concatenation
                    }
                    if (newLabel->open) labelHeap.push_back(newLabel);  // Add to heap if open
                }
                
            }
            labelHeap.erase(std::remove_if(labelHeap.begin(), labelHeap.end(),
                    [](const std::shared_ptr<Label>& lptr) {
                    return lptr->deleted;
                    }), labelHeap.end());
            std::make_heap(labelHeap.begin(), labelHeap.end(), LabelPtrHeapComparator());
        }
    }
}
        



// void LabelManager::DominanceCheckInsert(std::shared_ptr<Label>& label_ptr, Graph& graph) {
//     int v = label_ptr->vertex;
// 	std::vector<std::shared_ptr<Label>>& heap = label_ptr->direction ? F_Open : B_Open;
//     std::set<std::shared_ptr<Label>, LabelPtrSetComparator>& Set = label_ptr->direction ? F_set[v] : B_set[v];
//     //std::vector<Label> tempHeap;
    
//     for (auto it = Set.begin(); it !=  Set.end(); ) {
//         DominanceStatus status = label_ptr->DominanceCheck(*it);
//         if (status == DominanceStatus::DOMINATED) {
//             label_ptr->status = LabelStatus::DOMINATED; // Mark label as dominated
//             label_ptr->deleted = true; // Mark label as deleted
//             // std::cout << "New Label is dominated by label " << (*it)->id << std::endl;
//             return;
//         }else if (status == DominanceStatus::DOMINATES) {//new label dominates existing label
//             auto dominated_label = *it; // Store the dominated label
//             it = Set.erase(it); // Remove dominated label
//             heap.erase(std::remove_if(heap.begin(), heap.end(),
//                 [&](const std::shared_ptr<Label>& lptr) {
//                     return lptr->id == dominated_label->id;
//                 }), heap.end());
//             // std::cout << "New Label dominates label " << (*it)->id << std::endl;
//         } else if (status == DominanceStatus::INCOMPARABLE) {
//             ++it; // Keep the label in the heap
//         }
//     }

    
//     label_ptr->status = (label_ptr->open) ? LabelStatus::OPEN : LabelStatus::CLOSED;
    
//     ID++;
//     label_ptr->id = ID;
//     Set.insert(label_ptr); // Insert into the set
//     if (label_ptr->open){
//         heap.push_back(label_ptr);
//         //std::cout << " Line 46" << std::endl;
        
//     }
//     std::push_heap(heap.begin(), heap.end(), LabelPtrHeapComparator());    
//     label_ptr->direction? F_concat_need[label_ptr->vertex] = true : B_concat_needed[label_ptr->vertex] = true;
// }





void LabelManager::displayLabels(Graph& graph) const {
    for (int i = 0; i< graph.num_nodes; ++i) {
        if (F_set.find(i) == F_set.end()) continue;
        for (const auto& label_ptr : F_set.at(i)) {
            label_ptr->display(graph);
        }
        if (B_set.find(i) == B_set.end()) continue;
        for (const auto& label_ptr : B_set.at(i)) {
            label_ptr->display(graph);
        }
    }
    
}

double calculateCost(const std::shared_ptr<Label>& L_1, const std::shared_ptr<Label>& L_2, const Graph& graph) {
    double cost = L_1->cost + L_2->cost;
    for (size_t i = graph.phi_ID; i < graph.num_res; ++i) {
        cost += std::floor(L_1->resources[i] + L_2->resources[i])* graph.Pi[i-graph.phi_ID];
    }
    return cost;
}
void LabelManager::concatenateLabels(const Graph& graph) {
    std::vector<int> path;
    double cost_ = 0.0;
    bool new_UB = false;
    double ub = this->UB;
    for (int v = 1;v<graph.num_nodes; ++v){
        if (F_set.find(v) == F_set.end() || B_set.find(v) == B_set.end()) continue;
        for (auto fit = F_set[v].begin(); fit != F_set[v].end(); ++fit) {
            // std::cout << "Concatenating labels for vertex " << v << std::endl;
           
            if (calculateCost(*fit, *B_set[v].begin(),graph)>= ub) {
                // std::cout << "Lower bound: " << ((*fit)->cost + (*B_set[v].begin())->cost) << " UB: " << ub << std::endl;
                break;
                 
            } // Skip if the label cost exceeds UB
            for (auto bit = B_set[v].begin(); bit != B_set[v].end(); ++bit) {
            
                if (calculateCost(*fit,*bit,graph) >= ub) break; // Skip if the pair exceeds UB
                // else 
                if ((*fit)->isConcatenable(*bit, graph.res_max)) {
                    // std::cout << "Concatenating labels for vertex " << v << std::endl;
                    
                    cost_ = calculateCost(*fit, *bit, graph);
                    if (cost_ < ub) { // Skip if the concatenated cost exceeds UB
                    new_UB = true;
                    ub = cost_;
                    path = (*fit)->path;
                    path.insert(path.end(), (*bit)->path.begin()+1, (*bit)->path.end());
                    solutions.push_back(Solution(path, cost_));
                    // std::cout << "New solution found with cost: " << ub << std::endl;
                    }
                   
                }
            
            }
        }
    }
    if (new_UB) {
        this->UB = ub;
        // std::cout << "New UB found: " << UB << std::endl;
        PruneLabels(graph);
    }

    
}


void LabelManager::displaySolutions() const {
    for (const Solution& solution : solutions) {
        solution.display();
    }
}


void LabelManager::PruneLabels(const Graph& graph) {
    // std::cout << "Pruning Labels..." << std::endl;
    for (bool dir : {true, false}) {
        std::vector<std::shared_ptr<Label>>& labelHeap = dir ? F_Open : B_Open;
         for (auto it = labelHeap.begin(); it != labelHeap.end();) {
            if ((*it)->LB > UB) {
                it = labelHeap.erase(it);
            } else {
                ++it;
            }
        }
        labelHeap.erase(std::remove_if(labelHeap.begin(), labelHeap.end(),
                [](const std::shared_ptr<Label>& lptr) {
                    return lptr->deleted;
                }), labelHeap.end());
        std::make_heap(labelHeap.begin(), labelHeap.end(), LabelPtrHeapComparator());
        labelHeap.shrink_to_fit(); // Optional: shrink the vector to release unused memory
        for (int v=0; v<graph.num_nodes; ++v){
            std::set<std::shared_ptr<Label>, LabelPtrSetComparator>& labelSet = dir ? F_set[v] : B_set[v];
            for (auto it = labelSet.begin(); it != labelSet.end();) {
                if ((*it)->LB > UB) {
                    it = labelSet.erase(it);
                } else {
                    ++it;
                }
            }
            
        }
    }
}



bool LabelManager::Terminate() {
    // std::cout << "Checking Termination Condition..." << std::endl;
    // std::cout << "will Terminate: " << !(F_Open.empty() || B_Open.empty()) << std::endl;
    return (F_Open.empty() && B_Open.empty());
}


void LabelManager::Run(Graph& graph) {
    // std::cout << "Running ESPPRC..." << std::endl;
    while (!Terminate()) {
        // std::cout << "Propagation" << std::endl;
        Propagate(graph);
        concatenateLabels(graph);

    }
    concatenateLabels(graph);
    // std::cout << "Heap Labels: " << std::endl;
    // displayLabels(graph);
    // displayLabels(graph);<
    // std::cout << "Deleted Labels:" << std::endl;
    // for (const auto& lbl: F_dominated) {
    //     lbl.display(graph);
    // }
    // for (const auto& lbl: B_dominated) {
    //     lbl.display(graph);
    // }
    // std::cout << "==========================\n";
    // std::cout<< "Forward Labels: " <<std::endl;
    // for (auto& it = F_Heap.begin(); it != F_Heap.end(); ++it) {
    //     it->display(graph);
    // }
    // std::cout << "==========================\n";
    // std::cout<< "Backward Labels: " <<std::endl;
    // for (auto& it = B_Heap.begin(); it != B_Heap.end(); ++it) {
    //     it->display(graph);
    // }
    // displayLabels(graph);
    std::cout << "Solutions: " << std::endl;
    displaySolutions();
}

std::vector<std::pair<std::vector<int>,double>> LabelManager::getSolutions() const {
    std::vector<std::pair<std::vector<int>, double>> result;
    for (const auto& sol : solutions) {
        result.emplace_back(sol.path, sol.cost);
    }
    return result;
}
