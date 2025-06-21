#include "LabelManager.h"
#include <iostream>
#include <memory>

LabelManager::LabelManager(Graph& graph) {
    HuristicUB(graph);
    // std::cout << "Create Labels at source and sink" << std::endl;
    Label source(graph,true);
    // source.display(graph);
    DominanceCheckInsert(source, graph);
	Label sink(graph, false);
    // sink.display(graph);
    DominanceCheckInsert(sink, graph);
    // std::cout<< "Sink and source labels created" << std::endl;
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
    bool continuePropagation = true;
    // Get the top valid label
    //std::cout << "Line 131" << std::endl;
    while (continuePropagation){

        for (bool dir : {true, false}) {
            std::vector<Label>& labelHeap = dir ? F_Heap : B_Heap;
            if (!labelHeap.front().open) continue;
            std::pop_heap(labelHeap.begin(), labelHeap.end(), CompareLabel());  // Move best label to end
            Label parentLabel = labelHeap.back();  // Store the best label
            // std::cout << "Parent Label: " << std::endl;
            // parentLabel.display(graph);
            labelHeap.pop_back();  // Remove from heap
            
            // Step 2: Process the best label (propagate children labels)
            for (const auto& edge : dir?graph.OutList[parentLabel.vertex]: graph.InList[parentLabel.vertex]) {
                neighbor = dir ? edge->head : edge->tail;
                if (parentLabel.visited & (1ULL << neighbor)) continue;  // Skip if already visited
                // std::cout << "Neighbor: " << neighbor << std::endl;
                Label newLabel(parentLabel, graph, edge.get(), UB);  // Create new label
                // newLabel.display(graph);
                if (newLabel.status != LabelStatus::DOMINATED) DominanceCheckInsert(newLabel, graph);
                }
            parentLabel.open = false;  // Close the parent label
            parentLabel.status = LabelStatus::CLOSED;
            labelHeap.push_back(parentLabel);  // Reinsert the parent label
            std::push_heap(labelHeap.begin(), labelHeap.end(), CompareLabel());
        }
    continuePropagation = (F_Heap.front().open || B_Heap.front().open);
    
    }
}


void LabelManager::DominanceCheckInsert(Label& label, Graph& graph) {
    int v = label.vertex;
	std::vector<Label>& heap = label.direction ? F_Heap : B_Heap;
    std::vector<Label>& dominated = label.direction ? F_dominated : B_dominated;
    //std::vector<Label> tempHeap;
    if (heap.empty()) {
        ID++;
        label.id = ID;
        label.status = LabelStatus::OPEN;
        // label.display(graph);
        heap.push_back(label);
        std::push_heap(heap.begin(), heap.end(), CompareLabel());
        // std::cout<<"Heap size: "<<heap.size()<<std::endl;
        return;
    }
    for (auto it = heap.begin(); it != heap.end(); ) {
        DominanceStatus status = label.DominanceCheck(*it);
        if (status == DominanceStatus::DOMINATED){//new label is dominated by existing label
            // std::cout << "New label is dominated by existing label" << std::endl;
            // label.display(graph);
            // std::cout << "Existing label: " << std::endl;
            // it->display(graph);        
            dominated.push_back(label); // Add new label to dominated list 
            return;}
        else if (status == DominanceStatus::DOMINATES) {//new label dominates existing label
            // std::cout << "New label dominates existing label" << std::endl;
            // it->display(graph);
            dominated.push_back(*it); // Add dominated label to dominated list
            it = heap.erase(it); // Remove dominated label
        } else if (status == DominanceStatus::INCOMPARABLE) {
            ++it; // Keep the label in the heap
        }
    }
    // heap.erase(std::remove_if(heap.begin(), heap.end(),
    //     [&](const Label& label) {
    //         return (label.status == LabelStatus::DOMINATED);
    //     }), heap.end());

    
    label.status = (label.status == LabelStatus::NEW_CLOSED) ? LabelStatus::CLOSED : LabelStatus::OPEN;
    label.open = (label.status == LabelStatus::OPEN);
    ID++;
    //label.LBImprove(graph);
    label.id = ID;
    // label.display(graph);
    heap.push_back(label);

    //std::cout << " Line 46" << std::endl;
    std::push_heap(heap.begin(), heap.end(), CompareLabel());
}





void LabelManager::displayLabels(Graph& graph) const {
    for (const Label& label : F_Heap) {
        label.display(graph);
    }
    for (const Label& label : B_Heap) {
        label.display(graph);
    }
}


void LabelManager::concatenateLabels(const Graph& graph) {
    std::vector<int> path;
    double cost = 0.0;

    for (auto fit = F_Heap.begin(); fit != F_Heap.end(); ++fit) {
        for (auto bit = B_Heap.begin(); bit != B_Heap.end(); ++bit) {
            if (fit->vertex != bit->vertex) continue;
            
            if (!fit->isConcatenable(*bit, graph.res_max)) {
                fit->checked_with.insert(bit->id);
                bit->checked_with.insert(fit->id);
                continue;
            }

            cost = fit->cost + bit->cost;

            if (cost < UB) {
                path = fit->path;
                path.insert(path.end(), bit->path.begin() + 1, bit->path.end());
                solutions.emplace_back(Solution(path, cost));
                UB = cost;
                // std::cout << "New UB: " << UB << std::endl;
            }
            fit->checked_with.insert(bit->id);
            bit->checked_with.insert(fit->id);
            
        }
    }
}


void LabelManager::displaySolutions() const {
    for (const Solution& solution : solutions) {
        solution.display();
    }
}






bool LabelManager::Terminate() {
    return !(F_Heap.front().open || B_Heap.front().open);
}


void LabelManager::Run(Graph& graph) {
    while (!Terminate()) {
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
    //displayLabels();
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
