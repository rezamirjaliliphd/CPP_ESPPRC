#include "LabelManager.h"
#include <iostream>
#include <memory>

LabelManager::LabelManager(Graph& graph) {
    std::cout << "Create Labels at source and sink" << std::endl;
    Label source(graph,true);
    // source.display(graph);
    DominanceCheckInsert(source, graph);
	Label sink(graph, false);
    // sink.display(graph);
    DominanceCheckInsert(sink, graph);
    // std::cout<< "Sink and source labels created" << std::endl;
}


void LabelManager::DominanceCheckInsert(Label& label, Graph& graph) {
    int v = label.vertex;
	std::vector<Label>& heap = label.direction ? F_Heap : B_Heap;
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
    for (Label& rival : heap) {
        DominanceStatus status = label.DominanceCheck(rival);
        if (status == DominanceStatus::DOMINATED)//new label is dominated by existing label
            return;
        else if (status == DominanceStatus::DOMINATES) {//new label dominates existing label
            rival.status = LabelStatus::DOMINATED;
        }
    }
    heap.erase(std::remove_if(heap.begin(), heap.end(),
        [&](const Label& label) {
            return (label.status == LabelStatus::DOMINATED || label.LB > UB);
        }), heap.end());

    
    label.status = (label.status == LabelStatus::NEW_CLOSED) ? LabelStatus::CLOSED : LabelStatus::OPEN;
    ID++;
    //label.LBImprove(graph);
    label.id = ID;
    // label.display(graph);
    heap.push_back(label);

    //std::cout << " Line 46" << std::endl;
    std::push_heap(heap.begin(), heap.end(), CompareLabel());
}



void LabelManager::Propagate(Graph& graph) {

    //if (labelHeap.front().status != LabelStatus::OPEN) return;
	int neighbor;
    // Get the top valid label
    //std::cout << "Line 131" << std::endl;
    for (bool dir : {true, false}) {
		std::vector<Label>& labelHeap = dir ? F_Heap : B_Heap;
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
        parentLabel.status = LabelStatus::CLOSED;  // Close the parent label
        labelHeap.push_back(parentLabel);  // Reinsert the parent label
        std::push_heap(labelHeap.begin(), labelHeap.end(), CompareLabel());

        

    }
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


    for (auto fit = F_Heap.begin(); fit != F_Heap.end(); ++fit) {
        for (auto bit = B_Heap.begin(); bit != B_Heap.end() && bit != fit; ++bit) {
            if (fit->vertex != bit->vertex) {
                continue;
            }
            if (!fit->isConcatenable(*bit, graph.res_max)) {
                fit->checked_with.insert(bit->id);
                bit->checked_with.insert(fit->id);
                continue;
            }

            double cost = fit->cost + bit->cost;

            if (cost < UB) {
                path = fit->path;
                path.insert(path.end(), bit->path.begin() + 1, bit->path.end());
                solutions.emplace_back(Solution(path, cost, { fit->id, bit->id }));
                UB = cost;
                std::cout << "New UB: " << UB << std::endl;
            }else{
                fit->checked_with.insert(bit->id);
                bit->checked_with.insert(fit->id);
            }
        }
    }
}


void LabelManager::displaySolutions() const {
    for (const Solution& solution : solutions) {
        solution.display();
    }
}






bool LabelManager::Terminate() {
    return F_Heap.front().status != LabelStatus::OPEN || B_Heap.front().status != LabelStatus::OPEN;
}


void LabelManager::Run(Graph& graph) {
    while (!Terminate()) {
        Propagate(graph);
        concatenateLabels(graph);

    }
    concatenateLabels(graph);
    //displayLabels();
    //std::cout << "Solutions: " << std::endl;
    displaySolutions();
}


