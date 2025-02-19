#include "LabelManager.h"
#include <iostream>
#include <memory>

LabelManager::LabelManager(Graph& graph) {
    //std::cout << "Create Labels at source and sink" << std::endl;
    Label source(graph,true);
    DominanceCheckInsert(source, graph);
	Label sink(graph, false);
    DominanceCheckInsert(sink, graph);
}


void LabelManager::DominanceCheckInsert(Label& label, Graph& graph) {
    int v = label.vertex;
	std::vector<Label>& heap = label.direction ? F_Heap : B_Heap;
    //std::vector<Label> tempHeap;
    bool isDominated = false;
    if (heap.empty()) {
        ID++;
        label.id = ID;
        label.status = LabelStatus::OPEN;
        heap.push_back(label);
        std::push_heap(heap.begin(), heap.end(), CompareLabel());
        return;
    }
    for (Label& rival : heap) {
        if (label.vertex != rival.vertex) continue;
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

    if (!isDominated) {
        label.status = (label.status == LabelStatus::NEW_CLOSED) ? LabelStatus::CLOSED : LabelStatus::OPEN;
        ID++;
        //label.LBImprove(graph);
        label.id = ID;
        //label.display();
        heap.push_back(label);

        //std::cout << " Line 46" << std::endl;
        std::push_heap(heap.begin(), heap.end(), CompareLabel());
    }

}


bool LabelManager::isIDDuplicate(const long long fw_id, const long long bw_id) const {

    return IDs.find(std::make_pair(fw_id, bw_id)) != IDs.end();
    //std::cout << " Line 64" << std::endl;
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
        //std::cout << "Parent Label: " << std::endl;
        //parentLabel.display();
        labelHeap.pop_back();  // Remove from heap
        if (parentLabel.LB <= UB) {
            // Step 2: Process the best label (propagate children labels)
            for (const auto& edge : dir?graph.OutList[parentLabel.vertex]: graph.InList[parentLabel.vertex]) {
				neighbor = dir ? edge->to : edge->from;
                if (parentLabel.reachable[neighbor]) {
                    Label newLabel(parentLabel, graph, edge.get(), UB);  // Create new label

                    if (newLabel.status != LabelStatus::DOMINATED) {
                        DominanceCheckInsert(newLabel, graph);  // Insert new label into the heap if valid
                    }
                }
            }
            parentLabel.status = LabelStatus::CLOSED;  // Close the parent label
            labelHeap.push_back(parentLabel);  // Reinsert the parent label
            std::push_heap(labelHeap.begin(), labelHeap.end(), CompareLabel());


        }

    }
    

}



void LabelManager::displayLabels() const {
    for (const Label& label : F_Heap) {
        label.display();
    }
    for (const Label& label : B_Heap) {
        label.display();
    }
}


void LabelManager::concatenateLabels(const Graph& graph) {
    std::vector<int> path;


    for (auto fit = F_Heap.begin(); fit != F_Heap.end(); ++fit) {
        for (auto bit = B_Heap.begin(); bit != B_Heap.end() && bit != fit; ++bit) {
            if (isIDDuplicate(fit->id, bit->id)) continue;
            if (fit->vertex != bit->vertex || !fit->isConcatenable(*bit, graph.res_max)) {
                IDs.insert({ fit->id, bit->id });
                continue;
            }

            double cost = fit->cost + bit->cost;

            if (cost < UB) {
                path = fit->path;
                path.insert(path.end(), bit->path.begin() + 1, bit->path.end());
                solutions.emplace_back(Solution(path, cost, { fit->id, bit->id }));
                UB = cost;
                //std::cout << "New UB: " << UB << std::endl;
            }
			IDs.insert({ fit->id, bit->id });
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
    /*concatenateLabels(graph);*/
    //displayLabels();
    //std::cout << "Solutions: " << std::endl;
    //displaySolutions();
}


