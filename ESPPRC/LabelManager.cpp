#include "LabelManager.h"
#include <iostream>
#include <memory>

LabelManager::LabelManager(int num_nodes, int num_res, Graph& graph) {
    //std::cout << "Create Labels at source and sink" << std::endl;
	Label source(graph);
	DominanceCheckInsert(source,graph);
}


void LabelManager::DominanceCheckInsert(Label& label, Graph& graph) {
    int v = label.vertex;
    std::vector<Label> tempHeap;
	bool isDominated = false;
    if (labelHeap.empty()) {
		ID++;
		label.id = ID;
		label.status = LabelStatus::OPEN;
        labelHeap.push_back(label);
        std::push_heap(labelHeap.begin(), labelHeap.end(), CompareLabel ());
        return;
    }
    for (Label& rival : labelHeap) {
		if (label.vertex != rival.vertex) continue;
        DominanceStatus status = label.DominanceCheck(rival);
		if (status == DominanceStatus::DOMINATED)//new label is dominated by existing label
			return;
		else if (status == DominanceStatus::DOMINATES) {//new label dominates existing label
			rival.status = LabelStatus::DOMINATED;
		}
    }
    labelHeap.erase(std::remove_if(labelHeap.begin(), labelHeap.end(),
        [&](const Label& label) {
            return (label.status == LabelStatus::DOMINATED||label.LB>UB); 
        }), labelHeap.end());

	if (!isDominated){
        label.status = (label.status == LabelStatus::NEW_CLOSED) ? LabelStatus::CLOSED : LabelStatus::OPEN;
		ID++;
		//label.LBImprove(graph);
		label.id = ID;
		//label.display();
        labelHeap.push_back(label);
        
        //std::cout << " Line 46" << std::endl;
        std::push_heap(labelHeap.begin(), labelHeap.end(), CompareLabel());
    }
    
}


bool LabelManager::isIDDuplicate(const long long fw_id, const long long bw_id) const {
   
    return IDs.find(std::make_pair(fw_id,bw_id))!=IDs.end(); 
    //std::cout << " Line 64" << std::endl;
}

void LabelManager::Propagate(Graph& graph) {

    //if (labelHeap.front().status != LabelStatus::OPEN) return;

    // Get the top valid label
    //std::cout << "Line 131" << std::endl;
    std::pop_heap(labelHeap.begin(), labelHeap.end(), CompareLabel());  // Move best label to end
    Label parentLabel = labelHeap.back();  // Store the best label
	//std::cout << "Parent Label: " << std::endl;
	//parentLabel.display();
    labelHeap.pop_back();  // Remove from heap
    if (parentLabel.LB <= UB) {
        // Step 2: Process the best label (propagate children labels)
        for (const auto& edge : graph.OutList[parentLabel.vertex]) {
            if (parentLabel.reachable[edge->to]) {
                Label newLabel(parentLabel, graph, edge.get(), UB);  // Create new label

                if (newLabel.status != LabelStatus::DOMINATED) {
                    DominanceCheckInsert(newLabel,graph);  // Insert new label into the heap if valid
                }
            }
        }
		parentLabel.status = LabelStatus::CLOSED;  // Close the parent label
		labelHeap.push_back(parentLabel);  // Reinsert the parent label
        std::push_heap(labelHeap.begin(), labelHeap.end(), CompareLabel());


    }
    
}



void LabelManager::displayLabels() const {
    for (const Label& label : labelHeap) {
        label.display();
    }
}


void LabelManager::concatenateLabels(const Graph& graph) {
    std::vector<int> path;
   
    
    for (auto fit = labelHeap.begin(); fit != labelHeap.end(); ++fit) {
        for (auto bit = std::next(fit); bit != labelHeap.end() && bit != fit; ++bit) {
			if (isIDDuplicate(fit->id, bit->id)) continue;
            if (fit->vertex != bit->vertex || !fit->isConcatenable(*bit, graph.res_max)) {
                IDs.insert({ fit->id, bit->id });
                continue;
            }
            

            // Concatenate paths efficiently
            path = fit->path;
            path.insert(path.end(), bit->path.rbegin() + 1, bit->path.rend());
			//std::cout << " Forward:" << std::endl;
			//fit->display();
            //std::cout << " Backward:" << std::endl;
			//bit->display();
            // Calculate the new cost
            double cost = fit->cost + bit->cost;
			//std::cout << "Cost: " << cost << std::endl;
			//print_vector(path);
            // Check and update UB
            
            if (cost < UB) {
                solutions.emplace_back(Solution(path, cost, {fit->id, bit->id }));
                UB = cost;
                //std::cout << "New UB: " << UB << std::endl;
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
    return labelHeap.front().status != LabelStatus::OPEN;
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



