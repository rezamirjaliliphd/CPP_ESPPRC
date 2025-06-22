#ifndef LABELMANAGER_H
#define LABELMANAGER_H

#include <vector>
#include <unordered_map>
#include <set>
#include <map>
#include <omp.h>
#include "Label.h"
#include "Solution.h"
#include "Graph.h"
#include "Utils.h"
#include <queue>
#include <algorithm>
#include <unordered_set>
#include <cstdint>
#include <intrin.h>
#pragma intrinsic(_popcnt64)

struct LabelPtrSetComparator {
   bool operator()(const std::shared_ptr<Label>& a,
                    const std::shared_ptr<Label>& b) const {
		// if (a.LB != b.LB) return a.LB < b.LB;
       if (a->cost != b->cost) return a->cost < b->cost;
       for (size_t i = 0; i < a->resources.size(); ++i) {
           if (a->resources[i] != b->resources[i]) {
               return a->resources[i] < b->resources[i];
           }
       }
       int a_reachable_count = __popcnt64(a->visited);
       int b_reachable_count = __popcnt64(b->visited);
       if (a->visited != b->visited) return a->visited < b->visited; // Compare reachable nodes
       if (a->LB != b->LB) return a->LB < b->LB; // Compare lower bounds
       return a->id < b->id;
   }
};
struct LabelPtrHeapComparator {
    bool operator()(const std::shared_ptr<Label>& a,
                    const std::shared_ptr<Label>& b) const {
        if (a->LB != b->LB) return a->LB> b->LB; // Min-heap based on LB
        if (a->cost != b->cost) return a->cost > b->cost; // Min-heap based on cost
        if (a->id != b->id) return a->id > b->id; // Min-heap based on ID
    }
};

class LabelManager {
public:
    double UB = 1e9;
    std::vector<Solution> solutions;
    std::unordered_map<int, std::set<std::shared_ptr<Label> , LabelPtrSetComparator>> F_set, B_set;
    std::vector<std::shared_ptr<Label> > F_Open, B_Open;
    long long ID = 0;
    std::unordered_map<int,bool> F_concat_need, B_concat_needed;
    LabelManager(Graph& graph);
    void HuristicUB(Graph& graph);
    // void DominanceCheckInsert(std::shared_ptr<Label>& label_ptr, Graph& graph);
    void displayLabels(Graph& graph) const;
    void concatenateLabels(const Graph& graph);
    void displaySolutions() const;
    void Propagate(Graph& graph);
    bool Terminate();
    void Run(Graph& graph);
    void LabelManager::PruneLabels(const Graph& graph);
    std::vector<std::pair<std::vector<int>,double>> getSolutions() const;
};

#endif // LABELMANAGER_H
// , MIP& mip