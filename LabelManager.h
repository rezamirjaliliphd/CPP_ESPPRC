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

//struct CompareLabel {
//    bool operator()(const Label& a, const Label& b) const {
//		if (a.LB != b.LB) return a.LB < b.LB;
//        if (a.cost != b.cost) return a.cost < b.cost;
//        for (size_t i = 0; i < a.resources.size(); ++i) {
//            if (a.resources[i] != b.resources[i]) {
//                return a.resources[i] < b.resources[i];
//            }
//        }
//        int a_reachable_count = std::count(a.reachable.begin(), a.reachable.end(), true);
//        int b_reachable_count = std::count(b.reachable.begin(), b.reachable.end(), true);
//        if (a_reachable_count != b_reachable_count) return a_reachable_count > b_reachable_count;
//        return a.id < b.id;
//    }
//};
struct CompareLabel {
    bool operator()(const Label& a, const Label& b) const {
        if (a.open != b.open) return a.open < b.open;
        if (a.vertex != b.vertex) return a.vertex < b.vertex; // Min-heap based on vertex
        if (a.cost != b.cost) return a.cost > b.cost; // Min-heap based on cost
        if (a.LB != b.LB) return a.LB > b.LB; // Min-heap based on LB

    }
};

class LabelManager {
public:
    double UB = 1e9;
    std::vector<Solution> solutions;
    std::vector<Label> F_Heap,B_Heap, F_dominated, B_dominated;
    long long ID = 0;

    LabelManager(Graph& graph);
    void HuristicUB(Graph& graph);
    void DominanceCheckInsert(Label& label, Graph& graph);
    void displayLabels(Graph& graph) const;
    void concatenateLabels(const Graph& graph);
    void displaySolutions() const;
    void Propagate(Graph& graph);
    bool Terminate();
    void Run(Graph& graph);
    std::vector<std::pair<std::vector<int>,double>> getSolutions() const;
};

#endif // LABELMANAGER_H
// , MIP& mip