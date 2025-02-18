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
#include <omp.h>
#include <execution> 
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
        if (a.status != b.status) {
            return a.status > b.status;
        }
        return a.LB > b.LB; // Min-heap: higher priority value means lower priority
    }
};
struct pair_hash {
    std::size_t operator()(const std::pair<long long, long long>& p) const {
        return std::hash<long long>()(p.first) ^ (std::hash<long long>()(p.second) << 1);
    }
};
class LabelManager {
public:
    double UB = 0;
    std::vector<Solution> solutions;
    //std::map<int, std::set<Label, CompareLabel>> Labels;
    std::vector<Label> labelHeap;
    std::unordered_set<std::pair<long long, long long>, pair_hash> IDs;
    long long ID = 0;

    LabelManager(Graph& graph);

    void DominanceCheckInsert(Label& label, Graph& graph);
    bool isIDDuplicate(const long long fw_id, const long long bw_id) const;
    void displayLabels() const;
    void concatenateLabels(const Graph& graph);
    void displaySolutions() const;
    void Propagate(Graph& graph);
    bool Terminate();
    void Run(Graph& graph);
};

#endif // LABELMANAGER_H
// , MIP& mip