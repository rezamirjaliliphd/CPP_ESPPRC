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

struct CompareLabel {
    bool operator()(const Label& a, const Label& b) const {
		if (a.LB != b.LB) return a.LB < b.LB;
        if (a.cost != b.cost) return a.cost < b.cost;
        for (size_t i = 0; i < a.resources.size(); ++i) {
            if (a.resources[i] != b.resources[i]) {
                return a.resources[i] < b.resources[i];
            }
        }
        int a_reachable_count = std::count(a.reachable.begin(), a.reachable.end(), true);
        int b_reachable_count = std::count(b.reachable.begin(), b.reachable.end(), true);
        if (a_reachable_count != b_reachable_count) return a_reachable_count > b_reachable_count;
        return a.id < b.id;
    }
};
struct CompareLableLB {
    bool operator()(const Label* a, const Label* b) const {
        return a->cost > b->cost; // Min-heap: higher priority value means lower priority
    }
};
class LabelManager {
public:
    double UB = 1000;
    std::vector<Solution> solutions;
    std::map<int, std::set<Label, CompareLabel>> Labels;
    std::priority_queue<Label*, std::vector<Label*>, CompareLableLB> labelQueue;
	std::set<std::tuple<int, long long, long long>> IDs;
    std::unordered_set<Label*> invalidLabels;

    LabelManager(int num_nodes, int num_res, Graph& graph);
    void initializeLabels(int num_nodes, int num_res, Graph& graph);
    void DominanceCheckInsert(Label& label);
    bool isIDDuplicate(const int vertex, const long long fw_id, const long long bw_id) const;
    long long find_ID(const int vertex) const;
    void displayLabels() const;
    void concatenateLabels(const Graph& graph);
    void displaySolutions() const;
    void Propagate(Graph& graph);
    bool Terminate(Graph& graph);
    void Run(Graph& graph);
};

#endif // LABELMANAGER_H
// , MIP& mip