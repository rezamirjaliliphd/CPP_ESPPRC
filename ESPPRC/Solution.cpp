#include "Solution.h"

Solution::Solution(const std::vector<int>& p, double c, std::pair<int, std::pair<long long, long long>> id)
    : path(p), cost(c), ID(id) {
}

void Solution::display() const {
    std::cout << "Path: ";
    for (int node : path) {
        std::cout << node << " ";
    }
    std::cout << ", Cost: " << cost << " \n";
}

