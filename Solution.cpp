#include "Solution.h"

Solution::Solution(const std::vector<int>& p, double c)
    : path(p), cost(c){
}

void Solution::display() const {
    std::cout << "Path: ";
    for (int node : path) {
        std::cout << node << " ";
    }
    std::cout << ", Cost: " << cost << " \n";
}

