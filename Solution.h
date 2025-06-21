#ifndef SOLUTION_H
#define SOLUTION_H

#include <vector>
#include <iostream>
#include <utility>

class Solution {
public:
    std::vector<int> path;
    double cost;

    Solution(const std::vector<int>& p, double c);

    void display() const;
};

#endif // SOLUTION_H

