#ifndef SOLUTION_H
#define SOLUTION_H

#include <vector>
#include <iostream>
#include <utility>

class Solution {
public:
    std::vector<int> path;
    double cost;
    std::pair<int, std::pair<long long, long long>> ID;

    Solution(const std::vector<int>& p, double c, std::pair<int, std::pair<long long, long long>> id);

    void display() const;
};

#endif // SOLUTION_H

