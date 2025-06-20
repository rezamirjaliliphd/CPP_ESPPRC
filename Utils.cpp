#include "Utils.h"

void print_vector(const std::vector<int>& vec) {
    for (int i : vec) {
        std::cout << i << " ";
    }
    std::cout << std::endl;
}

std::vector<int> path_maker(std::vector<std::pair<int,int>>& edges) {
    std::vector<int> path = { 0 };
    while (path.back() != 0 || path.size() == 1) {
        for (const std::pair<int,int> e : edges) {
            if (e.first == path.back()) {
                path.push_back(e.second);
                break;
            }
        }
    }
    return path;
}
