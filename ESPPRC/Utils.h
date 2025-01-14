#pragma once
#ifndef UTILS_H
#define UTILS_H

#include <vector>
#include <utility>
#include <iostream>
#include "Edge.h"

void print_vector(const std::vector<int>& vec);
std::vector<int> path_maker(std::vector<std::pair<int,int>>& edges);

#endif // UTILS_H
