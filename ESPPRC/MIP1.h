#ifndef RELAXEDBASEESPPRC_H
#define RELAXEDBASEESPPRC_H

#include <gurobi_c++.h>
#include <string>
#include "Graph.h"
#include "Edge.h"
#include <unordered_map>
#include <iostream>

class MIP {
private:
    GRBModel* model;  // Pointer to the Gurobi model
    std::unordered_map<int, std::unordered_map<int, GRBVar>> x; // x as a member variables
    std::unordered_map<int, GRBVar> u; // Visit order

public:

    // Constructor to create the model
    MIP(Graph& Graph, bool LP_relaxation);

    // Destructor to clean up
    ~MIP();

    // Method to optimize the model
    double solve_with(std::vector<Edge>& edges);

    // Method to optimize without any edges added
    double solve();

};

#endif // RELAXEDBASEESPPRC_H
