#ifndef RELAXEDBASEESPPRC_H
#define RELAXEDBASEESPPRC_H

#include <gurobi_c++.h>
#include <string>
#include "Graph.h"
#include "Edge.h"
#include <map>
#include <iostream>

class MIP {
private:
    GRBModel* model;  // Pointer to the Gurobi model
    std::map < std::pair<int,int>, std::shared_ptr<GRBVar>> x; // x as a member variables
    std::map<int, std::shared_ptr<GRBVar>> u; // Visit order

public:

    // Constructor to create the model
    MIP(Graph& Graph, bool LP_relaxation);

    // Destructor to clean up
    ~MIP();

    // Method to optimize the model
    double solve_with(const std::vector<int>& p);

    // Method to optimize without any edges added
    double solve();

};

#endif // RELAXEDBASEESPPRC_H
