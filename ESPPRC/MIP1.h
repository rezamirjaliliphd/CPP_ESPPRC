#ifndef RELAXEDBASEESPPRC_H
#define RELAXEDBASEESPPRC_H

#include <gurobi_c++.h>
#include <string>
#include "Graph.h"
#include "Edge.h"

class MIP {
private:
    GRBModel* model;  // Pointer to the Gurobi model
    std::vector<std::vector<GRBVar>> x;  // Declare x as a member variable
    std::vector<GRBVar> u;  // Visit order

public:

    // Constructor to create the model
    MIP(std::vector<float> capacities, int num_resources, Graph& Graph, bool LP_relaxation);

    // Destructor to clean up
    ~MIP();

    // Method to optimize the model
    float solve(std::vector<Edge>& edges);

};

#endif // RELAXEDBASEESPPRC_H
