#ifndef RELAXEDBASEESPPRC_H
#define RELAXEDBASEESPPRC_H

#include <gurobi_c++.h>
#include <string>
#include "Graph.h"

class BaseModel {
private:
    GRBModel* model;  // Pointer to the Gurobi model
    std::vector<std::vector<GRBVar>> x;  // Declare x as a member variable
    std::vector<GRBVar> u;  // Visit order

public:

    // Constructor to create the model
    BaseModel(GRBEnv& env, int num_nodes, std::vector<float> capacities, int num_resources, Graph& Graph);

    // Destructor to clean up
    ~BaseModel();

    // Method to add a new edge 
    void addVisitedEdge(Edge& edge);

    // Method to optimize the model
    void optimize();

    // Method to display results
    void displayResults();
};

#endif // RELAXEDBASEESPPRC_H
