#ifndef RELAXEDBASEESPPRC_H
#define RELAXEDBASEESPPRC_H

#include <gurobi_c++.h>
#include <string>
#include "Graph.h"

class BaseModel {
public:
    GRBModel* model; // Pointer to Gurobi model

    // Constructor to create the model
    BaseModel(GRBEnv& env, int num_nodes, double capacity, Graph& Graph);

    // Destructor to clean up
    ~BaseModel();

    // Method to add a new edge 
    void addVisitedEdge(Edge& edge, double resource);

    // Method to optimize the model
    void optimize();

    // Method to display results
    void displayResults();
};

#endif // RELAXEDBASEESPPRC_H
