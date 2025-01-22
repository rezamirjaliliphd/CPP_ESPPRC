#include "RelaxedBaseESPPRC.h"
#include "Graph.h"
#include "Edge.h"
#include <iostream>

BaseModel::BaseModel(GRBEnv& env, int num_nodes, double capacity, Graph& Graph) {
    try {
        model = new GRBModel(env); // Initialize Gurobi model

        // store x_i_j variables [num_nodes+1, num_nodes+1]
        std::vector<std::vector<GRBVar>> x(num_nodes+1, std::vector<GRBVar>(num_nodes+1));
        std::vector<GRBVar> u(num_nodes + 2);
        GRBVar r;

        GRBConstr usedup_capacity;

        // Add variables with two indices
        for (int i = 0; i < num_nodes+2; ++i) {
            for (int j = 0; j < num_nodes+2; ++j) {
                std::string varName = "x_" + std::to_string(i) + "_" + std::to_string(j); // x_i_j
                x[i][j] = model->addVar(0.0, 1.0, 0.0, GRB_CONTINUOUS, varName);
            }
            std::string varname = 'u' + std::to_string(i);
            u[i] = model->addVar(1, num_nodes + 2, 0.0, GRB_INTEGER, varname);
        }
        r = model->addVar(0, GRB_INFINITY, 0, GRB_CONTINUOUS, "r"); // used up resource
        

        // Add constraints
        // Sub-tour elimination
        for (int i = 0; i < num_nodes + 2; ++i) {
            for (int j = 0; j < num_nodes + 2; ++j) {
                if (i != j) { // To avoid self-loops
                    std::string constrName = "st_" + std::to_string(i) + "_" + std::to_string(j);
                    model->addConstr(u[i] + 1 <= u[j] + num_nodes+2 * (1 - x[i][j]), constrName);
                }
            }
        }


        // Equality of inflow and outflow for N\{source, sink}
        for (int i = 1; i < num_nodes + 1; ++i) {
            GRBLinExpr inFlow = 0;
            GRBLinExpr outFlow = 0;

            for (int j = 1; j < num_nodes + 1; ++j) {
                if (i != j) { // Avoid self-loops
                    outFlow += x[i][j];  // Incoming flow to node j
                    inFlow += x[j][i]; // Outgoing flow from node j
                }
                std::string constrName = "in_out_" + std::to_string(i);
                model->addConstr(inFlow == outFlow, constrName);
            }


        }
        // Source outflow and sink inflow must be 1
        GRBLinExpr sourceOutFlow = 0; // Source
        for (int i = 1; i < num_nodes + 2; ++i) {
            sourceOutFlow += x[0][i];
            }
        model->addConstr(sourceOutFlow == 1, "source_out");
        GRBLinExpr sinkInFlow = 0;  // Sink
        for (int i = 0; i < num_nodes + 1; ++i) {
            sinkInFlow += x[0][i];
            }
        model->addConstr(sinkInFlow == 1, "sink_out");



        // Used up resource initial value. When a label propagates, this constraint's LHS is increased by a
        usedup_capacity = model->addConstr(r <= capacity, "usedup_apacity");



        // Set objective 
        GRBLinExpr obj = 0;
        for (int i = 0; i < num_nodes + 2; i++) {
            for (int j = 0; j < num_nodes + 2; j++) {
                if (i != j) {
                    obj += Graph.getEdge(i, j).cost * x[i][j];
                }

            }
        }
        model->setObjective(obj, GRB_MINIMIZE);

    }
    catch (GRBException& e) {
        std::cerr << "Error code = " << e.getErrorCode() << std::endl;
        std::cerr << e.getMessage() << std::endl;
    }
}

BaseModel::~BaseModel() {
    delete model; // Free the memory allocated for the model
}

void BaseModel::optimize() {
    try {
        model->optimize();
    }
    catch (GRBException& e) {
        std::cerr << "Error during optimization: " << e.getMessage() << std::endl;
    }
}


// adding the edge propagated to the model
void BaseModel::addVisitedEdge(Edge& edge, double resource) {

}

void BaseModel::displayResults() {
    try {
        if (model->get(GRB_IntAttr_Status) == GRB_OPTIMAL) {
            std::cout << "Optimal solution found!" << std::endl;
            std::cout << "Objective value: " << model->get(GRB_DoubleAttr_ObjVal) << std::endl;
        }
        else {
            std::cout << "No optimal solution found." << std::endl;
        }
    }
    catch (GRBException& e) {
        std::cerr << "Error during result display: " << e.getMessage() << std::endl;
    }
}
