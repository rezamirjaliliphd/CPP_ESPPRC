#include "MIP1.h"
#include "Graph.h"
#include "Edge.h"
#include "gurobi_c++.h"
#include <iostream>
#include <unordered_map>

MIP::MIP(Graph& Graph, bool LP_relaxation)
    : x(), u() {
    try {
        GRBEnv env = GRBEnv(true); // Initialize the Gurobi environment
        env.set(GRB_IntParam_OutputFlag, 0);
        env.set(GRB_IntParam_LogToConsole, 0);
        env.start();
        model = new GRBModel(env); // Use new to properly allocate memory for the model
        GRBLinExpr obj = 0, inflow, outflow;
        this->x.clear();
        this->u.clear();
        std::string name;
        // Define variables
        for (int i = 0; i < Graph.num_nodes; ++i) {
            for (const auto e : Graph.OutList[i]) {
                name = "x[" + std::to_string(i) + ", " + std::to_string(e->to) + "]";
                x[{i, e->to}] = std::make_shared<GRBVar>(model->addVar(0, 1, e->cost, !LP_relaxation ? GRB_BINARY : GRB_CONTINUOUS),name);
                obj += *x[{i, e->to}] * e->cost;
            }
        }

        for (int i = 0; i < Graph.num_nodes; i++) {
            name = "u[" + std::to_string(i) + "]";
            u[i] = std::make_shared<GRBVar>(model->addVar(0.0, Graph.num_nodes, 0.0, !LP_relaxation ? GRB_INTEGER : GRB_CONTINUOUS, name));
        }

        // Flow conservation constraints
        for (int i = 0; i < Graph.num_nodes; i++) {
            outflow = 0;
            inflow = 0;
            for (const auto e : Graph.OutList[i]) {
                outflow += *x[{e->from, e->to}];
            }
            for (const auto e : Graph.InList[i]) {
                inflow += *x[{e->from, e->to}];
            }
            if (i == 0) {
                model->addConstr(inflow == 1, "source");
                model->addConstr(outflow == 1, "sink");
            }
            else {
                model->addConstr(inflow == outflow, "flow_" + std::to_string(i));
            }
        }

        // Resource constraints
        for (int k = 0; k < Graph.num_res; ++k) {
            GRBLinExpr constraint = 0;
            for (int i = 0; i < Graph.num_nodes; ++i) {
                for (const auto e : Graph.OutList[i]) {
                    constraint += *x[{e->from, e->to}] * e->resources[k];
                }
            }
            model->addConstr(constraint <= Graph.res_max[k], "resource_" + std::to_string(k));
        }

        // Subtour elimination constraints
        for (int i = 0; i < Graph.num_nodes; i++) {
            for (const auto& e : Graph.OutList[i]) {
				name = "subtour_" + std::to_string(i) + "_" + std::to_string(e->to);
				if (e->from == 0 || e->to == 0) continue;
                model->addConstr(*u[e->from] + 1 <= *u[e->to] + Graph.num_nodes * (1 - *x[{e->to, e->from}]), name);
            }
        }
        model->setObjective(obj, GRB_MINIMIZE); // Set objective function

    }
    catch (GRBException& e) {
        std::cerr << "Error code = " << e.getErrorCode() << std::endl;
        std::cerr << e.getMessage() << std::endl;
    }
}


MIP::~MIP() {
    delete model; // Deallocate memory for the model
}

// Method to solve the optimization problem with the edges fixed
double MIP::solve_with(const std::vector<int>& p) {
    double objVal = -1.0;  // Initialize to an invalid value
    try {
        // Set the lower bound of each edge in the vector to 1
		for (size_t i = 0; i < p.size() - 1; ++i) {
			x[{p[i], p[i + 1]}]->set(GRB_DoubleAttr_LB, 1);  // Set the lower bound to 1
		}

        model->update();  // Ensure changes are reflected in the model
        model->optimize();  // Perform the optimization

        // Check if the model found an optimal solution
        if (model->get(GRB_IntAttr_Status) == GRB_OPTIMAL) {
            objVal = model->get(GRB_DoubleAttr_ObjVal);  // Get the objective value
        }
        else {
            std::cout << "No optimal solution found." << std::endl;
        }

        // Reset the lower bounds of the edges back to 0 after optimization
        for (size_t i = 0; i < p.size() - 1; ++i) {
            x[{p[i], p[i + 1]}]->set(GRB_DoubleAttr_LB, 0);  // Set the lower bound back to 0
        }
        model->update();

    }
    catch (GRBException& e) {
        std::cerr << "Error during edge addition: " << e.getMessage() << std::endl;
    }

    return objVal;  // Return the objective value
}


// Method to solve the model without any edges fixed
double MIP::solve() {
    double objVal = -1.0;  // Initialize to an invalid value
    try {
        model->optimize();  // Perform the optimization

        // Check if the model found an optimal solution
        if (model->get(GRB_IntAttr_Status) == GRB_OPTIMAL) {
            objVal = model->get(GRB_DoubleAttr_ObjVal);  // Get the objective value
        }
        else {
            std::cout << "No optimal solution found." << std::endl;
        }
    }
    catch (GRBException& e) {
        std::cerr << "Error during edge addition: " << e.getMessage() << std::endl;
    }

    return objVal;  // Return the objective 
}

