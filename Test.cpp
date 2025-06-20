#include "gurobi_c++.h"
#include <iostream>
#include <vector>

int main() {
    try {
        // Initialize Gurobi environment
        GRBEnv env = GRBEnv(true);
        env.start();

        // Define problem data
        int n = 5;                         // Number of items
        std::vector<double> weights = { 2, 3, 4, 5, 9 };  // Item weights
        std::vector<double> values = { 3, 4, 5, 8, 10 };  // Item values
        double capacity = 10;             // Maximum capacity of the knapsack

        // Create Gurobi model
        GRBModel model = GRBModel(env);

        // Decision variables: x[i] = 1 if item i is selected, 0 otherwise
        std::vector<GRBVar> x(n);
        for (int i = 0; i < n; ++i) {
            x[i] = model.addVar(0.0, 1.0, values[i], GRB_BINARY, "x_" + std::to_string(i));
        }

        // Add capacity constraint: sum(weights[i] * x[i]) <= capacity
        GRBLinExpr totalWeight = 0;
        for (int i = 0; i < n; ++i) {
            totalWeight += weights[i] * x[i];
        }
        model.addConstr(totalWeight <= capacity, "capacity");

        // Set the objective: maximize sum(values[i] * x[i])
        model.setObjective(model.getObjective(), GRB_MAXIMIZE);

        // Optimize the model
        model.optimize();

        // Display results
        if (model.get(GRB_IntAttr_Status) == GRB_OPTIMAL) {
            std::cout << "Optimal total value: " << model.get(GRB_DoubleAttr_ObjVal) << std::endl;
            std::cout << "Selected items: ";
            for (int i = 0; i < n; ++i) {
                if (x[i].get(GRB_DoubleAttr_X) > 0.5) { // Binary variable
                    std::cout << i << " ";
                }
            }
            std::cout << std::endl;
        }
        else {
            std::cout << "No optimal solution found!" << std::endl;
        }
    }
    catch (GRBException& e) {
        std::cerr << "Gurobi error: " << e.getMessage() << " (code " << e.getErrorCode() << ")" << std::endl;
    }
    catch (...) {
        std::cerr << "Unknown error!" << std::endl;
    }

    return 0;
}
