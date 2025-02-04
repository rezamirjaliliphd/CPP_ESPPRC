#include <gurobi_c++.h>
#include <iostream>
#include <vector>
#include <cstdlib>
#include <ctime>
#include <chrono>
#include "Graph.h"
#include "LabelManager.h"
#include "Solution.h"
#include "Edge.h"
#include "Utils.h"
#include "MIP.h"



void getRC(GRBModel& model, Graph& graph) {
	std::vector<double> rc(graph.num_edges, 0);
	for (int i = 0; i < graph.num_nodes; ++i) {
		for (auto e : graph.OutList[i]) {
			e->reduced_cost = model.getVarByName("x[" + std::to_string(e->from) + "," + std::to_string(e->to) + "]").get(GRB_DoubleAttr_RC);
		}
	}
}


int main() {
    // preprocessing
    int n = 10, m = 5;
    std::vector<double> res_max(m,0);
	for (int i = 0; i < m; ++i) {
		res_max[i] = 25;
	}

    // build a random graph with n nodes and m resources
    std::srand(std::time(nullptr));
    Graph graph(n, m, res_max);
    
    for (int i = 0; i < n; ++i) {
        for (int j = i + 1; j < n; ++j) {
            std::vector<double> randomResources(m);
            for (int k = 0; k < m; ++k) {
                randomResources[k] = ceil(static_cast<double>(std::rand()) / RAND_MAX * 6);
            }
            double cost = (static_cast<double>(std::rand()) / RAND_MAX - 0.75) * 10;
            graph.addEdge(i, j, cost, randomResources);
            graph.addEdge(j, i, cost, randomResources);
        }
    }
    graph.getMaxValue();
    graph.getMinWeights();
    
    auto start = std::chrono::high_resolution_clock::now();
    //std::cout << "Exact Solution: " << std::endl;
	solveMIP(graph, false);
    auto end = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end - start).count();
    //std::cout << "Exact Solution Time: " << duration << std::endl;

    
    GRBEnv env = GRBEnv(true);
    env.set(GRB_IntParam_OutputFlag, 0);
    env.set(GRB_IntParam_LogToConsole, 0);
    env.start();
    // Starting LP relaxation of the problem
    GRBModel model = GRBModel(env);
    GRBLinExpr obj = 0, inflow, outflow;
    std::unordered_map<int, std::unordered_map<int, GRBVar>> x;
    std::unordered_map<int, GRBVar> u;

    for (int i = 0; i < graph.num_nodes; ++i) {
        for (const auto e : graph.OutList[i]) {
            x[i][e->to] = model.addVar(0, 1, e->cost, GRB_CONTINUOUS, "x[" + std::to_string(e->from) + "," + std::to_string(e->to) + "]");
            obj += x[i][e->to] * e->cost;
        }
    }
    int cnt = 0;
    for (int i = 0; i < graph.num_nodes; i++) {
        u[i] = model.addVar(0.0, graph.num_nodes, 0.0, GRB_CONTINUOUS, "u[" + std::to_string(i) + "]");
    }

    // Flow conservation constraints
    for (int i = 0; i < graph.num_nodes; i++) {
        outflow = 0;
        inflow = 0;
        for (const auto e : graph.OutList[i]) {
            outflow += x[e->from][e->to];
        }
        for (const auto e : graph.InList[i]) {
            inflow += x[e->from][e->to];
        }
        if (i == 0) {
            model.addConstr(inflow == 1, "source");
            model.addConstr(outflow == 1, "sink");
        }
        else {
            model.addConstr(inflow == outflow, "flow_" + std::to_string(i));
        }
    }
    // Resource constraints
    for (int k = 0; k < graph.num_res; ++k) {
        GRBLinExpr constraint = 0;
        for (int i = 0; i < graph.num_nodes; ++i) {
            for (const auto e : graph.OutList[i]) {
                constraint += x[e->from][e->to] * e->resources[k];
            }
        }
        model.addConstr(constraint <= graph.res_max[k], "resource_" + std::to_string(k));
    }
    // Subtour elimination constraints
    for (int i = 0; i < graph.num_nodes; i++) {
        for (const auto e : graph.OutList[i]) {
            if (e->from != 0 && e->to != 0) {
                model.addConstr(u[e->from] + 1 <= u[e->to]
                    + graph.num_nodes * (1 - x[e->to][e->from]), "subtour_" + std::to_string(e->from)
                    + "_" + std::to_string(e->to));
            }
        }
    }
    model.setObjective(obj, GRB_MINIMIZE);
    model.optimize();
	getRC(model, graph);
    for (auto e : graph.edges) {
        if (e->reduced_cost > -model.get(GRB_DoubleAttr_ObjVal)) {
            graph.deleteEdge(e->from, e->to);
        }
    }
	
    std::cout << "First LB: " << model.get(GRB_DoubleAttr_ObjVal) << std::endl;





    // start of algorithm
    //std::cout << "Running ESPPRC" << std::endl;
    LabelManager manager(n, m, graph);
    //std::cout << "manager initialized" << std::endl;
    auto start_esp = std::chrono::high_resolution_clock::now();
    manager.Run(graph, res_max);
	manager.displaySolutions();
    auto end_esp = std::chrono::high_resolution_clock::now();
    auto duration_esp = std::chrono::duration_cast<std::chrono::microseconds>(end_esp - start_esp).count();
    std::cout << " ESPPRC Time: " << duration_esp << std::endl;
    std::cout << " Gurobi Time: " << duration << std::endl;
    if (duration - duration_esp > 0) {
        std::cout << " Gurobi is slower for " << static_cast<double>((duration - duration_esp))/duration*100<<"%" << std::endl;
    }
    else {
		std::cout << " ESPPRC is slower for " << static_cast<double>((duration_esp - duration))/duration_esp *100<<"%" << std::endl;
    }

    return 0;
}

