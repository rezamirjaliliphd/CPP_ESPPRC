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
                randomResources[k] = ceil(static_cast<double>(std::rand()) / RAND_MAX * 4);
            }
            double cost = (static_cast<double>(std::rand()) / RAND_MAX - 0.5) * 10;
            graph.addEdge(i, j, cost, randomResources);
            graph.addEdge(j, i, cost, randomResources);
        }
    }
    graph.getMaxValue();
    graph.getMinWeights();
   
    graph.buildBaseModel();
    std::cout << "root model objective value: " << graph.model->get(GRB_DoubleAttr_ObjVal) << std::endl;
    /*for (int i = 0; i < n; ++i) {
        for (auto& e : graph.OutList[i]) {
            if (graph.x[{e->from, e->to}]->get(GRB_DoubleAttr_X) > 0) {
                std::cout << "x[" << e->from << "," << e->to << "] = " << graph.x[{e->from, e->to}]->get(GRB_DoubleAttr_X) << std::endl;
            }
        }
        if (graph.model->getVarByName("y[" + std::to_string(i) + "]").get(GRB_DoubleAttr_X) > 0) {
			std::cout << "y[" + std::to_string(i) + "] = " << graph.model->getVarByName("y[" + std::to_string(i) + "]").get(GRB_DoubleAttr_X) << std::endl;
        }
    }*/
    
    
	//std::cout << "Graph created" << std::endl;
 //   // Build an instance of MIP as LP and IP
 //   MIP ip_model(graph, false);
 //   MIP lp_model(graph, true);
	//std::cout << "MIP created" << std::endl;

    //// Solving pure IP
    //double ip_obj;
    auto start = std::chrono::high_resolution_clock::now();
    solveMIP(graph, false);
    //ip_obj = ip_model.solve();
    auto end = std::chrono::high_resolution_clock::now();
    auto duration_ip = static_cast<double>(std::chrono::duration_cast<std::chrono::microseconds>(end - start).count());


    //double lp_obj;
    //lp_obj = lp_model.solve();
    //std::cout << "First LB: " << lp_obj << std::endl;

	

 //   // start of algorithm
    std::cout << "Running ESPPRC" << std::endl;
    LabelManager manager(n, m, graph);
    std::cout << "manager initialized" << std::endl;
    auto start_esp = std::chrono::high_resolution_clock::now();
    manager.Run(graph);
	
    auto end_esp = std::chrono::high_resolution_clock::now();
    auto duration_esp = static_cast<double>(std::chrono::duration_cast<std::chrono::microseconds>(end_esp - start_esp).count());
    if (duration_esp > duration_ip) {
		std::cout << " ESPPRC is " << (duration_esp- duration_ip)/1000000 << " s slower ("<<(duration_esp/duration_ip) <<" times)!" << std::endl;
	}
    else {
        std::cout << " Gurobi is slower for " << (duration_ip - duration_esp)/1000000 << " s  slower.(" << (duration_ip / duration_esp) << " times)!" << std::endl;
    }
    /*std::cout << " ESPPRC Time: " << round(duration_esp/1000000) << std::endl;
    std::cout << " Gurobi Time: " << round(duration_ip/1000000) << std::endl;*/
 //   if (duration_ip - duration_esp > 0) {
 //       std::cout << " Gurobi is slower for " << static_cast<double>((duration_ip - duration_esp))/ duration_ip * 100<<"%" << std::endl;
 //   }
 //   else {
	//	std::cout << " ESPPRC is slower for " << static_cast<double>((duration_esp - duration_ip))/duration_esp *100<<"%" << std::endl;
 //   }
    manager.displaySolutions();
    return 0;
}

