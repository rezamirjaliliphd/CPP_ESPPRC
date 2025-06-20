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




int main() {
    // preprocessing
    int n = 10, m = 5;
    std::vector<double> res_max(m,0);
	for (int i = 0; i < m; ++i) {
		res_max[i] = 25;
	}
    std::vector<int> {0,1,2,3,4,5,6,0};
    // build a random graph with n nodes and m resources
    std::srand(std::time(nullptr));
    Graph graph(n, res_max);
    
    for (int i = 0; i < n; ++i) {
        for (int j = i + 1; j < n; ++j) {
            std::vector<double> randomResources(m);
            if (i<6 && j==i+1){
                randomResources = {1, 1, 1, 1, 1};
                graph.addEdge(i, j, -1000, randomResources);
                graph.addEdge(j, i, 1000, randomResources);
            }
            else if(i==6){
                randomResources = {1, 1, 1, 1, 1};
                graph.addEdge(i, 0, -1000, randomResources);     
            }
            else{
            for (int k = 0; k < m; ++k) {
                randomResources[k] = ceil(static_cast<double>(std::rand()) / RAND_MAX * 4);
            }
            double cost = (static_cast<double>(std::rand()) / RAND_MAX - 0.5) * 10+100;
            graph.addEdge(i, j, cost, randomResources);
            graph.addEdge(j, i, cost, randomResources);
        }
    }
}
    // graph.display();
    
  
    std::cout << "Running ESPPRC" << std::endl;
    LabelManager manager(graph);
    std::cout << "manager initialized" << std::endl;
    auto start_esp = std::chrono::high_resolution_clock::now();
    manager.Run(graph);
	
    auto end_esp = std::chrono::high_resolution_clock::now();
    auto duration_esp = static_cast<double>(std::chrono::duration_cast<std::chrono::microseconds>(end_esp - start_esp).count());
    
    std::cout << " ESPPRC Time: " << duration_esp << std::endl;
    // std::cout << " Gurobi Time: " << round(duration_ip/1000000) << std::endl;*/
 //   if (duration_ip - duration_esp > 0) {
 //       std::cout << " Gurobi is slower for " << static_cast<double>((duration_ip - duration_esp))/ duration_ip * 100<<"%" << std::endl;
 //   }
 //   else {
	//	std::cout << " ESPPRC is slower for " << static_cast<double>((duration_esp - duration_ip))/duration_esp *100<<"%" << std::endl;
 //   }
    
    return 0;
}

