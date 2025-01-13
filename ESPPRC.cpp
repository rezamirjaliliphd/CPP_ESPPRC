// #include "gurobi_c++.h"
#include <cassert>
#include <sstream>
#include <iostream>
#include <vector>
#include <functional>
#include <algorithm>
#include <cstdlib>
#include <ctime>
#include <cmath>
#include<numeric>
#include<unordered_map>
#include<climits>
#include<unordered_set>
// added a comment

enum class LabelStatus {
    NEW_OPEN, // If label has been generated but has not been inserted into the set of labels and has not hit half-point
    NEW_CLOSED, // If label has been generated and has not been inserted into the set of labels but has hit the half-point   
    OPEN, // If label has been generated and does not have offspring but has survived the dominance check i.e. inserted into the set of labels
	CLOSED,// If label is propagated or hit the half-point
    DOMINATED
};
enum class DominanceStatus {
    DOMINATES,
    DOMINATED,
    INCOMPARABLE};

enum class LabelDirection {
	FORWARD,
	BACKWARD
};

void print_vector(const std::vector<int>& vec) {
    for (int i : vec) {
        std::cout << i << " ";
    }
    std::cout << std::endl;
};

// Class to represent a edge
class Edge {
public:
    int from;          // Source node
    int to;            // Destination node
    double cost;       // Cost of the edge
    std::vector<double> resources; // Resource consumption of the edge


    // Constructor
    Edge(int f, int t, double c, const std::vector<double>& r) : from(f), to(t), cost(c), resources(r) {}

};

class Solution {
public:
    std::vector<int> path;
    double cost;
    std::pair<int, std::pair<long long, long long>> ID;
    Solution(const std::vector<int>& p, double c, std::pair<int, std::pair<long long, long long>> id) : path(p), cost(c), ID(id) {
        path = p;
        cost = c;
        ID = id;
    }
    void display() const {
        std::cout << "Path: ";
        for (int node : path) {
            std::cout << node << " ";
        }
        std::cout << ", Cost: " << cost << " \n";
    }
};

class Graph {
private:
	std::vector<std::vector<Edge>> OutList, InList; // Outgoing and incoming edges

public:
    int num_nodes;
    int num_res;
    std::vector<std::vector<double>> min_weight;
    std::vector<double> max_value;
	std::vector<double> res_max;
    // Constructor
    Graph(int n, int m, std::vector<double> r_max) : OutList(n),InList(n), num_nodes(n), num_res(m),
        min_weight(n, std::vector<double>(m, 100.0)), max_value(n, 100.0), res_max(r_max) {}

    // Add an edge to the graph
    void addEdge(int from, int to, double cost, const std::vector<double>& resources) {
        OutList[from].emplace_back(Edge(from, to, cost, resources));
		InList[to].emplace_back(Edge(from, to, cost, resources));
    }
	
    // Get the neighbors of a node
    const std::vector<Edge>& getNeighbors(int node, LabelDirection dir) const {
		return dir==LabelDirection::BACKWARD ? InList[node] : OutList[node];
	}
    // Display the graph
    void display() const {
        for (int i = 0; i < num_nodes; ++i) {
            std::cout << "Node " << i << ":\n";
            for (const Edge& edge : OutList[i]) {
                std::cout << "  -> " << edge.to << " (Cost: " << edge.cost << ", Resources: [";
                for (size_t j = 0; j < edge.resources.size(); ++j) {
                    std::cout << edge.resources[j] << (j + 1 < edge.resources.size() ? ", " : "");
                }
                std::cout << "])\n";
            }
        }
    }

    std::vector<std::vector<double>> getMinWeights() {
        // outputing a vector of minimum resources' consumptions by node 

        for (int i = 0; i < num_nodes; i++) {
            for (const Edge& e : OutList[i]) {
                for (int k = 0; k < e.resources.size(); k++) {
                    if (min_weight[i][k] > e.resources[k]) {
                        min_weight[i][k] = e.resources[k];
                    }
                }
            }
        }
        return min_weight;
    }
    void getMaxValue() {
        // outputing a vector of maximum value by node 
        for (int i = 0; i < num_nodes; i++) {
            for (const Edge& e : OutList[i]) {
                if (max_value[i] > e.cost) {
                    max_value[i] = e.cost;
                }
            }
        }
    }
};


// Class to represent a label (partial path)
class Label {
public:
    long long id = -1;                  // Unique identifier
    int  vertex;                     // Current vertex
    std::vector<int> path;            // Nodes in the current path
    double cost;                      // Cost of the path
    std::vector<double> resources;    // Resource consumption of the path
    std::vector<bool> reachable;      // Reachability of the nodes
    LabelDirection direction;      // Forward:true or backward:false
    double LB;      // Lower bound
    LabelStatus status;     // Status of the label
	

    // Constructor
   
	Label(Graph& graph, LabelDirection dir) : vertex(0), path({ 0 }), cost(0), resources(graph.num_res,0), reachable(graph.num_nodes, true), direction(dir), LB(0), id(0) {
        //std::cout << "Creating a new label" << std::endl;
        // Initialize the resources
        resources = std::vector<double>(graph.num_res, 0);
        // Initialize the reachability
        reachable = std::vector<bool>(graph.num_nodes, true);
        // Initialize the direction
        direction = dir;
        // Initialize the status
        status = LabelStatus::OPEN;
        // Initialize the path
        path = { 0 };
        // Initialize the cost
        cost = 0;
        // Initialize the vertex
        vertex = 0;
        // Initialize the LB
        LB = 0;
        reachable[0] = false;
    }
    Label(const Label& parent, const Graph& graph, const Edge& edge, const double UB)
        : path(parent.path), cost(parent.cost),
        resources(parent.resources), reachable(parent.reachable),
        direction(parent.direction) {
        // Add the new node to the path and update the cost and resources
        vertex = direction==LabelDirection::FORWARD? edge.to: edge.from;
        cost = parent.cost+edge.cost;
        resources = parent.resources;
        reachable = parent.reachable;
        direction = parent.direction;
        path = parent.path;
        if (direction==LabelDirection::FORWARD){ path.push_back(vertex);} else {path.insert(path.begin(), vertex);}
        reachable[vertex] = false;
        reachable[0] = false;
        for (size_t i = 0; i < resources.size(); ++i) {
            resources[i] += edge.resources[i]; // Update resources
            }
        reachHalfPoint(graph.res_max,graph.num_nodes)? status = LabelStatus::NEW_CLOSED : status = LabelStatus::NEW_OPEN;
        
        LB = getLB(graph.res_max, graph.num_nodes, graph.min_weight, graph.max_value)+cost;
        UpdateReachable(edge, graph, UB);
        if (LB > UB) {
            status = LabelStatus::DOMINATED;
        }
     }

    // Add a node to the label
    void UpdateReachable(const Edge& edge, const Graph& graph, const double UB) {
        
        for (const Edge& e : graph.getNeighbors(vertex, direction)) {
            int neighbor = direction == LabelDirection::FORWARD ? e.to : e.from;
            if (reachable[neighbor] && neighbor != 0) {
                for (size_t i = 0; i < resources.size(); ++i) {
                    if (resources[i] + e.resources[i] > graph.res_max[i]) {
                        reachable[neighbor] = false;
                        break;
                    }
                }
            }
        }
    }

    // Reaches half=point
    bool reachHalfPoint(const std::vector<double>& res_max, int num_nodes) {
        if (path.size() >= static_cast<double>(num_nodes) / 2) {
            return true;
        }
        for (size_t i = 0; i < resources.size(); ++i) {
            if (resources[i] >= res_max[i] / 2) {
                return true;
            }
        }
        return false;
    }

    double getLB(const std::vector<double>& res_max,
        const int n,
        const std::vector<std::vector<double>>& min_weight,
        const std::vector<double>& max_value) {

        // GRBEnv env = GRBEnv();
        // env.set(GRB_IntParam_OutputFlag, 0);
        // GRBModel model = GRBModel(env);
        // std::vector<int> Items;


        // GRBLinExpr obj = 0;
        // std::vector<GRBVar> x(n);
        // for (int i = 0; i < reachable.size(); i++) {
        //     int ub = reachable[i] ? 1 : 0;
        //     x[i] = model.addVar(0.0, ub, 0, GRB_BINARY, "x" + std::to_string(i));
        //     obj -= x[i] * max_value[i];
        // }

        // model.setObjective(obj, GRB_MAXIMIZE);

        // for (int k = 0; k < res_max.size(); k++) {
        //     GRBLinExpr cntr = 0;
        //     for (int j = 0; j < x.size(); j++) {
        //         cntr += x[j] * min_weight[j][k];
        //     }
        //     model.addConstr(cntr <= res_max[k], "resource " + std::to_string(k));

        // }
        // // Optimize the model
        // model.optimize();
        // return model.get(GRB_DoubleAttr_ObjVal);
        return 100;
    }



    // Display the label
    void display() const {
        std::cout << "=========================\n";
        std::cout << "Direction: ";
        std::cout<<(direction==LabelDirection::FORWARD ? "Forward" : "Backward")<<std::endl;
        std::cout << "Path: ";
        for (int node : path) {
            std::cout << node << " ";
        }
        std::cout << ", Cost: " << cost << ", Resources: [";
        for (size_t i = 0; i < resources.size(); ++i) {
            std::cout << resources[i] << (i + 1 < resources.size() ? ", " : "");
        }
        std::cout << "]\n";
        for (size_t i = 0; i < reachable.size(); ++i) {
            if (reachable[i]) {
                std::cout << "Node " << i << " is reachable\n";
            }
        }
        std::cout << "Status: ";
        switch (status) {
        case LabelStatus::NEW_OPEN:
            std::cout << "NEW_OPEN\n";
            break;
        case LabelStatus::NEW_CLOSED:
            std::cout << "NEW_CLOSED\n";
            break;
        case LabelStatus::OPEN:
            std::cout << "OPEN\n";
            break;
        case LabelStatus::CLOSED:    
            std::cout << "CLOSED\n";
            break;
        case LabelStatus::DOMINATED:
            std::cout << "DOMINATED\n";
            break;
        }
        std::cout << "ID (vertex, id): ( " << vertex << " , " << id <<" )" << std::endl;
        std::cout << "=========================\n\n";

    }

    // Compare two labels based on cost
    // Function to check the dominance relationship between two labels
    DominanceStatus dominance(const Label& rival) const {
        bool dominates = true, dominated = true;

        // Compare resource consumption
        for (size_t i = 0; i < resources.size(); ++i) {
            if (resources[i] > rival.resources[i]) dominates = false; // Current label does not dominate if it uses less resources
            if (resources[i] < rival.resources[i]) dominated = false; // Current label is not dominated if it uses more resources
        }

        // Compare reachability
        for (size_t i = 0; i < reachable.size(); ++i) {
            if (reachable[i] < rival.reachable[i]) dominates = false; // Current label does not dominate if it reaches fewer nodes
            if (reachable[i] > rival.reachable[i]) dominated = false; // Current label is not dominated if it reaches more nodes
        }

        // Determine dominance status based on cost and resource/reachability comparison
        if (dominates && cost <= rival.cost) return DominanceStatus::DOMINATES; // Current label dominates if it has lower or equal cost and dominates in resources/reachability
        if (dominated && cost >= rival.cost) return DominanceStatus::DOMINATED; // Current label is dominated if it has higher or equal cost and is dominated in resources/reachability
        return DominanceStatus::INCOMPARABLE; // Labels are incomparable if neither dominates the other
    }

    // if label is concatable with the other label
    bool isConcatenable(const Label& bw_label, const std::vector<double>& r_max) const {
    // Allow concatenation regardless of which direction initiated it
      
        // Check resource constraints
        for (size_t i = 0; i < resources.size(); ++i) {
            if (resources[i] + bw_label.resources[i] > r_max[i]) {
                return false;
            }
        }
        
        // Check for path intersection (excluding meeting point)
        std::unordered_set<int> first_set(path.begin(), path.end() - 1);
        return std::none_of(bw_label.path.begin() + 1, bw_label.path.end(), 
            [&first_set](const int& x) { return first_set.count(x) > 0; });
    }
    
};

// Class to manage labels
class LabelManager {
private:
    std::unordered_map<int, std::vector<Label>> fw_labels, bw_labels; // Collection of labels
    /*
       The structure is std::unordered_map<vertex(int), label(Label)>
    */
public:

    LabelManager(int num_nodes, int num_res, Graph& graph) : fw_labels(),bw_labels() {
		std::cout << "Initializing Labels" << std::endl;
        initializeLabels(num_nodes, num_res, graph);
    }

    double UB = 1000;
    // Solution pool
    std::vector<Solution> solutions;

    void initializeLabels(int num_nodes, int num_res, Graph& graph) {
        auto initialize = [&](LabelDirection dir, std::unordered_map<int, std::vector<Label>>& labels) {
            Label initial_label(graph, dir);
            labels[0].push_back(initial_label);
            for (const Edge& e : graph.getNeighbors(0, dir)) {
                int neighbor = dir == LabelDirection::FORWARD ? e.to : e.from;
                labels[neighbor].push_back(Label(initial_label, graph, e, UB));
                labels[neighbor].back().status = (labels[neighbor].back().status == LabelStatus::NEW_CLOSED) ? LabelStatus::CLOSED : LabelStatus::OPEN;
                labels[neighbor].back().id = 0;
            }
            labels[0].back().status = LabelStatus::CLOSED;
            labels[0].back().id = 0;
        };

        initialize(LabelDirection::FORWARD, fw_labels);
        initialize(LabelDirection::BACKWARD, bw_labels);
    }

    // Add a label into the set of labels while keeping the labels sorted
    // Function to insert a label into the set of labels while keeping the labels sorted
    void InSert(Label& label) {
        int v = label.vertex; // Get the vertex of the label
        auto& labels = label.direction == LabelDirection::FORWARD ? fw_labels : bw_labels; // Select the appropriate label set based on the direction

        // Determine the correct position to insert the label
        auto it = std::lower_bound(labels[v].begin(), labels[v].end(), label, 
            [](const Label& a, const Label& b) { 
            if (a.cost != b.cost) return a.cost < b.cost;
            for (size_t i = 0; i < a.resources.size(); ++i) {
                if (a.resources[i] != b.resources[i]) return a.resources[i] < b.resources[i];
            }
            int a_reachable_count = std::count(a.reachable.begin(), a.reachable.end(), true);
            int b_reachable_count = std::count(b.reachable.begin(), b.reachable.end(), true);
            if (a_reachable_count != b_reachable_count) return a_reachable_count > b_reachable_count;
            return a.id<b.id; // If all are equal, label (input) locates based on the id
            }); // Find the position where the label should be inserted to keep the list sorted

        //change the status of the label
        label.status = (label.status == LabelStatus::NEW_CLOSED) ? LabelStatus::CLOSED : LabelStatus::OPEN;
        // Insert the label at the determined position
        labels[v].insert(it, label); // Insert the label into the list at the correct position

    }

    // Check if new concatenated labels exist in the solution pool
    bool isIDDuplicate(const int vertex, const long long fw_id, const long long bw_id) const {
        // Iterate through all solutions
        for (const Solution& solution : solutions) {
            // Check if the solution ID matches the given forward and backward IDs
            if (solution.ID == std::make_pair(vertex,std::make_pair(fw_id, bw_id))) {
                return true; // Duplicate found
            }
        }
        return false; // No duplicate found
    }

    // Get all labels
    const std::unordered_map<int, std::vector<Label>>& getLabels(bool direction) const {
        if (direction) {
            return fw_labels;
        }
        return bw_labels;
    }

    
    void DominanceCheck(Label& label) {
        int v = label.vertex; // Get the vertex of the label
        auto& labels = label.direction == LabelDirection::FORWARD ? fw_labels : bw_labels; // Select the appropriate label set based on the direction

        bool isDominated = false;

        // Remove labels that are dominated by the new label
        labels[v].erase(std::remove_if(labels[v].begin(), labels[v].end(),
            [&](Label& rival) {
            DominanceStatus status = label.dominance(rival); // Check dominance status between the new label and existing labels
            if (status == DominanceStatus::DOMINATES) {
                rival.status = LabelStatus::DOMINATED; // Mark the rival label as dominated if it is dominated by the new label
            } else if (status == DominanceStatus::DOMINATED) {
                isDominated = true; // Mark the new label as dominated if it is dominated by any existing label
                return false; // Stop further checks if the new label is dominated
            }
            return status == DominanceStatus::DOMINATES; // Remove the rival label if it is dominated by the new label
            }), labels[v].end());

        // If the new label is not dominated by any existing label, insert it
        if (!isDominated) {
            label.id = find_ID(label.direction, label.vertex); // Assign a new unique ID to the label
            InSert(label); // Insert the new label into the set of labels
            // label.display();
        }
    }

    long long find_ID(LabelDirection direction, const int vertex) const {
        // Determine the label set based on the direction
        const auto& labels = (direction == LabelDirection::FORWARD) ? fw_labels.at(vertex) : bw_labels.at(vertex);
        
        // Find the maximum ID in the label set
        long long id_max = 0;
        for (const Label& label : labels) { id_max = id_max>label.id ? id_max : label.id;}
        
        // Return the next ID
        return id_max + 1;
    }
    // Display all labels
    void displayLabels() const {
        for (const auto& pair : fw_labels) {
            for (const Label& label : pair.second) {
                label.display();
            }
        }
        for (const auto& pair : bw_labels) {
            for (const Label& label : pair.second) {
                label.display();
            }

        }
    }

    // Concatenate the forward and backward labels
    void concatenateLabels(const std::vector<double>& res_max) {
        for (const auto& pair : fw_labels) {
            for (const Label& fw_label : pair.second) {
                for (const Label& bw_label : bw_labels[fw_label.vertex]) {
                    std::cout<< "Concatenating Labels with ID: "<< fw_label.vertex<< " , " << bw_label.vertex << " , "<< fw_label.id << " , "<< bw_label.id << "Is possible "<< ( fw_label.isConcatenable(bw_label, res_max)) << std::endl;
                    // && fw_label.isConcatenable(bw_label, res_max)
                    if (!isIDDuplicate(fw_label.vertex, fw_label.id, bw_label.id) ) {
                        std::vector<int> path = fw_label.path;
                        path.pop_back();
                        path.insert(path.end(), bw_label.path.rbegin(), bw_label.path.rend());
                        double cost = fw_label.cost + bw_label.cost;
                        for(size_t i = 0;i<path.size();i++){
                            std::cout << path[i] << " ";
                        }
                        std::cout << "Cost: " << cost << std::endl;
                        if (cost < UB) {
                            solutions.emplace_back(Solution(path, cost, std::make_pair(fw_label.vertex, std::make_pair(fw_label.id, bw_label.id ))));
                            // Update UB
                            UB = std::min(UB, cost);
                        }
                    }
                }
            }
        }
    }


    // Print the solution pool
    void displaySolutions() const {
        for (const Solution& solution : solutions) {
            solution.display();
        }
    }

    void Propagate(Graph& graph, const std::vector<double>& res_max) {
        std::cout << "Propagating Labels" << std::endl;

        // Lambda function to propagate labels in a given direction
        auto propagateDirection = [&](std::unordered_map<int, std::vector<Label>>& labels, LabelDirection direction) {
            for (auto& pair : labels) {
                for (Label& label : pair.second) {
                    if (label.status == LabelStatus::OPEN) {
                        for (const Edge& edge : graph.getNeighbors(label.vertex, direction)) {
                            // Skip if the edge leads back to the start node
                            if ((direction == LabelDirection::FORWARD && label.reachable[edge.to]) || 
                                (direction == LabelDirection::BACKWARD && label.reachable[edge.from])) {
                                Label new_label(label, graph, edge, UB);
                                DominanceCheck(new_label);
                            }
                        }
                        label.status = LabelStatus::CLOSED;
                        break;
                    }
                }
            }
        };

        // Propagate forward and backward labels
        propagateDirection(fw_labels, LabelDirection::FORWARD);
        propagateDirection(bw_labels, LabelDirection::BACKWARD);
    }

    bool TerminationCheck() {
        for (const auto& pair : fw_labels) {
            for (const Label& label : pair.second) {
                if (label.status==LabelStatus::OPEN) {
                    return false;
                }
            }
        }
        for (const auto& pair : bw_labels) {
            for (const Label& label : pair.second) {
                if (label.status == LabelStatus::OPEN) {
                    return false;
                }
            }
        }
        return true;
    }
    void Run(Graph& graph, const std::vector<double>& res_max) {
		int cnt = 0;
        while (!TerminationCheck()) {
            // Generate labels
			std::cout << "Propagating Labels" << std::endl;
            Propagate(graph, res_max);
			std::cout << "Concatenating Labels" << std::endl;
            // Concatenate the forward and backward labels
            concatenateLabels(res_max);
			//displayLabels();
		
        }
        displayLabels();
        std::cout << "Solutions: " << std::endl;
        displaySolutions();
    }

};

int main() {
    // Initialize a label manager
    
    int n = 5, m = 2;
    
    std::vector<double> res_max = { 25.0, 25.0 };

    // Initialize random seed
    std::srand(std::time(nullptr));
    // Create a graph
    Graph graph(n, m, res_max);
    for (int i = 0; i < n; ++i) {
        for (int j = i + 1; j < n; ++j) {
            // Generate m non-negative random numbers
            std::vector<double> randomResources(m);
            for (int k = 0; k < m; ++k) {
                randomResources[k] = ceil(static_cast<double>(std::rand()) / RAND_MAX * 5);
            }
            double cost = (static_cast<double>(std::rand()) / RAND_MAX - 0.5) * 10;
            graph.addEdge(i, j, cost, randomResources);
            graph.addEdge(j, i, cost, randomResources); // Add the reverse edge to make the graph symmetric
        }
    }
    graph.getMaxValue();
    graph.getMinWeights();
    //graph.display();
	std::cout << "Graph created" << std::endl;
	std::cout << "starting the algorithm" << std::endl;
    LabelManager manager(n, m, graph);
	std::cout << "Manager created" << std::endl;
	manager.Run(graph, res_max);
	//manager.displayLabels();
	//manager.Propagate(graph, res_max);

    return 0;
}

  // what does this do?
    //RM: This constructor is used to create a new label from an existing label
  //  Label(const Label& parent, int v, const Graph& graph, const Edge& edge, const double UB)
  //      : vertex(v), path(parent.path), cost(parent.cost),
  //      resources(parent.resources), reachable(parent.reachable),
  //      direction(parent.direction) {}

  //  // Add a node to the label
  //  void addNode(const Edge& edge, const Graph& graph, const double UB) {
		//std::cout << "Adding Vertex: " << vertex << std::endl;
  //      if (direction) {
  //          if (edge.from == vertex) {
		//		std::cout << "edge.from: " << edge.from << " vertex: " << vertex << std::endl;
  //              path.push_back(edge.to);
  //              reachable[edge.to] = false;
  //              cost += edge.cost;//Update cost
		//		std::cout << "cost: " << cost << std::endl;
  //              for (size_t i = 0; i < resources.size(); ++i) {
  //                  resources[i] += edge.resources[i];//update resources
  //                  std::cout << "resources[" << i << "]: " << resources[i] << std::endl;
  //              }
  //              reachHalfPoint(graph.res_max);
  //              for (const Edge& e : graph.getNeighbors(edge.to,false)) {
  //                  if (reachable[e.to] && e.to != 0) {
  //                      for (size_t i = 0; i < resources.size(); ++i) {
  //                          if (resources[i] + e.resources[i] > graph.res_max[i]) {
  //                              reachable[e.to] = false;
  //                              break;
  //                          }
  //                      }
  //                  }
  //              }
  //              // Calculate LB, and compare it with UB
  //              LB = getLB(graph.res_max, graph.num_nodes, graph.min_weight, graph.max_value) + cost;
  //              if (LB > UB) {
  //                  status = LabelStatus::DOMINATED;
  //              }

  //          }
  //          else {

  //          }
  //      }
  //      else {
  //          if (edge.to == vertex) {
		//		std::cout << "edge.to: " << edge.to << " vertex: " << vertex << std::endl;
  //              path.insert(path.begin(), edge.from);
  //              reachable[edge.from] = false;
  //              cost += edge.cost;//Update cost
  //              for (size_t i = 0; i < resources.size(); ++i) {
  //                  resources[i] += edge.resources[i];//update resources
		//			std::cout << "resources[" << i << "]: " << resources[i] << std::endl;
  //              }
  //              reachHalfPoint(graph.res_max);
  //              for (const Edge& e : graph.getNeighbors(edge.from,true)) {
  //                  if (reachable[e.from] && e.from != 0) {
  //                      for (size_t i = 0; i < resources.size(); ++i) {
  //                          if (resources[i] + e.resources[i] > graph.res_max[i]) {
  //                              reachable[e.from] = false;
  //                              break;
  //                          }
  //                      }
  //                  }
  //              }
  //              // Calculate LB, and compare it with UB
  //              LB = getLB(graph.res_max, graph.num_nodes, graph.min_weight, graph.max_value) + cost;
  //              if (LB > UB) {
  //                  status = LabelStatus::DOMINATED;
  //              }
  //          }
  //      }

  //  }