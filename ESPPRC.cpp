#include "gurobi_c++.h"
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

/*
* TODO: In the course of updating UB, make sure to get rid of the labels whose LB are greater than new UB
* TODO: Concatenate the forward and backward labels
*/


enum class LabelStatus {
    DOMINATED,
    INCOMPARABLE,
    DOMINATES
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
    std::pair<long long, long long> ID;
    Solution(const std::vector<int>& p, double c, std::pair<long long, long long> id) : path(p), cost(c), ID(id) {}
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
    const std::vector<Edge>& getNeighbors(int node, bool IN) const {
		return IN ? InList[node] : OutList[node];
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
    long long id;                  // Unique identifier
    int  vertex;                     // Current vertex
    std::vector<int> path;            // Nodes in the current path
    double cost;                      // Cost of the path
    std::vector<double> resources;    // Resource consumption of the path
    std::vector<bool> reachable;      // Reachability of the nodes
    bool half_point = false;   // Half point reached or not
    bool direction;      // Forward:true or backward:false
    double LB;      // Lower bound
    LabelStatus status = LabelStatus::INCOMPARABLE;     // Status of the label


    // Constructor
   
	Label(Graph& graph, bool dir) : vertex(0), path({ 0 }), cost(0), resources(graph.num_res), reachable(graph.num_nodes, true), direction(dir), LB(0), id(0) {
		reachable[0] = false;
	}

    // what does this do?
    Label(const Label& parent, int v, const Graph& graph, const Edge& edge, const double UB)
        : vertex(v), path(parent.path), cost(parent.cost),
        resources(parent.resources), reachable(parent.reachable),
        direction(parent.direction) {
        addNode(edge, graph, UB);
    }

    // Add a node to the label
    void addNode(const Edge& edge, const Graph& graph, const double UB) {
        if (direction) {
            if (edge.from == vertex) {
				std::cout << "edge.from: " << edge.from << " vertex: " << vertex << std::endl;
                path.push_back(edge.to);
                reachable[edge.to] = false;
                cost += edge.cost;//Update cost
                for (size_t i = 0; i < resources.size(); ++i) {
                    resources[i] += edge.resources[i];//update resources
                }
                reachHalfPoint(graph.res_max);
                for (const Edge& e : graph.getNeighbors(edge.to,false)) {
                    if (reachable[e.to] && e.to != 0) {
                        for (size_t i = 0; i < resources.size(); ++i) {
                            if (resources[i] + e.resources[i] > graph.res_max[i]) {
                                reachable[e.to] = false;
                                break;
                            }
                        }
                    }
                }
                // Calculate LB, and compare it with UB
                LB = getLB(graph.res_max, graph.num_nodes, graph.min_weight, graph.max_value) + cost;
                if (LB > UB) {
                    status = LabelStatus::DOMINATED;
                }

            }
            else {

            }
        }
        else {
            if (edge.to == vertex) {
				std::cout << "edge.to: " << edge.to << " vertex: " << vertex << std::endl;
                path.insert(path.begin(), edge.from);
                reachable[edge.from] = false;
                cost += edge.cost;//Update cost
                for (size_t i = 0; i < resources.size(); ++i) {
                    resources[i] += edge.resources[i];//update resources
                }
                reachHalfPoint(graph.res_max);
                for (const Edge& e : graph.getNeighbors(edge.from,true)) {
                    if (reachable[e.from] && e.from != 0) {
                        for (size_t i = 0; i < resources.size(); ++i) {
                            if (resources[i] + e.resources[i] > graph.res_max[i]) {
                                reachable[e.from] = false;
                                break;
                            }
                        }
                    }
                }
                // Calculate LB, and compare it with UB
                LB = getLB(graph.res_max, graph.num_nodes, graph.min_weight, graph.max_value) + cost;
                if (LB > UB) {
                    status = LabelStatus::DOMINATED;
                }
            }
        }

    }

    // Reaches half=point
    void reachHalfPoint(const std::vector<double>& res_max) {
        for (size_t i = 0; i < resources.size(); ++i) {
            if (resources[i] >= res_max[i] / 2) {
                half_point = true;
                break;
            }
        }
    }

    double getLB(const std::vector<double>& res_max,
        const int n,
        const std::vector<std::vector<double>>& min_weight,
        const std::vector<double>& max_value) {

        GRBEnv env = GRBEnv();
        env.set(GRB_IntParam_OutputFlag, 0);
        GRBModel model = GRBModel(env);
        std::vector<int> Items;


        GRBLinExpr obj = 0;
        std::vector<GRBVar> x(n);
        for (int i = 0; i < reachable.size(); i++) {
            int ub = reachable[i] ? 1 : 0;
            x[i] = model.addVar(0.0, ub, 0, GRB_BINARY, "x" + std::to_string(i));
            obj -= x[i] * max_value[i];
        }

        model.setObjective(obj, GRB_MAXIMIZE);

        for (int k = 0; k < res_max.size(); k++) {
            GRBLinExpr cntr = 0;
            for (int j = 0; j < x.size(); j++) {
                cntr += x[j] * min_weight[j][k];
            }
            model.addConstr(cntr <= res_max[k], "resource " + std::to_string(k));

        }
        // Optimize the model
        model.optimize();
        return model.get(GRB_DoubleAttr_ObjVal);
    }



    // Display the label
    void display() const {
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

    }

    // Compare two labels based on cost
    LabelStatus dominance(const Label& rival) const {
        if (rival.cost >= cost) {

            for (size_t i = 0; i < resources.size(); ++i) {
                if (rival.resources[i] < resources[i]) {
                    return LabelStatus::INCOMPARABLE;
                }
            }
            for (size_t i = 0; i < reachable.size(); ++i) {
                if (rival.reachable[i] < reachable[i]) {
                    return LabelStatus::INCOMPARABLE;
                }
            }
            return LabelStatus::DOMINATED;

        }
        else {
            for (size_t i = 0; i < resources.size(); ++i) {
                if (rival.resources[i] > resources[i]) {
                    return LabelStatus::INCOMPARABLE;
                }
            }
            for (size_t i = 0; i < reachable.size(); ++i) {
                if (rival.reachable[i] > reachable[i]) {
                    return LabelStatus::INCOMPARABLE;
                }
            }

            return LabelStatus::DOMINATES;
        }
        return LabelStatus::INCOMPARABLE;
    }

    // if label is concatable with the other label
    bool isConcatenable(const Label& label, const std::vector<double>& r_max) const {
        // considering only the forward labels
        if (vertex == label.vertex && label.direction != direction && direction) {
            for (size_t i = 0; i < resources.size(); ++i) {
                if (resources[i] + label.resources[i] > r_max[i]) {
                    return false;
                }
            }
            for (size_t i = 1; i < reachable.size(); ++i) {
                if (reachable[i] && label.reachable[i] && i != vertex) {
                    return false;
                }
            }
            return true;
        }
        return false;
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
        initializeLabels(num_nodes, num_res, graph);
    }

    double UB = 1000;
    // Solution pool
    std::vector<Solution> solutions;

    void initializeLabels(int num_nodes, int num_res, Graph& graph) {
        // Create the initial forward label at the source
        Label source_label(graph, true);//n p c r dr lb
        fw_labels[0].push_back(source_label);
        for (const Edge& e : graph.getNeighbors(0, false)) {
			std::cout << "e.from: " << e.from << " e.to: " << e.to << std::endl;
            fw_labels[e.to].push_back(Label(source_label, e.to, graph, e, UB));
        }
        // Create the initial backward label at the sink
        Label sink_label(graph, false);
        bw_labels[0].push_back(sink_label);
        for (const Edge& e : graph.getNeighbors(0, true)) {
			std::cout << "e.from: " << e.from << " e.to: " << e.to << std::endl;
			bw_labels[e.from].push_back(Label(sink_label, e.from, graph, e, UB));
		}
    }

    // Add a label into the set of labels while keeping the labels sorted
    void InSert(const Label& label) {

        int v = label.vertex;
        // Farzane's Update
        if (label.direction) {
            // if label's cost smaller than index =0;
            if (label.cost < fw_labels.at(v)[0].cost) {
                fw_labels.at(v).insert(fw_labels.at(v).begin(), label);

            }
            // if label's cost greater than the last label
            else if (label.cost > fw_labels.at(v).back().cost) {
                fw_labels.at(v).push_back(label);
            }
            // if label's cost is somewhere in the middle
            else {
                size_t left = 1, right = fw_labels.at(v).size() - 1, mid;

                while (left < right) {

                    mid = static_cast<size_t>(left + (right - left) / 2);

                    if (label.cost < fw_labels.at(v)[mid].cost) {
                        right = mid; // Narrow the range to the left half
                    }
                    else {
                        left = mid + 1; // Narrow the range to the right half
                    }
                }

                // Insert the label at the correct position after narrowing down the range
                fw_labels.at(v).insert(fw_labels.at(v).begin() + left, label);

                //for (size_t i = 1; i < fw_labels.at(v).size() - 2; i++) {
                    //if (label.cost >= fw_labels.at(v)[i].cost && label.cost <= fw_labels.at(v)[i + 1].cost) {
                        //fw_labels.at(v).insert(fw_labels.at(v).begin() + i + 1, label);
                    //}
            }
        }
        else {
            // if label's cost smaller than index =0;
            if (label.cost < bw_labels.at(v)[0].cost) {
                bw_labels.at(v).insert(bw_labels.at(v).begin(), label);
            }
            // if label's cost greater than the last label
            if (label.cost > bw_labels.at(v).back().cost) {
                bw_labels.at(v).push_back(label);
            }
            for (int i = 0; i < bw_labels.at(v).size() - 1; i++) {
                if (label.cost >= bw_labels.at(v)[i].cost && label.cost <= bw_labels.at(v)[i + 1].cost) {
                    bw_labels.at(v).insert(bw_labels.at(v).begin() + i + 1, label);
                }
            }

        }

    }

    // Check if new concatenated labels exist in the solution pool
    bool isIDDuplicate(const long long fw_id, const long long bw_id) const {
        for (const Solution& solution : solutions) {
            if (solution.ID == std::make_pair(fw_id, bw_id)) {
                return true;
            }
        }
        return false;
    }

    // Get all labels
    const std::unordered_map<int, std::vector<Label>>& getLabels(bool direction) const {
        if (direction) {
            return fw_labels;
        }
        return bw_labels;
    }

    // Prune labels based on a condition
    void DominanceCheck(Label& label) {
        bool eligible_to_insert = true;
        int v = label.vertex;
        bool dir = label.direction;
        if (label.direction) {
            for (auto& rival : fw_labels.at(label.vertex)) {
                LabelStatus status = label.dominance(rival);
                if (status == LabelStatus::DOMINATED) {
                    rival.status = LabelStatus::DOMINATED;
                }
                else if (status == LabelStatus::DOMINATES) {
                    eligible_to_insert = false;
                    break;
                }
            }
        }
        else {
            for (auto& rival : bw_labels.at(label.vertex)) {
                LabelStatus status = label.dominance(rival);
                if (status == LabelStatus::DOMINATED) {
                    rival.status = LabelStatus::DOMINATED;
                }
                else if (status == LabelStatus::DOMINATES) {
                    eligible_to_insert = false;
                    break;
                }
            }

        }
        // If the label is not dominated by any other label, insert it
        if (eligible_to_insert) {
            label.id = find_ID(label.direction, label.vertex);
            InSert(label);
        }
        else {
            delete& label;
        }
        // Get rid of dominated labels
        if (dir) {
            fw_labels.at(v).erase(std::remove_if(fw_labels.at(v).begin(), fw_labels.at(v).end(),
                [](const Label& label) { return label.status == LabelStatus::DOMINATED; }), fw_labels.at(v).end());
        }
        else {
            bw_labels.at(v).erase(std::remove_if(bw_labels.at(v).begin(), bw_labels.at(v).end(),
                [](const Label& label) { return label.status == LabelStatus::DOMINATED; }), bw_labels.at(v).end());
        }

    }

    long long find_ID(bool direction, const int vertex) const {
        long long id_max = 0;
        if (direction) {
            for (const Label& label : fw_labels.at(vertex)) {
                id_max = std::max(id_max, label.id);
            }
        }
        else {
            for (const Label& label : bw_labels.at(vertex)) {
                id_max = std::max(id_max, label.id);
            }
        }
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
                for (const Label& bw_label : bw_labels.at(fw_label.vertex)) {
                    if (!isIDDuplicate(fw_label.id, bw_label.id) && fw_label.isConcatenable(bw_label, res_max)) {
                        std::vector<int> path = fw_label.path;
                        path.pop_back();
                        path.insert(path.end(), bw_label.path.rbegin(), bw_label.path.rend());
                        double cost = fw_label.cost + bw_label.cost;
                        if (cost < UB) {
                            solutions.emplace_back(Solution(path, cost, { fw_label.id, bw_label.id }));
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


    //TODO: when a label propagates, half point should be True
    // Select a label
    void Propagate(Graph& graph, const std::vector<double>& res_max) {
        for (const auto& pair : fw_labels) {
            for (const Label& label : pair.second) {
                if (!label.half_point) {
                    for (const Edge& edge : graph.getNeighbors(label.vertex,false)) {
                        Label new_label(label, edge.to, graph, edge, UB);
                        
                        DominanceCheck(new_label);
                    }
                    break;
                }
            }
        }
        for (const auto& pair : bw_labels) {
            for (const Label& label : pair.second) {
                if (!label.half_point) {
                    for (const Edge& edge : graph.getNeighbors(label.vertex,true)) {
                        Label new_label(label, edge.from, graph, edge, UB);
                        DominanceCheck(new_label);
                    }
                    break;
                }
            }
        }

    }

    bool TerminationCheck() {
        for (const auto& pair : fw_labels) {
            for (const Label& label : pair.second) {
                if (label.status == LabelStatus::INCOMPARABLE && !label.half_point) {
                    return false;
                }
            }
        }
        for (const auto& pair : bw_labels) {
            for (const Label& label : pair.second) {
                if (label.status == LabelStatus::INCOMPARABLE && !label.half_point) {
                    return false;
                }
            }
        }
        return true;
    }
    void Run(Graph& graph, const std::vector<double>& res_max) {
        while (!TerminationCheck()) {
            // Generate labels
            Propagate(graph, res_max);
            // Concatenate the forward and backward labels
            concatenateLabels(res_max);
        }
    }

};

int main() {
    // Initialize a label manager
    
    int n = 5, m = 2;
    
    std::vector<double> res_max = { 5.0, 5.0 };

    // Initialize random seed
    std::srand(std::time(nullptr));
    // Create a graph
    Graph graph(n, m, res_max);
    for (int i = 0; i < n; ++i) {
        for (int j = 0; j < n; ++j) {
            if (i != j) {
                // Generate m non-negative random numbers
                std::vector<double> randomResources(m);
                for (int k = 0; k < m; ++k) {
                    if (static_cast<double>(std::rand()) / RAND_MAX > 0.5) {
                        randomResources[k] = static_cast<double>(std::rand()) / RAND_MAX * 5;
                    }
                    else {
                        randomResources[k] = ceil(static_cast<double>(std::rand()) / RAND_MAX * 5);
                    }
                    graph.addEdge(i, j, (static_cast<double>(std::rand()) / RAND_MAX - 0.5) * 10, randomResources);
                }
            }
        }
    }
    graph.getMaxValue();
    graph.getMinWeights();
    //graph.display();

    LabelManager manager(n, m, graph);
	manager.displayLabels();
	//manager.Propagate(graph, res_max);

    return 0;
}