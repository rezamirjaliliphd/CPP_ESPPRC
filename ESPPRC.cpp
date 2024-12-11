#include <iostream>
#include <vector>
#include <functional>
#include <algorithm>
#include <cstdlib>
#include <ctime>
#include <cmath>

enum class NodeStatus {
    UNVISITED,
    OPEN,
    CLOSED
};
enum class LabelStatus {
    DOMINATED,
    INCOMPARABLE
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

class Graph {
private:
    std::vector<std::vector<Edge>> adjList; // Adjacency list 

public:
    // Constructor
    Graph(int n) : adjList(n) {}

    // Add an edge to the graph
    void addEdge(int from, int to, double cost, const std::vector<double>& resources) {
        adjList[from].emplace_back(Edge(from, to, cost, resources));
    }

    // Get the neighbors of a node
    const std::vector<Edge>& getNeighbors(int node) const {
        return adjList[node];
    }
    // Display the graph
    void display() const {
        for (int i = 0; i < adjList.size(); ++i) {
            std::cout << "Node " << i << ":\n";
            for (const Edge& edge : adjList[i]) {
                std::cout << "  -> " << edge.to << " (Cost: " << edge.cost << ", Resources: [";
                for (size_t j = 0; j < edge.resources.size(); ++j) {
                    std::cout << edge.resources[j] << (j + 1 < edge.resources.size() ? ", " : "");
                }
                std::cout << "])\n";
            }
        }
    }
};

// Class to represent a label (partial path)
class Label {
public:
    int  vertex;                     // Current vertex
    std::vector<int> path;            // Nodes in the current path
    double cost;                      // Cost of the path
    std::vector<double> resources;    // Resource consumption of the path
    std::vector<bool> reachable;      // Reachability of the nodes
    bool half_point = false;   // Half point reached or not
    // Constructor
    
    Label(const int n,const std::vector<int>& p, double c, const std::vector<double>& r) 
        : vertex(p.back()), path(p), cost(c), resources(r), reachable(n, true) {
            reachable[0] = false;
        }

    Label(const Label& parent, int v) 
        : vertex(v), path(parent.path), cost(parent.cost), resources(parent.resources), reachable(parent.reachable) {}

    // Add a node to the path
    void addNode(const Edge& edge, const Graph& graph, const std::vector<double>& res_max) {
        if (edge.from == path.back()) {
            path.push_back(edge.to);
            reachable[edge.to] = false;
            cost += edge.cost;
            for (size_t i = 0; i < resources.size(); ++i) {
                resources[i] += edge.resources[i];
            }
            for (const Edge& e : graph.getNeighbors(edge.to)) {
                for (size_t i = 0; i < resources.size(); ++i) {
                    if (resources[i] + e.resources[i] > res_max[i]){
                        reachable[e.to] = false;
                        break;
                    }
                }
            }
            
        }
    }

    // Reaches half=point
    void reachHalfPoint(const std::vector<double>& res_max) {
        for (size_t i = 0; i < resources.size(); ++i) {
            if (resources[i] >= res_max[i]/2){
                half_point = true;
                break;
            }
        }
    }

    double getLB(){
        std::cout << "Calculating LB " << std::endl;
        return -1.0;
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
            if (reachable[i]){
                std::cout << "Node " << i << " is reachable\n";
            }
        }
        
    }

    // Compare two labels based on cost
    LabelStatus dominance(const Label& rival) const {
        if (rival.cost>=cost){
            for (size_t i = 0; i < resources.size(); ++i) {
                if (rival.resources[i]<resources[i]){
                    return LabelStatus::INCOMPARABLE;
                }
            }
            for (size_t i = 0; i < reachable.size(); ++i) {
                if (rival.reachable[i] < reachable[i]){
                    return LabelStatus::INCOMPARABLE;
                }
            }
            return LabelStatus::DOMINATED;

        }
        return LabelStatus::INCOMPARABLE;         
    }
};

// Class to manage labels
class LabelManager {
private:
    std::vector<Label> labels; // Collection of labels

public:
    // Add a label
    void addLabel(const Label& label) {
        labels.push_back(label);
    }

    // Get all labels
    const std::vector<Label>& getLabels() const {
        return labels;
    }

    // Prune labels based on a condition
    void pruneLabels(const std::function<bool(const Label&)>& condition) {
        labels.erase(std::remove_if(labels.begin(), labels.end(), condition), labels.end());
    }

    // Display all labels
    void displayLabels() const {
        for (const auto& label : labels) {
            label.display();
        }
    }
};

int main() {
    // Initialize a label manager
    LabelManager manager;
    int n= 5, m =2;
    

    // Initialize random seed
    std::srand(std::time(nullptr));
    // Create a graph
    Graph graph(n);
    for (int i = 0; i < n; ++i) {
        for (int j = 0; j < n ; ++j) {
            if (i!=j){
                // Generate m non-negative random numbers
                std::vector<double> randomResources(m);
                for (int k = 0; k < m; ++k) {
                    if (static_cast<double>(std::rand()) / RAND_MAX>0.5){
                    randomResources[k] = static_cast<double>(std::rand()) / RAND_MAX*5;}
                    else{
                        randomResources[k] = ceil(static_cast<double>(std::rand()) / RAND_MAX*5);
                    }
                graph.addEdge(i, j, (static_cast<double>(std::rand()) / RAND_MAX-0.5)*10, randomResources);
            }
            }
        }
    }
    graph.display();
    // Create initial label (starting from node 0)
    Label initialLabel(n, {0}, 0.0, {0.0, 0.0}); // Path: {0}, Cost: 0.0, Resources: [0.0, 0.0]
    manager.addLabel(initialLabel);
    for(const Edge& edge : graph.getNeighbors(0)){
        Label newLabel(initialLabel);
        newLabel.addNode(edge, graph, {5.0, 5.0});
        newLabel.reachHalfPoint({5.0, 5.0});
        manager.addLabel(newLabel);
    }

    // Simulate adding labels
    // Label label1 = initialLabel;
    // label1.addNode(1, 10.0, {5.0, 2.0});
    // manager.addLabel(label1);

    // Label label2 = label1;
    // label2.addNode(2, 15.0, {3.0, 1.0});
    // manager.addLabel(label2);

    // Label label3 = label1;
    // label3.addNode(3, 12.0, {4.0, 1.5});
    // manager.addLabel(label3);

    // // Display all labels
    // std::cout << "All Labels:\n";
    // manager.displayLabels();

    // // Prune labels with cost > 20
    // manager.pruneLabels([](const Label& label) { return label.cost > 1000.0; });

    // Display remaining labels
    std::cout << "\nLabels after pruning (Cost <= 20):\n";
    manager.displayLabels();

    return 0;
}
