// Graph.cpp

#include "Graph.h"
#include <iostream>

// Constructor
Graph::Graph(int n, int m, std::vector<double> r_max)
    : OutList(n), InList(n), num_nodes(n), num_res(m),
    min_weight(n, std::vector<double>(m, 100.0)),
    max_value(n, 100.0), res_max(r_max) {
	OutList.resize(n); InList.resize(n);
    predecessor = std::vector<std::vector<bool>>(n, std::vector<bool>(n, false));
}

// Method to add an edge to the graph
void Graph::addEdge(int from, int to, double cost, const std::vector<double>& resources) {
	auto edge = make_shared<Edge>(from, to, cost, resources);
    OutList[from].push_back(edge);
    InList[to].push_back(edge);
	edges.push_back(edge);
    num_edges += 1;
	predecessor[from][to] = true;
}

// Method to get neighbors of a node
const std::vector<std::shared_ptr<Edge>> Graph::getNeighbors(int node, bool dir) const {
    return dir? OutList[node] : InList[node];
}

// Method to display the graph
void Graph::display() const {
    for (int i = 0; i < num_nodes; ++i) {
        std::cout << "Node " << i << ":\n";
        for (const auto edge : OutList[i]) {
            std::cout << "  -> " << edge->to << " (Cost: " << edge->cost << ", Resources: [";
            for (size_t j = 0; j < edge->resources.size(); ++j) {
                std::cout << edge->resources[j] << (j + 1 < edge->resources.size() ? ", " : "");
            }
            std::cout << "])\n";
        }
    }
}

// Method to get minimum weights
std::vector<std::vector<double>> Graph::getMinWeights() {
    for (int i = 0; i < num_nodes; i++) {
        for (const auto edge : OutList[i]) {
            for (int k = 0; k < edge->resources.size(); k++) {
                if (min_weight[i][k] > edge->resources[k]) {
                    min_weight[i][k] = edge->resources[k];
                }
            }
        }
    }
    return min_weight;
}
void Graph::deleteEdge(int from, int to ) {
    // Remove (v, w) from OutList[u]

    OutList[from].erase(
        std::remove_if(OutList[from].begin(), OutList[from].end(),
            [to](std::shared_ptr<Edge> e) { return to ==e->to; }),
        OutList[from].end()
    );

    // Remove (u, w) from InList[v]
    InList[to].erase(
        std::remove_if(InList[to].begin(), InList[to].end(),
            [from](std::shared_ptr<Edge> edge) { return edge->from == from; }),
        InList[to].end()
    );
	predecessor[from][to] = false;

}
// Method to get maximum values
void Graph::getMaxValue() {
    for (int i = 0; i < num_nodes; i++) {
        for (const auto edge : OutList[i]) {
            if (max_value[i] > edge->cost) {
                max_value[i] = edge->cost;
            }
        }
		//std::cout << "Node " << i << " Min Value: " << max_value[i] << std::endl;
    }
}
bool Graph::is_neighbor(const int from, const int to) const {
	return predecessor[from][to];
}

Edge& Graph::getEdge(int from, int to) const {
	for (const auto edge : OutList[from]) {
		if (edge->to == to) {
			return *edge;
		}
	}
}

void Graph::buildBaseModel(bool LP_relaxation) {
	GRBEnv env = GRBEnv();
    env.set(GRB_IntParam_OutputFlag, 0);
    env.set(GRB_IntParam_LogToConsole, 0);
    env.start();
	model = std::make_shared<GRBModel>(env);
	
    GRBLinExpr obj = 0, inflow, outflow;
	int cnt = 0;
    
	std::vector<GRBVar> s;
    std::string name;
    // Define variables
    for (int i = 0; i < num_nodes; ++i) {
        for (const auto e : OutList[i]) {
            name = "x[" + std::to_string(i) + ", " + std::to_string(e->to) + "]";
            x[{i, e->to}] = std::make_shared<GRBVar>(model->addVar(0, 1, ROUND(e->cost,0), !LP_relaxation ? GRB_BINARY : GRB_CONTINUOUS, name));
            obj += *x[{i, e->to}] * ROUND(e->cost, 0);
        }
    }

    for (int i = 0; i < num_nodes; i++) {
        name = "u[" + std::to_string(i) + "]";
        u[i] = std::make_shared<GRBVar>(model->addVar(0.0, num_nodes, 0.0, !LP_relaxation ? GRB_INTEGER : GRB_CONTINUOUS, name));
    }

    // Flow conservation constraints
    for (int i = 0; i < num_nodes; i++) {
        outflow = 0;
        inflow = 0;
        for (const auto e : OutList[i]) {
            outflow += *x[{e->from, e->to}];
        }
        for (const auto e : InList[i]) {
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
    for (int k = 0; k < num_res; ++k) {
        GRBLinExpr constraint = 0;
        for (int i = 0; i < num_nodes; ++i) {
            for (const auto e : OutList[i]) {
                constraint += *x[{e->from, e->to}] * e->resources[k];
            }
        }
        s.push_back(model->addVar(0, GRB_INFINITY, 0, GRB_CONTINUOUS, "s[" + std::to_string(cnt++)+"]"));
        model->addConstr(constraint+s.back()== res_max[k], "resource_" + std::to_string(k));
    }

    // Subtour elimination constraints
    for (int i = 0; i < num_nodes; i++) {
        for (const auto& e : OutList[i]) {
            name = "subtour_" + std::to_string(i) + "_" + std::to_string(e->to);
            if (e->from == 0 || e->to == 0) continue;
			s.push_back(model->addVar(0, GRB_INFINITY, 0, GRB_CONTINUOUS, "s[" + std::to_string(cnt++) + "]"));
            model->addConstr(*u[e->from] + 1+s.back() == *u[e->to] + num_nodes * (1 - *x[{e->to, e->from}]), name);
        }
    }
    model->setObjective(obj, GRB_MINIMIZE); // Set objective function
	model->update();
	model->optimize();
	/*GRBLinExpr lhs = 0;
	double rc = 0;*/
  //  while (true) {
  //      GRBVar* vars = model->getVars();
  //      int numvars = model->get(GRB_IntAttr_NumVars);
  //      for (int i = 0; i < numvars; i++) {
		//	rc = vars[i].get(GRB_DoubleAttr_RC);
		//	//std::cout << "RC of " << vars[i].get(GRB_StringAttr_VarName) << " is " << ROUND(rc, 3) << std::endl;
		//	lhs += (ROUND(rc,3) - floor(ROUND(rc,3))) * vars[i];
  //      }
		//
		//s.push_back(model->addVar(0, GRB_INFINITY, 0, GRB_CONTINUOUS, "s[" + std::to_string(cnt++) + "]"));
		//double rhs = 1 - ROUND(model->get(GRB_DoubleAttr_ObjVal), 3) + floor(ROUND(model->get(GRB_DoubleAttr_ObjVal), 3));
  //      std::cout << "RHS:" << rhs << std::endl;
		//model->addConstr(lhs-s.back() == rhs,"rounding");
		//model->update();
		//model->optimize();
		//std::cout << "Objective value: " << model->get(GRB_DoubleAttr_ObjVal)/ 100000 << std::endl;
  //      if (rhs <= 0.1) break;

  //  }
}

std::pair<std::map<std::pair<int, int>, double>, double> Graph::getRCLabel(const std::vector<int>& p) {
    std::map<std::pair<int, int>, double> rc;
	GRBConstr constr;
	GRBLinExpr lhs = 0;
    double objVal = -1.0;  // Initialize to an invalid value
    if (p.size() == 1) {

        for (int i = 0; i < num_nodes; i++) {
            for (const auto& e : OutList[i]) rc[{e->from, e->to}] = x[{e->from, e->to}]->get(GRB_DoubleAttr_RC);
		}
		objVal = model->get(GRB_DoubleAttr_ObjVal);
		return std::make_pair(rc, objVal);
    }
    for (size_t i = 0; i < p.size() - 1; ++i) {
		//std::cout << "Adding constraint for edge " << p[i] << " -> " << p[i + 1] << std::endl;
        x[{ p[i], p[i + 1] }]->set(GRB_DoubleAttr_LB, 1);  // Set the lower bound to 1
    }
    model->update();  // Ensure changes are reflected in the model
    model->optimize();  // Perform the optimization
    objVal = model->get(GRB_DoubleAttr_ObjVal);
    for (int i = 0; i < num_nodes; i++) {
        for (const auto& e : OutList[i]) rc[{e->from, e->to}] = x[{e->from, e->to}]->get(GRB_DoubleAttr_RC);
    }
	//model->remove(constr);
    for (size_t i = 0; i < p.size() - 1; ++i) {
        x[{ p[i], p[i + 1] }]->set(GRB_DoubleAttr_LB, 0);  // Set the lower bound to 0
    }
    model->update();  // Ensure changes are reflected in the model
    model->optimize();  // Perform the optimization
    return std::make_pair(rc, objVal);
}

