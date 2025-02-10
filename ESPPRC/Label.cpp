#include "Label.h"
#include <iostream>
#include <cmath>
#include <algorithm>

Label::Label(Graph& graph, bool dir)
    : vertex(0), path({ 0 }), cost(0),
    resources(graph.num_res, 0),
    reachable(graph.num_nodes, true),
    direction(dir), LB(0), id(0), rc(graph.num_edges, 0) { // Farzane: initialized "edges()"
    status = LabelStatus::NEW_OPEN;
    reachable[0] = false;
    
	model = std::make_shared<GRBModel>(*graph.model);
	model->update();
	model->optimize();
    LB = model->get(GRB_DoubleAttr_ObjVal);
	//std::cout << "checking undreachable nodes" << std::endl;
    UpdateReachable(graph, 0);
    
}

// Farzane: passed pointer of MIP to the label
Label::Label(const Label& parent, Graph& graph, const Edge* edge, const double UB)
    : path(parent.path), cost(parent.cost),
    resources(parent.resources), reachable(parent.reachable),
    direction(parent.direction) {
	vertex = direction ? edge->to : edge->from;
    cost = parent.cost + edge->cost;

    //Farzane: add the edge data to visited edges by the label
    //edges.push_back(*edge);
    //
	model = std::make_shared<GRBModel>(*graph.model);
	std::string var_name = "x[" + std::to_string(edge->from) + "," + std::to_string(edge->to) + "]";
	model->addConstr(model->getVarByName(var_name) == 1, "edge_" + std::to_string(edge->from) + "_" + std::to_string(edge->to));
	model->update();
	model->optimize();
	LB = model->get(GRB_DoubleAttr_ObjVal);
	//std::cout << "LB: " << LB << std::endl;
	
    if (direction) {
        path.push_back(vertex);
    } else {
        path.insert(path.begin(), vertex);
    }
    reachable[vertex] = false;
    reachable[0] = false;
    for (size_t i = 0; i < resources.size(); ++i) {
        resources[i] += edge->resources[i];
    }
    UpdateReachable(graph, UB);

    // Farzane: get LB and update it
    //LB = mip->solve_with(edges);
	

    // Farzane: commented out
    /*LB = cost+graph.max_value[vertex];
	for (int i = 0; i < reachable.size(); i++) {
		if (reachable[i]&& graph.max_value[i]<0) {
			LB+=graph.max_value[i];
		}
	}*/

    if (reachHalfPoint(graph.res_max, graph.num_nodes)) {
        status = LabelStatus::NEW_CLOSED;
    } else {
        status = LabelStatus::NEW_OPEN;
    }
    
    if (LB > UB) {
        status = LabelStatus::DOMINATED;
		//std::cout << "Pruned" << std::endl;
    }

}


void Label::UpdateReachable(Graph& graph, const double UB) {
	std::string var_name;
    for (const auto e : graph.getNeighbors(vertex, direction)) {
        int neighbor = direction? e->to : e->from;
        if (reachable[neighbor] && neighbor != 0 ) {
			var_name = "x[" + std::to_string(e->from) + "," + std::to_string(e->to) + "]";
            if (model->getVarByName(var_name).get(GRB_DoubleAttr_RC) > UB - LB) {
                reachable[neighbor] = false;
                std::cout << "UB: " << UB << " LB: " << LB;
				std::cout << " RC "+var_name+": " << model->getVarByName(var_name).get(GRB_DoubleAttr_RC) << std::endl;
            }
            else {
                for (size_t i = 0; i < resources.size(); ++i) {
                    if (resources[i] + e->resources[i] > graph.res_max[i]) reachable[neighbor] = false;
                }
            }
        }
    }
	for (int i = 1; i < graph.num_nodes; ++i) {
		var_name = "u[" + std::to_string(i) + "]";

		if (reachable[i] && model->getVarByName(var_name).get(GRB_DoubleAttr_RC) > UB - LB) {
            std::cout << "RC " + var_name + " " << model->getVarByName(var_name).get(GRB_DoubleAttr_RC) << std::endl;
			reachable[i] = false;
		}
	}
    

}

bool Label::reachHalfPoint(const std::vector<double>& res_max, int num_nodes) {
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


void Label::display() const {
    std::cout << "=========================\n";
    std::cout << "Direction: " << (direction? "Forward" : "Backward") << std::endl;
    std::cout << "Path: ";
    for (int node : path) {
        std::cout << node << " ";
    }
    std::cout << ", LB: " << LB << " Cost: " << cost << ", Resources : [";
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
    case LabelStatus::NEW_OPEN:    std::cout << "NEW_OPEN\n"; break;
    case LabelStatus::NEW_CLOSED:  std::cout << "NEW_CLOSED\n"; break;
    case LabelStatus::OPEN:        std::cout << "OPEN\n"; break;
    case LabelStatus::CLOSED:      std::cout << "CLOSED\n"; break;
    case LabelStatus::DOMINATED:   std::cout << "DOMINATED\n"; break;
    }
    std::cout << "ID (vertex, id): ( " << vertex << " , " << id << " )" << std::endl;
    std::cout << "=========================\n\n";
}

DominanceStatus Label::DominanceCheck(const Label& rival) const {
    bool dominates = true, dominated = true;

    for (size_t i = 0; i < resources.size(); ++i) {
        if (resources[i] > rival.resources[i]) dominates = false;
        if (resources[i] < rival.resources[i]) dominated = false;
        if (!dominates && !dominated) return DominanceStatus::INCOMPARABLE;
    }

    for (size_t i = 0; i < reachable.size(); ++i) {
        if (reachable[i] < rival.reachable[i]) dominates = false;
        if (reachable[i] > rival.reachable[i]) dominated = false;
        if (!dominates && !dominated) return DominanceStatus::INCOMPARABLE;
    }

    if (dominates && cost <= rival.cost) return DominanceStatus::DOMINATES;
    if (dominated && cost >= rival.cost) return DominanceStatus::DOMINATED;
    return DominanceStatus::INCOMPARABLE;
}

bool Label::isConcatenable(const Label& bw_label, const std::vector<double>& r_max) const {
    for (size_t i = 0; i < resources.size(); ++i) {
        if (resources[i] + bw_label.resources[i] > r_max[i]) {
            return false;
        }
    }

    std::unordered_set<int> first_set(path.begin() + 1, path.end());
    return std::none_of(bw_label.path.begin() + 1, bw_label.path.end() - 1,
        [&first_set](const int& x) { return first_set.count(x) > 0; });
}

//double Label::RighiniLB(Graph& graph) {
//    GRBEnv env = GRBEnv(true);
//    env.set(GRB_IntParam_OutputFlag, 0);
//    env.set(GRB_IntParam_LogToConsole, 0);
//    env.start();
//    double c = 0;
//	GRBLinExpr obj = 0;
//	std::map<int, GRBVar> y;
//    std::map<int, GRBLinExpr> r_lhs;
//    int num_res = graph.res_max.size();
//    std::vector<double> r(num_res,0);
//    
//    // Starting LP relaxation of the problem
//    GRBModel mdl = GRBModel(env);
//    for (int i = 0; i < graph.num_nodes; ++i) {
//        c = 1000;
//        for (int k = 0; k < num_res; k++) r[k] = 1000;
//        if (i == vertex || i == 0 || reachable[i]) {
//            for (const auto e : direction ? graph.OutList[i] : graph.InList[i]) {
//                c = std::min(c, e->cost);
//				for (int k = 0; k < num_res; k++) r[k] = std::min(r[k], e->resources[k]);
//            }
//
//            if (i != vertex) {
//                y[i] = mdl.addVar(0, 1, c, GRB_CONTINUOUS);
//                obj += y[i] * c;
//				for (int k = 0; k < num_res; k++) r_lhs[k] += y[i] * r[k];
//            }
//            else {
//                obj += c;
//                for (int k = 0; k < num_res; k++) r_lhs[k] += r[k];
//            }
//			
//        }
//	}
//	for (int k = 0; k < num_res; k++) {
//		mdl.addConstr(r_lhs[k]+resources[k]<= graph.res_max[k]);
//	}
//	mdl.setObjective(obj, GRB_MINIMIZE);
//
//	mdl.update();
//	mdl.optimize();
//	return mdl.get(GRB_DoubleAttr_ObjVal) + cost;
//	}
