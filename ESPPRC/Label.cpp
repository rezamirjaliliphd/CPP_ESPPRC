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
    LB = graph.model->get(GRB_DoubleAttr_ObjVal);
    UpdateReachable(graph, 0);   
}

// Farzane: passed pointer of MIP to the label
Label::Label(const Label& parent, Graph& graph, const Edge* edge, const double UB)
    : path(parent.path), cost(parent.cost),
    resources(parent.resources), reachable(parent.reachable),
    direction(parent.direction) {
    vertex = direction ? edge->to : edge->from;
    cost = parent.cost + edge->cost;
    model = std::make_shared<GRBModel>(*parent.model);
    std::string var_name = "x[" + std::to_string(edge->from) + "," + std::to_string(edge->to) + "]";
    model->getVarByName(var_name).set(GRB_DoubleAttr_LB,1);
    model->update();
    model->optimize();
    //LB = model->get(GRB_DoubleAttr_ObjVal);
	//std::cout << "Graph model objective value: " << graph.model->get(GRB_DoubleAttr_ObjVal)<< " Label model: "<< model->get(GRB_DoubleAttr_ObjVal) << std::endl;
    LB = model->get(GRB_DoubleAttr_ObjVal);
    if (direction) path.push_back(vertex);
    else path.insert(path.begin(), vertex);

    reachable[vertex] = false;
    reachable[0] = false;
    for (size_t i = 0; i < resources.size(); ++i) {
        resources[i] += edge->resources[i];
    }
    UpdateReachable(graph, UB);
	
    


    if (reachHalfPoint(graph.res_max, graph.num_nodes)) {
        status = LabelStatus::NEW_CLOSED;
    }
    else {
        status = LabelStatus::NEW_OPEN;
    }

    if (LB > UB) {
        status = LabelStatus::DOMINATED;
        //std::cout << "Pruned" << std::endl;
    }

}


void Label::getUpdateMinRes(Graph& graph) {
    for (int i = 0; i < graph.num_nodes; i++) {
        if ((i == 0)||(i==vertex)|| reachable[i]) {
            for (const auto e : graph.getNeighbors(i, direction)) {
                int neighbor = direction ? e->to : e->from;
                if (reachable[neighbor]) {
					for (int k = 0; k < graph.num_res; k++) {
						if (min_res.find({ i,k }) == min_res.end())  min_res[{i, k}] = e->resources[k];
						else min_res[{i, k}] = std::min(min_res[{i, k}], e->resources[k]);
					}
                  
                }
            }
        
        }
    }
}
void Label::UpdateReachable(Graph& graph, const double UB) {
    std::string var_name;
    bool ind = true;
    for (int i = 1; i < graph.num_nodes; ++i) {
        ind = (reachable[i] && model->getVarByName("u[" + std::to_string(i) + "]").get(GRB_DoubleAttr_RC) > UB - LB);
		ind = ind && (model->getVarByName("y[" + std::to_string(i) + "]").get(GRB_DoubleAttr_RC) > UB - LB);
        if (ind) reachable[i] = false;
    }
    
    for (const auto e : graph.getNeighbors(vertex, direction)) {
        int neighbor = direction ? e->to : e->from;
        if (reachable[neighbor] && neighbor != 0) {
            var_name = "x[" + std::to_string(e->from) + "," + std::to_string(e->to) + "]";
            if (model->getVarByName(var_name).get(GRB_DoubleAttr_RC) > UB - LB)  reachable[neighbor] = false;
        }
    }
	//getUpdateMinRes(graph);

    for (const auto e : graph.getNeighbors(vertex, direction)) {
        int neighbor = direction ? e->to : e->from;
        if (reachable[neighbor]) {
            for (int k = 0; k < graph.num_res; k++) {
				//std::cout << resources[k] + e->resources[k] + (direction ? graph.OutList[neighbor][0]->resources[k] : graph.InList[0][neighbor]->resources[k]) << " <= " << graph.res_max[k] << std::endl;
                if (resources[k] + e->resources[k] > graph.res_max[k]) {
                    reachable[neighbor] = false;
                    break;
                }
            }
        }
    }
	std::cout << "Updating y for  nodes" << std::endl;
	for (int i = 1; i < graph.num_nodes; i++) {
		if (reachable[i]) continue;
        for (const int& j : path) {
			if (i == j) continue;
            std::string var_name = "y[" + std::to_string(i) + "]";
            model->getVarByName(var_name).set(GRB_DoubleAttr_UB, 0);
			break;
        }
	}
	model->update();
	model->optimize();
   
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

bool Label::LBImprove(Graph& graph) {
    GRBEnv enV = GRBEnv();
    enV.set(GRB_IntParam_OutputFlag, 0);
    enV.set(GRB_IntParam_LogToConsole, 0);
    GRBModel mdl = GRBModel(enV);
    GRBLinExpr obj = 0, lhs = 0;
    std::map<std::pair<int, int>, GRBVar> w;
    std::map<int, GRBVar> z,z_;

    
    std::map<std::pair<int, int>, double > x;
    std::map<int, double> y;
	std::cout << "starting LB improvement" << std::endl;
	std::string name;
    
    for (int i = 1; i < graph.num_nodes; ++i) {
		name = "[" + std::to_string(i) + "]";
        y[i] = model->getVarByName("y" + name).get(GRB_DoubleAttr_X);
		
        for (const auto e : graph.OutList[i]) {
			if (e->from == 0 || e->to == 0) continue;
            name = "[" + std::to_string(e->from) + "," + std::to_string(e->to) + "]";
            x[{e->from, e->to}] = model->getVarByName("x"+name).get(GRB_DoubleAttr_X);
            w[{e->from, e->to}] = mdl.addVar(0, 1, x[{e->from, e->to}], GRB_BINARY, "y"+name);
			
			obj += x[{e->from, e->to}] * w[{e->from, e->to}];
			if (z.find(e->from) == z.end()) {
                z[e->from] = mdl.addVar(0, 1, 0, GRB_BINARY, "z[" + std::to_string(e->from) + "]");
				z_[e->from] = mdl.addVar(0, 1, 0, GRB_BINARY, "z_[" + std::to_string(e->from) + "]");
			}
			if (z.find(e->to) == z.end()) {
				z[e->to] = mdl.addVar(0, 1, 0, GRB_BINARY, "z[" + std::to_string(e->to) + "]");
				z_[e->to] = mdl.addVar(0, 1, 0, GRB_BINARY, "z_[" + std::to_string(e->to) + "]");
			}
			mdl.addConstr(2 * w[{e->from, e->to}] <= z[e->from] + z[e->to]);
			mdl.addConstr(z[e->from] + z[e->to] <= w[{e->from, e->to}] + 1);
            
        }
		obj -= z[i] * y[i];
        lhs += z_[i];
		mdl.addConstr(z_[i] <= z[i]);
		obj += y[i] * z_[i];
    }
	mdl.addConstr(lhs == 1);
	
	//std::cout << "At least one resource must be violated" << std::endl;
   
    mdl.setObjective(obj, GRB_MAXIMIZE);
    mdl.optimize();
    if (mdl.get(GRB_IntAttr_Status) == GRB_OPTIMAL){
        double zeta = mdl.get(GRB_DoubleAttr_ObjVal);
		std::cout << "zeta: " << zeta << std::endl;
        if (zeta >0.01) {
			
            lhs = 0;
			for (const auto& [key, w_] : w) {
				if (w_.get(GRB_DoubleAttr_X) > 0.5) {
					lhs += model->getVarByName("x[" + std::to_string(key.first) + "," + std::to_string(key.second) + "]");
					std::cout << key.first << " -> " << key.second << std::endl;
				}
			}
			for (const auto& [key, Z] : z) {
				if (Z.get(GRB_DoubleAttr_X) > 0.5  and z_[key].get(GRB_DoubleAttr_X)<0.001) {
					lhs -= model->getVarByName("y[" + std::to_string(key) + "]");
					std::cout << "y[" << key << "]" << std::endl;
				}
			}
			model->addConstr(lhs <= 0);
			model->update();
			model->optimize();
			LB = model->get(GRB_DoubleAttr_ObjVal);
			std::cout << " New LB: " << LB << std::endl;
			return true;
        }
		return false;

    }
    return false;
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
