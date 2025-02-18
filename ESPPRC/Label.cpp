#include "Label.h"
#include <iostream>
#include <cmath>
#include <algorithm>

Label::Label(Graph& graph)
    : vertex(0), path({ 0 }), cost(0),
    resources(graph.num_res, 0),
    reachable(graph.num_nodes, true), LB(0), id(0), rc(graph.num_edges, 0) { // Farzane: initialized "edges()"
    status = LabelStatus::OPEN;
    reachable[0] = false;
    id = 0;
    //std::cout << "Copying root model" << std::endl;
    model = std::make_shared<GRBModel>(*graph.model);
    //std::cout << "Copying separation model" << std::endl;
    //sep_model = std::make_shared<GRBModel>(*graph.sep_model);
    //std::cout << "Updating root model" << std::endl;
    model->update();
    model->optimize();
    /*std::cout << " updating separation model" << std::endl;
    sep_model->update();
    std::cout << " updated separation model" << std::endl;
    sep_model->optimize();
    std::cout << " optimized separation model" << std::endl;*/
    LB = graph.model->get(GRB_DoubleAttr_ObjVal);
    std::cout << "LB: " << LB << std::endl;
    //LBImprove(graph);
    UpdateReachable(graph, 0);
}

// Farzane: passed pointer of MIP to the label
Label::Label(const Label& parent, Graph& graph, const Edge* edge, const double UB)
    : path(parent.path), cost(parent.cost),
    resources(parent.resources), reachable(parent.reachable) {
    vertex = edge->to;
    cost = parent.cost + edge->cost;
    model = std::make_shared<GRBModel>(*parent.model);
    model->getVar(graph.x_index[{edge->to,edge->from}]).set(GRB_DoubleAttr_LB, 1);
    model->update();
    model->optimize();
    /*sep_model = std::make_shared<GRBModel>(*parent.sep_model);
    sep_model->update();
    sep_model->optimize();*/
    LB = model->get(GRB_DoubleAttr_ObjVal);
    path.push_back(vertex);

    reachable[vertex] = false;
    reachable[0] = false;
    for (size_t i = 0; i < resources.size(); ++i) {
        resources[i] += edge->resources[i];
    }
    //LBImprove(graph);
    UpdateReachable(graph, UB);

    /*LB = cost;
    for (int i = 0; i < graph.num_nodes; i++) {
        if (reachable[i]&&graph.max_value[i]<0) {
            LB += graph.max_value[i];
        }
    }*/


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
        if ((i == 0) || (i == vertex) || reachable[i]) {
            for (const auto e : graph.OutList[i]) {
                if (reachable[e->to]) {
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
        //ind = ;
        //ind = ind && (model->getVar(graph.y_index[i]).get(GRB_DoubleAttr_RC) > UB - LB);
        if (reachable[i] && model->getVar(graph.u_index[i]).get(GRB_DoubleAttr_RC) > UB - LB) {
            reachable[i] = false;
            model->getVar(graph.u_index[i]).set(GRB_DoubleAttr_UB, 0);
            model->update();
            model->optimize();
            LB = model->get(GRB_DoubleAttr_ObjVal);
        }
    }
    
    for (const auto e : graph.OutList[vertex]) {
        int neighbor =  e->to;
        if (reachable[neighbor] && neighbor != 0) {
            
            if (model->getVar(graph.x_index[{e->from, e->to}]).get(GRB_DoubleAttr_RC) > UB - LB) reachable[neighbor] = false ;
        }
    }
    //getUpdateMinRes(graph);

    for (const auto e : graph.OutList[vertex]) {
        int neighbor = e->to;
        if (reachable[neighbor]) {
            for (int k = 0; k < graph.num_res; k++) {
                if (resources[k] + e->resources[k] > graph.res_max[k]) {
                    reachable[neighbor] = false;
                    break;
                }
            }
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

bool Label::isInPath(int node) const {
    for (const int i : path) {
        if (i == node) return true;
    }
    return false;
}

void Label::display() const {
    std::cout << "=========================\n";
    std::cout << "Path: ";
    for (int node : path) {
        std::cout << node << " ";
    }
    std::cout << ", LB: " << LB << " Cost: " << cost << ", Resources : [";
    for (size_t i = 0; i < resources.size(); ++i) {
        std::cout << resources[i] << (i + 1 < resources.size() ? ", " : "");
    }
    std::cout << "] Reachables: {";
    for (size_t i = 0; i < reachable.size(); ++i) {
        if (reachable[i]) {
            std::cout << i << (i + 1 < reachable.size() ? ", " : "");
        }
    }
    std::cout << "} ";
    std::cout << "Status: ";
    switch (status) {
    case LabelStatus::NEW_OPEN:    std::cout << "NEW_OPEN "; break;
    case LabelStatus::NEW_CLOSED:  std::cout << "NEW_CLOSED "; break;
    case LabelStatus::OPEN:        std::cout << "OPEN "; break;
    case LabelStatus::CLOSED:      std::cout << "CLOSED "; break;
    case LabelStatus::DOMINATED:   std::cout << "DOMINATED "; break;
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

bool Label::isConcatenable(const Label& label, const std::vector<double>& r_max) const {
    for (size_t i = 0; i < resources.size(); ++i) {
        if (resources[i] + label.resources[i] > r_max[i]) {
            return false;
        }
    }

    for (const int& i : path) {
        if (i == vertex || i == 0) continue;
        if (label.isInPath(i)) return false;
    }
    return true;
}

void Label::LBImprove(Graph& graph) {
    while (true) {
        GRBLinExpr lhs = 0;
        double y_val = 0, x_val = 0;
        //std::cout << "starting LB improvement" << std::endl;
        std::string name;

        for (int i = 1; i < graph.num_nodes; ++i) {
            name = "[" + std::to_string(i) + "]";
            //std::cout << "changing coefficient of z[" << i << "]" << std::endl;
            y_val = model->getVarByName("y" + name).get(GRB_DoubleAttr_X);
            sep_model->getVarByName("z" + name).set(GRB_DoubleAttr_Obj, -y_val);
            //std::cout << "changing coefficient of z_[" << i << "]" << std::endl;
            sep_model->getVarByName("z_" + name).set(GRB_DoubleAttr_Obj, y_val);
            for (const auto e : graph.OutList[i]) {
                if (e->from == 0 || e->to == 0) continue;
                name = "[" + std::to_string(e->from) + "," + std::to_string(e->to) + "]";
                x_val = model->getVarByName("x" + name).get(GRB_DoubleAttr_X);
                sep_model->getVarByName("w" + name).set(GRB_DoubleAttr_Obj, x_val);
            }
        }
        sep_model->update();
        sep_model->optimize();
        if (sep_model->get(GRB_IntAttr_Status) == GRB_OPTIMAL) {
            double zeta = sep_model->get(GRB_DoubleAttr_ObjVal);
            if (zeta > 0.01) {
                lhs = 0;
                for (int i = 1; i < graph.num_nodes; ++i) {
                    name = "[" + std::to_string(i) + "]";
                    if (sep_model->getVarByName("z" + name).get(GRB_DoubleAttr_X) > 0.5 && sep_model->getVarByName("z_" + name).get(GRB_DoubleAttr_X) < 0.01) {
                        lhs -= model->getVarByName("y" + name);
                    }

                    for (const auto e : graph.OutList[i]) {
                        if (e->from == 0 || e->to == 0) continue;
                        name = "[" + std::to_string(e->from) + "," + std::to_string(e->to) + "]";
                        if (sep_model->getVarByName("w" + name).get(GRB_DoubleAttr_X) > 0.5) {
                            lhs += model->getVarByName("x" + name);
                        }
                    }
                }
                model->addConstr(lhs <= 0);
                model->update();
                model->optimize();
                LB = model->get(GRB_DoubleAttr_ObjVal);
                std::cout << " New LB: " << LB << std::endl;

            }
            else {
                break;
            }
        }
        else {
            break;
        }

    }
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