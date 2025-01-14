#ifndef BOOSTGRAPHCONVERTER_H
#define BOOSTGRAPHCONVERTER_H

#include "Graph.h"
#include "Edge.h"
#include "Label.h"
#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/r_c_shortest_paths.hpp>
#include <vector>

using namespace boost;

// Vertex and Edge Property Definitions
struct SPPRC_Example_Graph_Vert_Prop {
    int num;
    double eat;
    double lat;
    SPPRC_Example_Graph_Vert_Prop(int n = 0, double e = 0, double l = 0) : num(n), eat(e), lat(l) {}
};

struct SPPRC_Example_Graph_Arc_Prop {
    int num;
    double cost;
    double time;
    SPPRC_Example_Graph_Arc_Prop(int n = 0, double c = 0.0, double t = 0.0) : num(n), cost(c), time(t) {}
};

using SPPRC_Example_Graph = adjacency_list<vecS, vecS, directedS, SPPRC_Example_Graph_Vert_Prop, SPPRC_Example_Graph_Arc_Prop>;

// Function Declaration
int rcBoost(const Graph& graph, const std::vector<double>& r_max,const Label& label);

#endif // BOOSTGRAPHCONVERTER_H
