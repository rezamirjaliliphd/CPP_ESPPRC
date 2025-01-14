// MIP.h
#ifndef MIP_H
#define MIP_H

#include <gurobi_c++.h>
#include "Graph.h"

void solveMIP(Graph& graph, bool LP_relaxation);

#endif // MIP_H
