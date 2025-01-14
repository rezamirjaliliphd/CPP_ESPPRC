#include "Edge.h"

Edge::Edge(int f, int t, double c, const std::vector<double>& r)
    : from(f), to(t), cost(c), resources(r) {
}
