#include "Edge.h"

Edge::Edge(int t, int h, double c, const std::vector<double>& r)
    : tail(t), head(h), cost(c), resources(r) {
}
