#ifndef EDGE_H
#define EDGE_H

#include <vector>

class Edge {
public:
    int from;
	int to;
    double cost;
    std::vector<double> resources;
    Edge(int f, int t, double c, const std::vector<double>& r);
};


#endif // EDGE_H