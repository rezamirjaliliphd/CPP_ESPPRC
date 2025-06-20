#ifndef EDGE_H
#define EDGE_H

#include <vector>

class Edge {
public:
    int tail;
	int head;
    double cost;
    std::vector<double> resources;
    Edge(int t, int h, double c, const std::vector<double>& r);
};


#endif // EDGE_H