#pragma once
#include "Edge.h"
#include "Vertex.h"
#include "Spawner.h"
#include "RoutingTable.h"
#include <boost/graph/astar_search.hpp>
#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/random.hpp>
#include <boost/random.hpp>
#include <boost/graph/graphviz.hpp>
#include <time.h>
#include <list>
#include <fstream>
#include <math.h>    // for sqrt


class Graph
{
public:
	Graph();
	~Graph();

private:
	void filterSpawner();
	void calculateRoutingPaths();
	float distance_heuristic2(size_t start, size_t goal);

	RoutingTable* getRoutingTable();
	RoutingTable copyRoutingTable();

	std::vector<Spawner*> _spawner;
	std::vector<Edge*> _edges;
	std:: vector<Vertex*> _vertices;
	std::map<int, Edge*> _edgeMap;
	std::map<int, Vertex*> _vertexMap;
	RoutingTable* _routingTable;
};

