#pragma once
#include "Edge.h"
#include "Vertex.h"
#include "RoutingTable.h"


class Graph
{
public:
	Graph();
	~Graph();

private:
	void filterSpawner();
	void calculateRoutingPaths();
	float distance_heuristic(int start, int goal);

	std::vector<Vertex*> _spawner;
	std::vector<Edge*> _edges;
	std:: vector<Vertex*> _vertices;
	std::map<int, Edge*> _edgesMap;
	std::map<int, Vertex*> _verticesMap;
	RoutingTable* _routingTable;
};

