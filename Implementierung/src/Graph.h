#pragma once
#include "Edge.h";
#include "Vertex.h";
#include "RoutingTable.h"
class Graph
{
public:
	Graph();
	~Graph();

private:
	std::vector<Vertex*> filterSpawner();
	void calculateRoutingPaths();

	std::vector<Vertex*> _spawner;
	std::vector<Edge*> _edges;
	std:: vector<Vertex*> _vertices;
	RoutingTable* _routingTable;
};

