#pragma once
#include "Graph.h"
#include "Spawner.h"
#include "main.h"
#include "RoutingTable.h"

class Simulation
{
public:

	Simulation();
	~Simulation();
	void writeResultsCurrentTick();

private:

	RoutingTable* _routingTable;
	Graph* _graph;
	int _currentTick;

	void nextTick();
	void initSpawner();

	void InitVectors();

	std::map<int, Vertex*> _vertexMap;
	
	std::vector<Vertex*> _vertices;
	std::vector<Edge*> _edges;
	std::vector<Spawner*> _spawners;

};

