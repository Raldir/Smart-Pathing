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

	//Stores which process has which connections to this process
	//Incoming -> all edges which can receive cars from other processes
	
	/*
		int -> processID
		vector -> edges related to that process
	*/
	std::map<int, std::vector<int>> incomingConnections;
	std::map<int, std::vector<int>> outgoingConnections;

	//Vectors of vectors of pointers to buffer for information about other processes edges
	/*
		int -> processID
		vector -> buffer for edge space of every edge in ascending order
		//TODO
	*/
	std::map<int, std::vector<int>*> edgeSpaceRecvBuffer;
	std::map<int, std::vector<int>*> edgeSpaceSendBuffer;

	//Stores how much space is left for every edge (only incomingEdges)
	std::map<int, int> edgeSpace;

	void InitVectors();
};

