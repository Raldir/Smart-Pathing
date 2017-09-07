#pragma once
#include "Graph.h"
#include "Spawner.h"
#include "main.h"
#include "RoutingTable.h"
#include "mpi.h"


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

	void fillEdgeSpaceSendBuffer();

	void exchangeEgdeFreeSpace();

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

	//Vectors that transfer information about cars between processes
	std::map<int, std::vector<int>*> carRecvBuffer;
	std::map<int, std::vector<int>*> carSendBuffer;

	/*
	Request arrays for Waitall
	*/
	MPI_Request *recvReq = new MPI_Request[incomingConnections.size()];
	MPI_Request *sendReq = new MPI_Request[outgoingConnections.size()];

	void InitVectors();
};

