#pragma once
#include "Graph.h"
#include "Spawner.h"
#include "main.h"
#include "RoutingTable.h"
#include "mpi.h"


class Simulation
{
public:
	Simulation(int numberProcesses, int rank);
	~Simulation();
	void writeResultsCurrentTick();
	
private:
	std::vector<std::vector<int>> splitGraph(int numberProcesses);
	void parallelRouting();
	std::vector<int> Simulation::splitGraphSize(int numberProcesses);
	std::vector<int> Simulation::splitGraphLocation(std::vector<int> buffer);
	void nextTick();
	void initSpawner();

	int Simulation::getEnd(std::queue<int> route);
	void clear(std::queue<int> &q);
	int getMaxCol(std::vector < std::vector<int>> &vals);
	int** setupHMM(std::vector<std::vector<int>> &vals, int N, int M);

	RoutingTable* _routingTable;
	int _world_size;
	int _rank;
	Graph* _graph;
	int _currentTick;

	//All local Vertices and Edges to iterate through in Update
	std::vector<Edge*> localEdges;
	std::vector<Vertex*> localVertices;
	//TODO
	std::vector<Spawner*> localSpawners;

	//All Edges and Vertices mapped to their ID for easier access
	std::map<int, Edge*> localEdgeMap;
	std::map<int, Vertex*> localVertexMap;

	void fillEdgeSpaceSendBuffer();
	void exchangeEgdeFreeSpace();

	//Get Free Space from every relevant edge
	std::map<int, std::map<int,int>> getEdgeFreeSpaceMaps();

	void sendCarInformation();
	void receiveCarInformation();

	//Stores which process has which connections to this process
	//Incoming -> all edges which can receive cars from other processes
	/*
		int -> processID
		vector -> edges related to that process
	*/
	std::map<int, std::vector<int>> incomingConnections;
	std::map<int, std::vector<int>> outgoingConnections;

	void InitEdgeFreeSpaceBuffers();
	void InitConnections();
	void InitLocalVectors();

	//Vectors of vectors of pointers to buffer for information about other processes edges
	/*
		int -> processID
		vector -> buffer for edge space of every edge in ascending order
	*/
	std::map<int, int*> edgeSpaceRecvBuffer;
	std::map<int, int*> edgeSpaceSendBuffer;

	//Vectors that transfer information about cars between processes
	std::map<int, int*> carRecvBuffer;
	std::map<int, int*> carSendBuffer;

	/*
	Request arrays for Waitall
	*/
	MPI_Request *req = new MPI_Request[outgoingConnections.size() + incomingConnections.size()];
	int reqCounter;
};

