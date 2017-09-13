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
	std::vector<int> splitGraphSize(int numberProcesses);
	std::vector<int> splitGraphLocation(std::vector<int> buffer);
	void nextTick();
	void initSpawner();
	void executeGatherRouting(std::vector<std::vector<int>> matrix,
		int* displays, int*splitting, int cols, int totalRows);
	int getEnd(std::queue<int> route);
	void clear(std::queue<int> &q);
	int getMaxCol(std::vector < std::vector<int>> &vals);
	int** setupHMM(std::vector<std::vector<int>> &vals, int N, int M);

	RoutingTable* _routingTable;
	int _world_size;
	int _rank;
	Graph* _graph;
	int _currentTick;

	//For Parallelisation
	int* receive_displs;
	int* receive_elementC;
	int* receive_buf;
	void fillEdgeSpaceSendBuffer();
	void exchangeEgdeFreeSpace();

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

	void InitEdgeFreeSpaceBuffers();
	void InitConnections(std::map<int,int> vertexVector);
};

