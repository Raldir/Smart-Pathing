#pragma once
#include "Graph.h"
#include "Spawner.h"
#include "main.h"
#include "RoutingTable.h"
#include "mpi.h"


class Simulation
{
public:
	//Creates an Simulation instance with the number of processes and the rank of the process calling this instance
	Simulation(int numberProcesses, int rank);

	/*
	Write down the load of each road of the graph into a file of the current tick
	*/
	void writeResultsCurrentTick();
	
private:

	/*
	Splits the Spawners of the graph in evenly splitted parts.
	*/
	std::vector<std::vector<int>> splitGraph(int numberProcesses);
	/*
	Complete Algorithm for calculating the routing tables distributed, so that the routes will be sequentially
	calculated on multiple nodes and exchanged with each other to form a complete routing table.
	*/
	void parallelRouting();

	//Splits the Spawners of the graph in evenly splitted parts and returns the size of the splititng
	std::vector<int> splitGraphSize(int numberProcesses);


	//Calculates the position of the spawnerstructure in which it should be splitted
	std::vector<int> splitGraphLocation(std::vector<int> buffer);
	void nextTick();
	void initSpawner();

	//Parallel Collecting(Gatherv) algorithm that uses a two dimensional Sendbuffer
	void executeGatherRouting(std::vector<std::vector<int>> matrix,
		int* displays, int*splitting, int cols, int totalRows);
	int getEnd(std::queue<int> route);

	//Clears a Queue
	void clear(std::queue<int> &q);

	//Converts a twodimensional vector into a continiously spaced array and return the pointer to it.
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

	//For Parallelisation
	int* receive_displs;
	int* receive_elementC;
	int* receive_buf;

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
	//Init outgoing and incoming connections
	void InitConnections();
	//Init localEdge and vertices
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
	MPI_Request *req;
	int reqCounter;
};

