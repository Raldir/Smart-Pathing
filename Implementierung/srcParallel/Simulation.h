#pragma once
#include "Graph.h"
#include "Spawner.h"
#include "main.h"
#include "RoutingTable.h"


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
};

