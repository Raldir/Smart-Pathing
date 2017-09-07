#pragma once
#include "Graph.h"
#include "Spawner.h"
#include "main.h"
#include "RoutingTable.h"


class Simulation
{
public:
	Simulation(int numberProcesses);
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
};

