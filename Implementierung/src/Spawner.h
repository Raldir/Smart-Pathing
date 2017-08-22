#pragma once
#include "Car.h"
#include "Vertex.h"

class RoutingTable;

class Spawner : public Vertex {

public:

	Spawner(int id, float x, float y);
	void linkRoutingTable(RoutingTable* table);
	void linkVertexPriorities(std::map<Spawner*, int> vertexPriorities);
	void randomizeSpawnRate();
	void update();

private:

	void spawnCar();
	void Spawner::calculatePossibleGoals(int goal);

	std::map<Spawner*, int> _vertexPriorities;
	RoutingTable* _routingTable;
	int _spawnRate;
	int _stepsToNextSpawn;
};
