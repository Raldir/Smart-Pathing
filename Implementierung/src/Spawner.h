#pragma once
#include "Car.h"
#include "Vertex.h"

class RoutingTable;

class Spawner : public Vertex {

public:

	Spawner(int id, float x, float y);
	void linkRoutingTable(RoutingTable* table);
	void linkVertexPriorities(std::vector<std::pair<Spawner*, int>> vertexPriorities);
	void randomizeSpawnRate();
	void Update(int currentTick);

private:

	int _currentTick;

	void spawnCar();
	Spawner* Spawner::createPartlyRandomizedGoal();

	std::vector<std::pair<Spawner*, int>> _vertexPriorities;
	RoutingTable* _routingTable;
	int _spawnRate;
	int _stepsToNextSpawn;
};
