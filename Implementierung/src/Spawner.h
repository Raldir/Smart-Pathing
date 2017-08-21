#pragma once

#include "Vertex.h"
#include "main.h"
#include "RoutingTable.h"

class Spawner : public Vertex {

public:

	Spawner(int id, float x, float y);

	void SpawnCar();

private:
	RoutingTable routingTable;

	Vertex* nextCarGoal();

	int _spawnRate;
};
