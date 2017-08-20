#pragma once

#include "Vertex.h"
#include "main.h"
#include "RoutingTable.h"

class Spawner : public Vertex {

public:

	Spawner(int id, float x, float y);

	void SpawnCar(Car* car, RoutingTable routingTable);

private:
	Vertex* nextCarGoal();
};
