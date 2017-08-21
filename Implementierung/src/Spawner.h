#pragma once
#include "Car.h"
#include "Vertex.h"

class Vertex;
class main;
class RoutingTable;

class Spawner : public Vertex {

public:

	Spawner(int id, float x, float y);

	void SpawnCar();

private:
	//RoutingTable routingTable;

	Vertex* nextCarGoal();

	//void spawnCar(Car* car, RoutingTable routingTable);

	int _spawnRate;
};
