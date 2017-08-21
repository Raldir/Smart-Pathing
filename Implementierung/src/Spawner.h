#pragma once
#include "Car.h"
#include "Vertex.h"

class Vertex;
class main;
class RoutingTable;

class Spawner : public Vertex {

public:

	Spawner(int id, float x, float y);
	//void spawnCar(Car* car, RoutingTable routingTable);

private:
	int _spawnRate;
};
