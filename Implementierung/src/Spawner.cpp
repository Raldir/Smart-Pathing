#include "Spawner.h"
#include "Car.h"
#include <stdlib.h>
#include "RoutingTable.h"

Spawner::Spawner(int id, float x, float y) : Vertex(id, x, y) {
	_spawnRate = rand() % BASE_SPAWN_RATE ;
	_stepsToNextSpawn = _spawnRate;
}

void Spawner::linkRoutingTable(RoutingTable * table)
{
	_routingTable = table;
}

void Spawner::linkVertexPriorities(std::vector<std::pair<Spawner*, int>> vertexPriorities)
{
	_vertexPriorities = vertexPriorities;
}

void Spawner::randomizeSpawnRate()
{
	_spawnRate = rand() % BASE_SPAWN_RATE;
}



void Spawner::spawnCar() {

	//TODO Einbauen dass Car sich current tick merkt;
	Spawner* initDestination = createPartlyRandomizedGoal();
	int bestVertexID = _routingTable->calculateBestGoal(_ID, initDestination->getID(), current_timeTable_tick);
	if (outgoingNeighbor(bestVertexID)->isFull()) {
		return;
	}
	Car* car = new Car();
	car->assignRoute(_routingTable->getRoute(_ID, bestVertexID));
	_routingTable->addCosts(_ID, bestVertexID, current_timeTable_tick);
}


Spawner* Spawner::createPartlyRandomizedGoal() {
	int sumElements = 0;
	for (std::vector<std::pair<Spawner*, int>>::iterator it2 = _vertexPriorities.begin(); it2 != _vertexPriorities.end(); it2++) {
		sumElements += (*it2).second;
	}
	int random = rand() % sumElements;
	int paircount = 0;
	int currentCount = 0;
	for (int i = 0; i < random; i++) {
		if (_vertexPriorities[paircount].second == currentCount) {
			paircount++;
			currentCount = 0;
		}
		currentCount++;
	}
	return _vertexPriorities[paircount].first;
}

void Spawner::Update() {
	if (_stepsToNextSpawn == 0) {
		//std::pair<float, float> goal = std::pair<float, float>()
		spawnCar();
		_stepsToNextSpawn = _spawnRate;
	}
	else {
		_stepsToNextSpawn--;
	}
}

