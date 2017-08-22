#include "Spawner.h"
#include <stdlib.h>


Spawner::Spawner(int id, float x, float y) : Vertex(id, x, y) {
	_spawnRate = rand() % BASE_SPAWN_RATE ;
	_stepsToNextSpawn = _spawnRate;
}

void Spawner::linkRoutingTable(RoutingTable * table)
{
	_routingTable = table;
}

void Spawner::linkVertexPriorities(std::map<Spawner*, int> vertexPriorities)
{
	_vertexPriorities = vertexPriorities;
}

void Spawner::randomizeSpawnRate()
{
	_spawnRate = rand() % BASE_SPAWN_RATE;
}



void Spawner::spawnCar() {

	Car* car = new Car();
}


void Spawner::calculatePossibleGoals(int goal) {

}

void Spawner::update() {
	if (_stepsToNextSpawn == 0) {
		//std::pair<float, float> goal = std::pair<float, float>()
		//spawnCar();
		_stepsToNextSpawn = _spawnRate;
	}
	else {
		_stepsToNextSpawn--;
	}
}

