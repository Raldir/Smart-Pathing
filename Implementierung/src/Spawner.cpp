#include "Spawner.h"
#include "Car.h"
#include <stdlib.h>
#include "RoutingTable.h"
#include "main.h"

Spawner::Spawner(int id, float x, float y) : Vertex(id, x, y) {
	randomizeSpawnRate();
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

void Spawner::spawnCar(int currentTick) {

	//TODO Einbauen dass Car sich current tick merkt;
	Spawner* initDestination = createPartlyRandomizedGoal();
	int bestVertexID = _routingTable->calculateBestGoal(_ID, initDestination->getID(), currentTick);
	std::queue<int> route =_routingTable->getRoute(_ID, bestVertexID);
	route.pop();
	Edge* edge = this->Vertex::outgoingNeighbor(route.front());
	std::cout << this->getOutgoingEdges().size();
	if (edge->isFull()) {
		std::cout << "Edge is Full, no new car"<<std::endl;
		return;
	}
	std::cout<<"Create Car" << std::endl;
	Car* car = new Car(currentTick);
	car->assignRoute(route);
	this->giveCar(edge, car);
	_routingTable->addCosts(_ID, bestVertexID, currentTick);
	
}


Spawner* Spawner::createPartlyRandomizedGoal() {
	int sumElements = 0;
	for (std::vector<std::pair<Spawner*, int>>::iterator it2 = _vertexPriorities.begin(); it2 != _vertexPriorities.end(); it2++) {
		sumElements += (*it2).second;
	}
	std::cout << "sumElements:" << sumElements<<'\n';
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
	if (_vertexPriorities[paircount].first->getID() == _ID) return createPartlyRandomizedGoal();
	return _vertexPriorities[paircount].first;
}

void Spawner::Update(int cT) {
	if (_stepsToNextSpawn == 0) {
		spawnCar(cT);
		_stepsToNextSpawn = _spawnRate;
	}
	else {
		_stepsToNextSpawn--;
	}
}

void Spawner::setIncomingEdges(std::vector<Edge*> edges)
{
	incomingEdges = edges;
}

void Spawner::setOutigoingEdges(std::vector<Edge*> edges)
{
	outgoingEdges = edges;
}

