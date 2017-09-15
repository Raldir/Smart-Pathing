/*############################################

Author: Rami Aly and Christoph Hueter
Date : 20.09.17
Libraries and Licences:
Boost Library 1.60, MPI Boost Library 1.60, Open Streetmaps and MPI in use.
All used Maps are licensed under the Open Street Maps License.

#############################################*/

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
	_spawnRate = rand() % BASE_SPAWN_RATE + 1;
}

void Spawner::spawnCar(int currentTick) {

	//Create a Goal for the Car
	Spawner* initDestination = createPartlyRandomizedGoal();
	//Calculates if alternative Route in neighborhood is better
	int bestVertexID = _routingTable->calculateBestGoal(_ID, initDestination->getID(), currentTick);
	//Catch if no path is found
	if (bestVertexID == -1) {
		std::cout << "No path to current goal" << std::endl;
		return;
	}
	std::queue<int> route =_routingTable->getRoute(_ID, bestVertexID);
	route.pop();
	Edge* edge = this->Vertex::outgoingNeighbor(route.front());
	//Check if it is possible to spawn car on edge or if it is full
	if (edge->isFull()) {
		return;
	}
	std::cout<<"Create Car on Spawner " << this->getID() <<"and " << edge->getID()<< std::endl;
	//Creates new car and assign the route and current Tick, as well on the Edge
	Car* car = new Car(currentTick);
	car->assignRoute(route);
	this->giveCar(edge, car);
	//Add the ocsts to the Timetables on the paths
	_routingTable->addCosts(_ID, bestVertexID, currentTick);
	
}


Spawner* Spawner::createPartlyRandomizedGoal() {
	int sumElements = 0;
	for (std::vector<std::pair<Spawner*, int>>::iterator it2 = _vertexPriorities.begin(); it2 != _vertexPriorities.end(); it2++) {
		sumElements += (*it2).second;
	}
	//The selected goal depends on the vertexPriorities, higher value increases the chance that this method returns this spawner
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
	//Tries to spawn a car when step reach 0 and sets new countdown
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

