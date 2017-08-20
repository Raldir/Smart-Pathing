#include "Simulation.h"

typedef std::vector<Spawner*> spawnerContainer;

Simulation::Simulation()
{
	Graph* g = new Graph();
	initSpawner(g->getSpawner());
	for (int i = 0; i < _SIMULATION_TICKS; i++) {
		nextTick();
		writeResults();
	}

}


Simulation::~Simulation()
{
}

void Simulation::writeResults()
{
}

void Simulation::initSpawner(std::vector<Spawner*> spawner)
{
	for (spawnerContainer::iterator it = spawner.begin(); it != spawner.end(); it++) {

	}
}

void Simulation::nextTick()
{

}

