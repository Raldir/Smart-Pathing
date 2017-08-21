#include "Simulation.h"

typedef std::vector<Spawner*> spawnerContainer;

Simulation::Simulation()
{
	Graph* g = new Graph();
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


void Simulation::nextTick()
{

}

