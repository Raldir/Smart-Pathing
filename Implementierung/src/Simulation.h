#pragma once
#include "Graph.h"
#include "Spawner.h"
#include "main.h"

class Simulation
{
public:
	Simulation();
	~Simulation();
	void writeResults();



private:
	void nextTick();
};

