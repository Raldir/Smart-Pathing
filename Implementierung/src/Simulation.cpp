#include "Simulation.h"
#include <iostream>
#include <fstream>

typedef std::vector<Edge*> edgeContainer;

Simulation::Simulation()
{
	_graph = new Graph();
	_routingTable = new RoutingTable(_graph, 6);
	initSpawner();
	for (int i = 0; i < _SIMULATION_TICKS; i++) {
		nextTick();
		writeResultsCurrentTick();
	}

}


Simulation::~Simulation()
{
}

void Simulation::writeResultsCurrentTick()
{
	std::ofstream results;
	edgeContainer edges = _graph->getEdges();
	results.open(_currentTick + ".txt");
	for (edgeContainer::iterator it2 = edges.begin(); it2 != edges.end(); it2++) {
		results << (*it2)->numberOfCars() << '\n';
	}
	results.close();
}


void Simulation::nextTick()
{
	for (Vertex* v : _graph->getVertices()) {
		v->Update();
	}
	for (Edge* ed : _graph->getEdges()) {
		//ed->Update();
		ed->Update(_currentTick);
		//Methode zum Testen
		ed->printCars();
	}
	std::vector<Edge*> remainingEdges(_graph->getEdges());
	for (int i = 0; i < _graph->getEdges().size(); i++) {
		for (edgeContainer::iterator it2 = remainingEdges.begin(); it2 != remainingEdges.end(); it2++) {
			(*it2)->UpdateOverflow();
			(*it2)->printCars();
			if (!(*it2)->hasOverflow()) {
				remainingEdges.erase(it2);
			}
		}
	}
	for (Spawner* v : _graph->getSpawner()) {
		v->Update();
	}
}

void Simulation::initSpawner() {
	std::vector<Spawner*> spawner = _graph->getSpawner();
	std::map<Spawner*, int> vertexPriorities;
	for (std::vector<Spawner*>::iterator it2 = spawner.begin(); it2 != spawner.end(); it2++) {
		vertexPriorities[(*it2)] = rand() % VERTEX_PRIORITY_DIVERGENCE;
	}
	for (std::vector<Spawner*>::iterator it2 = spawner.begin(); it2 != spawner.end(); it2++) {
		(*it2)->linkRoutingTable(_routingTable);
		(*it2)->linkVertexPriorities(vertexPriorities);
	}
}

