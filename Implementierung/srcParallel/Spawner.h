#pragma once
#include "Vertex.h"

class RoutingTable;

class Spawner : public Vertex {

public:
	Spawner(int id, float x, float y);

	void linkRoutingTable(RoutingTable* table);
	void linkVertexPriorities(std::vector<std::pair<Spawner*, int>> vertexPriorities);

	//create a randomized Spawnrate
	void randomizeSpawnRate();

	//Update the Spawner, check if it needs to Spawn car
	void Update(int currentTick);

	void setIncomingEdges(std::vector<Edge*> edges);
	void setOutigoingEdges(std::vector<Edge*> edges);

private:
	//Spawns a car
	void spawnCar(int currentTick);

	//returns a goal obtained through a probability vector
	Spawner* createPartlyRandomizedGoal();


	std::vector<std::pair<Spawner*, int>> _vertexPriorities;
	RoutingTable* _routingTable;
	int _spawnRate;
	int _stepsToNextSpawn;
};
