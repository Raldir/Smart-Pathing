#pragma once

#include <iostream>
#include <vector>
#include "main.h"
#include "Vertex.h"


class TrafficLight {

public:
	TrafficLight();

	TrafficLight(std::vector<std::pair<int, int>> trafficLightMap, int phaseDuration) {};

	//void removeConnection(std::pair<int, int> connection);

	bool canCross(int incomingEdgeID);

	void setConnection(std::pair<int, int> connection);
	void setConnections(std::vector<std::pair<int, int>> connections);

	void Update();

	void togglePhase();

	std::pair<int, int> getCurrentPhase();

	std::vector<std::pair<int, int>> getPossiblePhase();

private:
	std::vector<std::pair<int,int>> possiblePhases;
	std::vector<std::pair<int,int>>::iterator phaseIt;

	std::pair<int, int> currentPhase;

	int timer;
	int phaseDuration;
};
