#pragma once

#include <iostream>
#include <vector>
#include "main.h"

class Vertex;

class TrafficLight {

public:
	TrafficLight();

	TrafficLight(std::vector<std::pair<int, int>> trafficLightMap, int phaseDuration, int timerBeginTime);

	//void removeConnection(std::pair<int, int> connection);

	bool canCross(int incomingEdgeID);

	//void setConnection(std::pair<int, int> connection);
	//void setConnections(std::vector<std::pair<int, int>> connections);

	void Update();

	std::pair<int, int> getCurrentPhase();

	std::vector<std::pair<int, int>> getPossiblePhases(); //Irrelevant, weil kein "Ripple-Update" mehr

private:
	//Iteriert durch Phase
	void togglePhase();

	std::vector<std::pair<int,int>> possiblePhases;
	std::vector<std::pair<int,int>>::iterator phaseIt;

	std::pair<int, int> currentPhase;

	//Misst die Zeit die seit dem letzten Phasenwechsel vergangen ist
	int timer;
	
	//Dauer jeder Phase
	int phaseDuration;
};
