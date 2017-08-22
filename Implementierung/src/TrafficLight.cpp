
#include <iostream>
#include <vector>
#include <algorithm>
#include "TrafficLight.h"

TrafficLight::TrafficLight() {
	phaseDuration = 10;
	timer = rand() % phaseDuration;
}

TrafficLight::TrafficLight(std::vector<std::pair<int, int>> trafficLightMap, int phaseDur, int timerBeginPoint) {

	possiblePhases = trafficLightMap;

	phaseIt = possiblePhases.begin();
	currentPhase = *phaseIt;

	phaseDuration = phaseDur;
	timer = timerBeginPoint;
}

bool TrafficLight::canCross(int incomingEdgeID) {

	//If one of the edges is contained in the current pair then return true
	if (currentPhase.first == incomingEdgeID || currentPhase.second == incomingEdgeID) {
		return true;
	}
	return false;
}

/*
void TrafficLight::setConnection(std::pair<int, int> connection)
{
	possiblePhases.push_back(connection);
}

void TrafficLight::setConnections(std::vector<std::pair<int, int>> connections) {
	possiblePhases = connections;
}*/

void TrafficLight::Update() {

	//Increment timer by one tick
	timer++;

	//If timer exceeds phaseDuration set timer to zero and begin new phase
	if (timer >= phaseDuration) {
		timer = 0;
		togglePhase();
	}
}

void TrafficLight::togglePhase() {
	auto oldPhaseIt = phaseIt;
	phaseIt++;

	if (*phaseIt == possiblePhases.at(possiblePhases.size() - 1)) {
		phaseIt = possiblePhases.begin();
	}

	currentPhase = *phaseIt;
}

std::pair<int, int> TrafficLight::getCurrentPhase()
{
	return currentPhase;
}

std::vector<std::pair<int, int>> TrafficLight::getPossiblePhases()
{
	return possiblePhases;
}
