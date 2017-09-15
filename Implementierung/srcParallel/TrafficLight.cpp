/*############################################

Author: Rami Aly and Christoph Hueter
Date : 20.09.17
Libraries and Licences:
Boost Library 1.60, MPI Boost Library 1.60, Open Streetmaps and MPI in use.
All used Maps are licensed under the Open Street Maps License.

#############################################*/


#include <iostream>
#include <vector>
#include <algorithm>
#include "TrafficLight.h"
#include <iterator>   

TrafficLight::TrafficLight() {
	phaseDuration = 10;
	timer = rand() % phaseDuration;
}

TrafficLight::TrafficLight(std::vector<std::pair<int, int>> trafficLightMap, int phaseDur, int timerBeginPoint) {

	possiblePhases = trafficLightMap;
	//std::copy(trafficLightMap.begin(), trafficLightMap.end(), std::back_inserter(possiblePhases));
	currentPhaseCounter = rand() % possiblePhases.size();
	//phaseIt = possiblePhases.begin();
	currentPhase = possiblePhases[currentPhaseCounter];

	phaseDuration = phaseDur;
	timer = timerBeginPoint;
}

bool TrafficLight::canCross(int incomingEdgeID) {

	//If one of the edges is contained in the current pair then return true
	//std::cout << "Green: " <<currentPhase.first << " " << currentPhase.second << std::endl;
	if (currentPhase.first == incomingEdgeID || currentPhase.second == incomingEdgeID) {
		return true;
	}
	else {
		//std::cout << "CANNOT CROSS"<<std::endl;
		return false;
	}
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
	currentPhaseCounter++;

	if (currentPhaseCounter >= possiblePhases.size()) {
		currentPhaseCounter = 0;
	}

	currentPhase = possiblePhases[currentPhaseCounter];

	//currentPhase = *phaseIt;
}

std::pair<int, int> TrafficLight::getCurrentPhase()
{
	return currentPhase;
}

std::vector<std::pair<int, int>> TrafficLight::getPossiblePhases()
{
	return possiblePhases;
	//std::vector<std::pair<int, int>> g{ std::begin(possiblePhases), std::end(possiblePhases) };
	//return g;
}
