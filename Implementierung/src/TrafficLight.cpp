
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
	currPhaseN = 0;
	//phaseIt = possiblePhases.begin();
	currentPhase = possiblePhases[currPhaseN];

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
	/*auto oldPhaseIt = phaseIt;*/
	auto oldPhaseIt = currPhaseN;
	//std::advance(phaseIt, 1);
	//std::cout << " " << possiblePhases.size() << possiblePhases[0].first;
	//phaseIt++;
	currPhaseN++;
	/*if (phaseIt != possiblePhases.end()) {*/
	if (currPhaseN >= possiblePhases.size() - 1){
		currPhaseN = 0;
	}

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
