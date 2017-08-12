#pragma once

#include <iostream>
#include <utility>
#include "main.h"
#include "ObserverPattern.h"

class Edge;
class Car;

class Vertex : public SubjectEdge {

public:
	Vertex(int id);

	//TrafficLight* trafficLight;

	/*
		Makes the actual transition of the car
		ONLY use when certain that transtition will be possible
	*/
	void transferCar(Edge* edge);

	//Checks wheter TrafficLight allow transit and wheter or not the next edge is full
	bool canTransit(Edge* nextEdge);

	void addIncomingEdges(Edge *edge);
	void addOutgoingEdges(Edge *edge);

	void printEdges();

	int getID();

private:

	std::vector<Edge*> incomingEdges;
	std::vector<Edge*> outgoingEdges;

	int _ID;
};