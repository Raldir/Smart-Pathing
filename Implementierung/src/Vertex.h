#pragma once

#include <iostream>
#include <utility>
#include "main.h"
#include "ObserverPattern.h"

class Edge;
class Car;

class Vertex : public ObserverVertex {

public:
	Vertex(int id, float x, float y);

	//TrafficLight* trafficLight;

	/*
		Makes the actual transition of the car
		ONLY use when certain that transtition will be possible
	*/
	virtual void transferCar(Edge* edge) override;

	//Hilfefunktion
	Car* takeCar(Edge* edge);

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
	float _X;
	float _Y;
};