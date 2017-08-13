#pragma once

#include <iostream>
#include <utility>
#include "main.h"
#include "ObserverPattern.h"

class Edge;
class Car;

class Vertex{

public:
	Vertex(int id, float x, float y);

	//TrafficLight* trafficLight;

	//Makes the actual transition of the car
	void transferCar(Edge* edge);

	bool canTransit(Car* car);

	void addIncomingEdges(Edge *edge);
	void addOutgoingEdges(Edge *edge);

	void printEdges();

	int getID();
	float getX();
	float getY();

private:

	std::vector<Edge*> incomingEdges;
	std::vector<Edge*> outgoingEdges;

	int _ID;
	float _X;
	float _Y;
};