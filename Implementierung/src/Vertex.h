#pragma once

#include <iostream>
#include <utility>
#include "main.h"
#include "ObserverPattern.h"

class Edge;
class Car;

class Vertex : public SubjectEdge {

public:
	Vertex(int id, float x, float y);

	//TrafficLight* trafficLight;

	void transferCar(Car * car, std::pair<Edge*, Edge*> edges);

	void addIncomingEdges(Edge *edge);
	void addOutgoingEdges(Edge *edge);

	void printEdges();

	int getId();

private:

	std::vector<Edge*> incomingEdges;
	std::vector<Edge*> outgoingEdges;

	int _ID;
	float _X;
	float _Y;

};