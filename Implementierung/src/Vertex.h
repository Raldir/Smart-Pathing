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

	//Makes the actual transition of the car
	void transferCar(Edge* edge);

	bool canTransit(Car* car);

	void addIncomingEdges(Edge *edge);
	void addOutgoingEdges(Edge *edge);

	void printEdges();

	int getID();

private:

	std::vector<Edge*> incomingEdges;
	std::vector<Edge*> outgoingEdges;

	int _ID;
<<<<<<< HEAD
	float _X;
	float _Y;

=======
>>>>>>> c9c87935356490dfbe41c0eba73a7914f774cf98
};