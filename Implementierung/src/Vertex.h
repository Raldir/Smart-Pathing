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

		void transferCar(Car * car, std::pair<Edge*, Edge*> edges);

		void addIncomingEdges(Edge *edge);
		void addOutgoingEdges(Edge *edge);

		void printEdges();

		int getId();

private:

		std::vector<Edge*> incomingEdges;
		//Edge* outgoingEdges[x];

		int _ID;
		
};