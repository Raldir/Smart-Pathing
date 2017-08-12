#include <iostream>
#include <utility>
#include "Vertex.h"
#include "ObserverPattern.h"
#include "Car.h"
#include "Edge.h"

/**
*/
Vertex::Vertex(int id, float x, float y) : _ID(id), _X(x), _Y(y) {

}

/*
###### CARSTUFF
*/

/*
	Takes stored vertex of car and searches for the edge connecting to vertex then
	transfers car onto this edge
*/
void Vertex::transferCar(Edge* edge)
{
	//Get cat in front
	Car* car = edge->getFrontCar();

	//Remove car from edge
	edge->popCar();

	//Removes this point as destination to reveal next point
	car->popCurrentVertex();

	Edge* nextEdge;
	bool nextEdgeFound = false;

	for (Edge* edge : outgoingEdges) {

		//Look if one of the edges has Vertex with matching ID from car
		if (edge->getObserver()->getID() == car->getCurrentVertex()->getID()) {
			nextEdge = edge;
			nextEdgeFound = true;
			break;
		}
		
	}

	if (nextEdgeFound) {
		//Begin transfer


	} else {
		std::cout << "No edge found leading to next vertex!" << std::endl;
	}
}

bool Vertex::canTransit(Edge* nextEdge) {
	//TODO Implement when Traffic Light is ready

	return !(nextEdge->isFull());
}

/*
###### EDGESTUFF
*/

//Adds pointer of incoming edge to Vector
void Vertex::addIncomingEdges(Edge* edge) {
	incomingEdges.push_back(edge);
}

//Adds Pointer of outgoing edge to Vector
void Vertex::addOutgoingEdges(Edge* edge) {
	outgoingEdges.push_back(edge);
}

void Vertex::printEdges() {
	for (Edge* e : incomingEdges) {
		std::cout << "Incoming Edge: " << e->getID() << std::endl;
	}

	for (Edge* e : outgoingEdges) {
		std::cout << "Outgoing Edge: " << e->getID() << std::endl;
	}
}

int Vertex::getID() {
	return _ID;
}

void TakeCar(Edge* edge, Car* car) {

}
