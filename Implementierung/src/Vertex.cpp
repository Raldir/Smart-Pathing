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

void Vertex::transferCar(Edge* edge)
{
	//Get cat in front
	Car* car = edge->getFrontCar();

	//Remove car from edge
	edge->popCar();

	//Removes this point as destination to reveal next point
	car->popCurrentVertex();
	car->getCurrentVertex();

	Vertex* nextVertex;

	for (Edge* edge : outgoingEdges) {
		int ID = edge->getObserver()->getID();

		if (ID == car->getCurrentVertex()->getID()) {

		}
	}
}

bool Vertex::canTransit(Car* car) {
	//TODO Implement when Traffic Light is ready

	return NULL;
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
		std::cout << "Incoming Edge: "<< e->getID() << std::endl;
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
