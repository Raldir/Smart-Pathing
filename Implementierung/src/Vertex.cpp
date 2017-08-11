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

void Vertex::transferCar(Car * car, std::pair<Edge*, Edge*> edges)
{
	//TODO Transfers car
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

/*void Vertex::update(Edge *edge, Car *car) {

}*/

void Vertex::printEdges() {
	for (Edge* e : incomingEdges) {
		std::cout << e->_ID;
	}
}

int Vertex::getId() {
	return _ID;
}

void TakeCar(Edge* edge, Car* car) {

}
