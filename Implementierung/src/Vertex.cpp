#include <iostream>
#include <utility>
#include "Vertex.h"
#include "ObserverPattern.h"
#include "Car.h"
#include "Edge.h"

/**
*/
Vertex::Vertex(int id, float x, float y) : _X(x), _Y(y), _ID(id) {

}

/*
###### CARSTUFF
*/

/*
	Takes stored vertex of car and searches for the edge connecting to vertex then
	transfers car onto this edge
*/
void Vertex::transferCar(int incomingEdgeID)
{
	Car* car = getEdgeFromID(incomingEdgeID)->getFrontCar();

	Edge* nextEdge = NULL;
	bool nextEdgeFound = false;

	for (std::pair<int, Edge*> e : outgoingEdges) {

		//Look if one of the edges has Vertex with matching ID from car
		if (e.second->getVertices().second->getID() == car->getCurrentVertexID()) {
			nextEdge = e.second;
			nextEdgeFound = true;
			break;
		}
	}

	if (nextEdgeFound) {
		if (canTransit(nextEdge->getID())) {

			Car* car = takeCar(incomingEdgeID);

			giveCar(nextEdge, car);
			//Removes the next point as destination
			car->popCurrentVertex();
		}
		else {
			//TODO
		}
	}
	else {
		//TODO No next edge found?
		std::cout << "No edge found leading to next vertex!" << std::endl;
	}
}

//Hilfsfunktion falls sich was an der Struktur verändert
Car* Vertex::takeCar(int incomingEdgeID) {
	//Get cat in front
	Car* car = getEdgeFromID(incomingEdgeID)->getFrontCar();

	//Remove car from edge
	getEdgeFromID(incomingEdgeID)->popCar();

	return car;
}

void Vertex::giveCar(Edge* outgoingEdge, Car* car)
{
	outgoingEdge->pushCar(car);
}

bool Vertex::canTransit(int outgoingEdgeID) {
	//TODO Implement when Traffic Light is ready

	return isEdgeFullMap[outgoingEdgeID];
	//return !(getEdgeFromID(outgoingEdgeID)->isFull());
}

/*
###### EDGESTUFF
*/

//Adds pointer of incoming edge to Vector
void Vertex::addIncomingEdges(Edge* edge) {
	incomingEdges[edge->getID()] = edge;
}

//Adds Pointer of outgoing edge to Vector
void Vertex::addOutgoingEdges(Edge* edge) {
	outgoingEdges[edge->getID()] = edge;
}

std::vector<Edge*> Vertex::getIncomingEdges()
{
	std::vector<Edge*> v;

	for (std::pair<int, Edge*> e : incomingEdges) {
		v.push_back(e.second);
	}

	return v;
}

std::vector<Edge*> Vertex::getOutgoingEdges()
{
	std::vector<Edge*> v;

	for (std::pair<int, Edge*> e : outgoingEdges) {
		v.push_back(e.second);
	}

	return v;
}

Edge* Vertex::getEdgeFromID(int edgeID) {

	//Searches through the pairs in the map and matches the IDs
	for (std::pair<int, Edge*> p : incomingEdges) {
		if (p.first == edgeID) {
			return p.second;
		}
	}

	//Searches through the pairs in the map and matches the IDs
	for (std::pair<int, Edge*> p : incomingEdges) {
		if (p.first == edgeID) {
			return p.second;
		}
	}

	std::cout << "No Edge with matching id has been found" << std::endl;
	return NULL;
}

void Vertex::setEdgeIsFull(int outgoingEdgeID, bool isFull)
{
	isEdgeFullMap[outgoingEdgeID] = isFull;
}

void Vertex::printEdges() {
	std::cout << "Vertex:" << _ID << " ";
	for (std::pair<int, Edge*> e : incomingEdges) {
		std::cout << "Incoming Edge: " << e.second->getID() << std::endl;
	}

	for (std::pair<int, Edge*> e : outgoingEdges) {
		std::cout << "Vertex:" << _ID << " ";
		std::cout << "Outgoing Edge: " << e.second->getID() << std::endl;
	}
}

int Vertex::getID() {
	return _ID;
}

float Vertex::getX() {
	return _X;
}

float Vertex::getY() {
	return _Y;
}

std::pair<float, float> Vertex::getPosition()
{
	return std::make_pair(_X, _Y);
}
