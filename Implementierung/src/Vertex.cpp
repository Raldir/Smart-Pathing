#include <iostream>
#include <utility>
#include "Vertex.h"
#include "ObserverPattern.h"
#include "Car.h"
#include "Edge.h"
#include "TrafficLight.h"

/**
*/
Vertex::Vertex(int id, float x, float y) : _X(x), _Y(y), _ID(id) {

}

void Vertex::setTrafficLight(TrafficLight* tL) {
	this->trafficLight = tL;
}

//Initialize update wave for edges that have a red phased edge
void Vertex::InitialUpdate() {

	trafficLight->Update();

	std::pair<int, int> pair = trafficLight->getCurrentPhase();

	for (auto p : trafficLight->getPossiblePhase()) {

		if (p != pair) {
			//Update edges which have a red light (no green light)
			getEdgeFromID(p.first)->Update();
			getEdgeFromID(p.second)->Update();
		}
	}
}

//Update for edges that do have a green phase
void Vertex::ContinueUpdate(int edgeID)
{
	std::pair<int,int> pair = trafficLight->getCurrentPhase();

	//If the endvertex of our calling edge is not the start vertex of a potential green phase edge then this edge can be updated
	if (getEdgeFromID(pair.first)->getVertices().first->getID() != getEdgeFromID(edgeID)->getVertices().second->getID()) {

		getEdgeFromID(pair.first)->Update();
	}

	if (getEdgeFromID(pair.second)->getVertices().first->getID() != getEdgeFromID(edgeID)->getVertices().second->getID()) {

		getEdgeFromID(pair.second)->Update();
	}
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
		if (e.second->getVertices().second->getID() == car->getNextVertexID()) {
			nextEdge = e.second;
			nextEdgeFound = true;
			break;
		}
	}

	//If there is an edge the car can transported to
	if (nextEdgeFound) {
		if (canTransit(nextEdge->getID())) {

			Car* car = takeCar(incomingEdgeID);

			giveCar(nextEdge, car);

			//Removes the next point as destination
			car->popCurrentVertex();
		}
		else {
			//DO NOTHING AS NOTHING WAS CHANGED
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

std::vector<Edge*> Vertex::getEdges()
{
	std::vector<Edge*> edges;
	std::vector<Edge*> outgoingEdges = getOutgoingEdges();
	std::vector<Edge*> incomingEdges = getIncomingEdges();
	edges.reserve(getOutgoingEdges().size() + getIncomingEdges().size());
	edges.insert(edges.end(), outgoingEdges.begin(), outgoingEdges.end());
	edges.insert(edges.end(), incomingEdges.begin(), incomingEdges.end());
	return edges;
}

Edge* Vertex::outgoingNeighbor(int destID) {
	for (std::pair<int, Edge*> p : outgoingEdges) {
		if (p.second->getVertices().second->getID() == destID) {
			return p.second;
		}
	}
	return NULL;
}

Edge* Vertex::incomingNeighbor(int destID) {
	for (std::pair<int, Edge*> p : incomingEdges) {
		if (p.second->getVertices().first->getID() == destID) {
			return p.second;
		}
	}
	return NULL;
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