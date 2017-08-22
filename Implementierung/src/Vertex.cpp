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


Vertex::Vertex(int id, float x, float y, TrafficLight tL) : _X(x), _Y(y), _ID(id) {
	trafficLight = tL;
}

void Vertex::setTrafficLight(TrafficLight tL) {
	trafficLight = tL;
}

TrafficLight* Vertex::getTrafficLight()
{
	TrafficLight* ptr = &trafficLight;

	return ptr;
}

//Initialize update wave for edges that have a red phased edge
void Vertex::Update() {

	//Update trafficLight
	trafficLight.Update();

	//Für Ripple Update
	//std::pair<int, int> pair = trafficLight.getCurrentPhase();
	/*for (auto p : trafficLight.getPossiblePhases()) {

		if (p != pair) {
			//Update edges which have a red light (no green light)
			getEdgeFromID(p.first)->Update();
			getEdgeFromID(p.second)->Update();
		}
	}*/
}

/*
###### CARSTUFF
*/

/*
	Takes stored vertex of car and searches for the edge connecting to vertex then
	transfers car onto this edge
*/
void Vertex::transferCar(int incomingEdgeID) {

	Car* car = getEdgeFromID(incomingEdgeID)->getFrontCar();

	Edge* nextEdge = NULL;
	bool nextEdgeFound = false;

	for (std::pair<int, Edge*> e : outgoingEdges) {

		int endVertexID;

		if (e.second->getVertices().second != NULL) { 
			endVertexID = e.second->getVertices().second->getID(); 
		}
		else {
			break;
		}
		int nextVertex = car->getNextVertexID();

		//Look if one of the edges has Vertex with matching ID from car
		if (endVertexID == nextVertex) {
			nextEdge = e.second;
			nextEdgeFound = true;
			break;
		}
	}

	//If there is an edge the car can transported to
	if (nextEdgeFound) {
		if (!nextEdge->isFull()) {

			Car* car = takeCar(incomingEdgeID);

			giveCar(nextEdge, car);
			std::cout << "VERTEX" << _ID << ", transferred car " << car->getID() << " from " << incomingEdgeID << " to " << nextEdge->getID() << std::endl;



			//Removes the next point as destination
			car->popCurrentVertex();
		}
		else {
			//DO NOTHING AS NOTHING WAS CHANGED
		}
	}
	else {
		//TODO No next edge found?
		std::cout << "No edge/vertex found leading to next vertex " << car->getNextVertexID() << "!" << std::endl;
	}
}

//Hilfsfunktion falls sich was an der Struktur verändert
Car* Vertex::takeCar(int incomingEdgeID) {

	//Remove car from edge
	Car* car = getEdgeFromID(incomingEdgeID)->popCar();

	return car;
}

void Vertex::giveCar(Edge* outgoingEdge, Car* car)
{
	outgoingEdge->pushCar(car);
}

void Vertex::destroyCar(Car* car) {
	delete car;
}

bool Vertex::canTransit(int incomingEdgeID, int outgoingEdgeID) {
	//TODO Implement when Traffic Light is ready

	//return !isEdgeFullMap[outgoingEdgeID];
	return trafficLight.canCross(incomingEdgeID) &&!(getEdgeFromID(outgoingEdgeID)->isFull());
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
	isEdgeFullMap[edge->getID()] = true;
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

void Vertex::setIsEdgeFull(int outgoingEdgeID, bool isFull)
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
