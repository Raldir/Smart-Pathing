#include <iostream>
#include <utility>
#include "Vertex.h"
#include "ObserverPattern.h"
#include "Car.h"
#include "Edge.h"
#include "TrafficLight.h"
#include "Graph.h"
#include "main.h"

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

float Vertex::distanceTo(Vertex * v)
{
	return sqrt(pow(_X - v->getX(), 2.0f) + pow(_Y - v->getY(), 2.0f));
}

//Initialize update wave for edges that have a red phased edge
void Vertex::Update() {

	//Update trafficLight
	trafficLight.Update();
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

	Edge* nextEdge;

	//Checks wheter or not this is the destination of this car
	if (_ID != car->getDestination()) {
		nextEdge = outgoingNeighbor(car->getNextVertexID());
		//std::cout << "PUT CAR ON" << this->getID() << " " << car->getNextVertexID() << std::endl;
		//If there is an edge the car can transported to
		if (nextEdge != NULL) {
			if (!nextEdge->isFull()) {
				std::cout << "CALLED on IncomingEdgeID " <<incomingEdgeID << std::endl;
				//Haben car schon, brauchen keinen neuen Pointer
				takeCar(incomingEdgeID);
				car->popCurrentVertex();

				//TODO PARALLEL
				giveCar(nextEdge, car);


				std::cout << "VERTEX" << _ID << ", transferred car " << car->getID() << " from " << incomingEdgeID << " to " << nextEdge->getID() << std::endl;

				//Removes the next point as destination
			}
		}
		//No edge found
		else {
			std::cout << "No edge/vertex found leading to next vertex " << car->getNextVertexID() << "!" << std::endl;
		}
	} 
	//If this is the destination -> take car from edge and destroy it
	else {
		car->markAsDeleted();
	}
}

//Nimmt car aus edge heraus und entfernt weight
Car* Vertex::takeCar(int incomingEdgeID) {

	//Get edge pointer
	Edge* e = getEdgeFromID(incomingEdgeID);
	//Remove car from edge
	Car* car = e->popCar();
	std::cout << "TOOK OUT CAR of EDGE " <<incomingEdgeID<< std::endl;
	//Add travelled distance to car inner variable
	car->addDistanceTravelled(e->getLength());

	//VON CHRISTOPH
	//Correct timestamp to remove weight from timetable 
	std::pair<int, int> timetableValues = Graph::calculateTimetableValues(car->getSpawnTick(), car->getDistanceTravelled());
	//Remove weight from timetable set at the moment of birth of car
	e->removeWeightTimetable(timetableValues.first, timetableValues.second);

	return car;
}

void Vertex::giveCar(Edge* outgoingEdge, Car* car)
{
	//TODO PARALLEL
	outgoingEdge->pushCar(car);
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
	incomingEdges.push_back(edge);
}

//Adds Pointer of outgoing edge to Vector
void Vertex::addOutgoingEdges(Edge* edge) {
	outgoingEdges.push_back(edge);
	//isEdgeFullMap[edge->getID()] = true;
}

std::vector<Edge*> Vertex::getIncomingEdges()
{
	std::vector<Edge*> v;

	for (Edge* e : incomingEdges) {
		v.push_back(e);
	}

	return v;
}


std::vector<Edge*> Vertex::getOutgoingEdges()
{
	std::vector<Edge*> v;

	for (Edge* e : outgoingEdges) {
		v.push_back(e);
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
	for (Edge* p : outgoingEdges) {
		//std::cout << "Current Edge:" << p->getVertices().first->getID() <<" " << p->getVertices().second->getID() << '\n';
		//std::cout << "GleichheitstesT: " <<p->getVertices().second->getID() << " " << destID <<'\n';
		if (p->getVertices().second->getID() == destID) {
			//std::cout << "return";
			return p;
		}
	}
	return NULL;
}

Edge* Vertex::incomingNeighbor(int destID) {
	for (Edge* p : incomingEdges) {
		if (p->getVertices().first->getID() == destID) {
			return p;
		}
	}
	return NULL;
}


Edge* Vertex::getEdgeFromID(int edgeID) {

	//Searches through the pairs in the map and matches the IDs
	for (Edge* p : incomingEdges) {
		if (p->getID() == edgeID) {
			return p;
		}
	}

	//Searches through the pairs in the map and matches the IDs
	for (Edge* p : incomingEdges) {
		if (p->getID() == edgeID) {
			return p;
		}
	}

	std::cout << "No Edge with matching id has been found" << std::endl;
	return NULL;
}

void Vertex::printEdges() {
	std::cout << "Vertex:" << _ID << " ";
	for ( Edge* e : incomingEdges) {
		std::cout << "Incoming Edge: " << e->getID() << std::endl;
	}

	for (Edge* e : outgoingEdges) {
		std::cout << "Vertex:" << _ID << " ";
		std::cout << "Outgoing Edge: " << e->getID() << std::endl;
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
