#pragma once

#include <iostream>
#include <utility>
#include <map>
#include "main.h"
#include "Edge.h"
#include "TrafficLight.h"

class Edge;
class Car;
class TrafficLight;

class Vertex {

public:
	Vertex() {};

	Vertex(int id, float x, float y);
	Vertex(int id, float x, float y, TrafficLight tL);

	void setTrafficLight(TrafficLight tL);
	TrafficLight* getTrafficLight();

	float distanceTo(Vertex* v);

	void Update();

	//TrafficLight* trafficLight;

	/*
		Makes the actual transition of the car
		ONLY use when certain that transtition will be possible
	*/
	void transferCar(int incomingEdgeID);

	//Hilfefunktion
	Car* takeCar(int incomingEdgeID);
	void giveCar(Edge* edge, Car* car);

	//Checks wheter TrafficLight allow transit and wheter or not the next edge is full
	bool canTransit(int incomingEdgeID, int outgoingEdgeID);

	void addIncomingEdges(Edge *edge);
	void addOutgoingEdges(Edge *edge);

	std::vector<Edge*> getIncomingEdges();
	std::vector<Edge*> getOutgoingEdges();

	std::vector<Edge*> getEdges();

	Edge* outgoingNeighbor(int destID);
	Edge* incomingNeighbor(int destID);

	Edge* getEdgeFromID(int edgeID);

	void printEdges();

	float getX();
	float getY();

	int getID();

	std::pair<float, float> getPosition();

	std::vector<Car*> popTransitioningCars(int outgoingEdgeID);

protected:

	TrafficLight trafficLight;

	void setProcessOfEdge(int edgeID, int processID);
	int getProcessOfVertex(int vertexID);

	/*
		int -> ID of edge
		Edge* -> pointer to the actual edge
	*/
	std::vector<Edge*> incomingEdges;
	std::vector<Edge*> outgoingEdges;

	float _X;
	float _Y;
	int _ID;
	int _rank;

	/*
		Maps Edge ID to its process
		Needed for transitioning between two processes

		first int -> edgeID
		second int -> processID
	*/
	std::map<int, int> processMap;

	//TODO Nötig?
	bool isInsideProcess(int edgeID);

	//TODO
	/*
		first int -> edge
		second int -> freeSpace
	*/
	std::map<int, int> freeEdgeSpaceMap;

	//After each update is finished transitioning cars will have been stored here
	std::map<int, std::vector<Car*>> transitioningCars;
};