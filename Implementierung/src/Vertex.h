#pragma once

#include <iostream>
#include <utility>
#include <map>
#include "main.h"
#include "ObserverPattern.h"

class Edge;
class Car;
class TrafficLight;

class Vertex {

public:
	Vertex() {};

	Vertex(int id, float x, float y);

	void setTrafficLight(TrafficLight* tL);

	void InitialUpdate();
	void ContinueUpdate(int edgeID);

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
	bool canTransit(int outgoingEdgeID);

	void addIncomingEdges(Edge *edge);
	void addOutgoingEdges(Edge *edge);

	std::vector<Edge*> getIncomingEdges();
	std::vector<Edge*> getOutgoingEdges();

	std::vector<Edge*> getEdges();

	Edge* getEdgeFromID(int edgeID);

	void setEdgeIsFull(int, bool);

	void printEdges();

	float getX();
	float getY();

	int getID();

	std::pair<float, float> getPosition();

private:

	TrafficLight* trafficLight;

	/*
		First int -> ID of edge
		Second bool -> is full or not
	*/
	std::map<int, bool> isEdgeFullMap;

	/*
		int -> ID of edge
		Edge* -> pointer to the actual edge
	*/
	std::map<int, Edge*> incomingEdges;
	std::map<int, Edge*> outgoingEdges;

	float _X;
	float _Y;
	int _ID;
};