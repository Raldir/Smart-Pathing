#pragma once

#include <iostream>
#include <vector>
#include "main.h"

class Edge;
class Car;

class ObserverVertex {

public:
	ObserverVertex(int id, float x, float y) : _X(x), _Y(y), _ID(id) {};

	virtual void transferCar(Edge* edge) = 0;

	virtual int getID() = 0;

	virtual std::pair<float, float> getPosition() = 0;

protected:

	float _X;
	float _Y;

	int _ID;
};

class SubjectEdge {

protected:
	ObserverVertex* endVertex;
	ObserverVertex* startVertex;
	int _ID;

public:
	virtual void registerObserver(ObserverVertex * obs) {};
	virtual void removeObserver(ObserverVertex * obs) {};

	//Notifies attached Vertex that car has reached position 0
	virtual void notifyVertex(Edge* edge) {};

	virtual ObserverVertex* getObserver() { return endVertex; };

	virtual int getID() = 0;
};
