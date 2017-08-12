#pragma once

#include <iostream>
#include <vector>
#include "main.h"

class Edge;
class Car;

class ObserverVertex {

public:
	virtual void TakeCar(Edge *edge, Car *car) {};

	virtual int getID() {};

private:
	int _ID;
};

class SubjectEdge {

private:
	int _ID;

protected:
	ObserverVertex* vertex;

public:
	virtual void registerObserver(ObserverVertex * obs) {};
	virtual void removeObserver(ObserverVertex * obs) {};

	//Notifies attached Vertex that car has reached position 0
	virtual void notifyVertex(Car* car) {};

	virtual ObserverVertex* getObserver() { return vertex; };

	virtual int getID() = 0;
};
