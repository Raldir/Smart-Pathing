#pragma once

#include <iostream>
#include <vector>

class Edge;
class Car;

class ObserverVertex {

public:
	virtual void TakeCar(Edge *edge, Car *car) = 0;
	
	int getID();

private:
	int _ID;
};

class SubjectEdge {

protected:
	ObserverVertex* vertex;

public:
	virtual void registerObserver(ObserverVertex * obs) = 0;
	virtual void removeObserver(ObserverVertex * obs) = 0;
	
	//Notifies attached Vertex that car has reached position 0
	virtual void notifyVertex(Car* car) = 0;

	virtual int getObserver() = 0;
};
