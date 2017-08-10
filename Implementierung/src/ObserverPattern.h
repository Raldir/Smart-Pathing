#pragma once

#include <iostream>
#include <vector>

class ObserverVertex {

public:
	virtual void TakeCar(Edge *edge, Car *car) {};
};

class SubjectEdge {

protected:
	ObserverVertex* obs;

public:
	virtual void registerObserver(ObserverVertex * obs) = 0;
	virtual void removeObserver(ObserverVertex * obs) = 0;
	
	//Notifies attached Vertex that car has reached position 0
	virtual void notifyVertex(Car* car) = 0;

	virtual void getObserver() = 0;
};
