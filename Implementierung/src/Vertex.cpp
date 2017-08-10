#include <iostream>
#include <utility>
#include "Vertex.h"
#include "ObserverPattern.h"
#include "Car.h"
#include "Edge.h"

/**
*/
Vertex::Vertex(int id) : _ID(id) {

	}


	void Vertex::transferCar(Car * car, std::pair<Edge*, Edge*> edges)
	{
		//TODO Transfers car
	}

	void Vertex::addIncomingEdges(Edge* edge) {
		//TODO!!!
		incomingEdges.push_back(edge);
	}

	void Vertex::addOutgoingEdges(Edge* edge) {

	}

	/*void Vertex::update(Edge *edge, Car *car) {

	}*/
	
	void Vertex::printEdges() {
		for (Edge* e : incomingEdges) {
			std::cout << e->_ID;
		}
	}

	int Vertex::getId() {
		return _ID;
	}

	void TakeCar(Edge* edge, Car* car) {

	}
