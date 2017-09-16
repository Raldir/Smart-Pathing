/*############################################

Author: Rami Aly and Christoph Hueter
Date : 20.09.17
Libraries and Licences:
Boost Library 1.60, MPI Boost Library 1.60, Open Streetmaps andMPI in use.
All used Maps are licensed under the Open Street Maps License.

#############################################*/

#pragma once
#include <iostream>
#include <vector>
#include "main.h"

class Edge;
class Car;
class Vertex;

/*
Fundamental Parent class of Edges using observer Pattern for communication
*/
class SubjectEdge {

protected:
	Vertex* endVertex;
	Vertex* startVertex;
	int _ID;

public:
	virtual void registerObserver(Vertex * obs, std::string indicator) {};
	virtual void removeObserver(std::string indicator) {};

	//Notifies attached Vertex that car has reached position 0
	virtual void notifyVertex() = 0;

	virtual std::pair<Vertex*, Vertex*> getVertices() = 0;

	virtual int getID() = 0;
};
