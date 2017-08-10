#pragma once

#include <iostream>
#include <queue>
#include "main.h"

class Vertex;
class Edge;

class Car {

public:
	Car(int id);

	//Updates position of car for one tick on the edge.
	void updatePosition(float nextCarPosition);

	//Updates the currentEdge of the car
	void updateCurrentEdge(Edge * edge);

	//Pops current Vertex so the next point on the path is available
	void popCurrentVertex();

	//Gets current nearest vertex the car is headed for
	Vertex* getCurrentVertex();

	//Assigns new queue to path (when path is recalculated) and deletes the old one
	void assignRoute(std::queue<Vertex*> q);

	//Gets ID of Car
	int getID();

private:
	//Stores the route as queue of ids
	std::queue<Vertex*> route;

	//Current Position on edge
	float currentPosition;

	int _ID;

};

