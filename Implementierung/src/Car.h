#pragma once

#include <iostream>
#include <queue>
#include "main.h"

class Vertex;
class Edge;

class Car {

public:
	Car(int id);

	Car() {};

	//Updates position of car for one tick on the edge.
	void updatePosition(float nextCarPosition);

	int getCurrentPosition();

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

	/*
		When reaching the maximum allow position on the edge the leftover
		distance is stored in the overflow variable to use
		it e.g. for crossing an intersection
		Probably only needed for intersection crossing
	*/
	float overflowPosition;

	int _ID;

};

