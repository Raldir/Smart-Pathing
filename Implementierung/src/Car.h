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

	~Car();

	//Updates position of car for one tick on the edge.
	void updatePosition(float nextCarPosition);

	float getCurrentPosition();

	//Pops current Vertex so the next point on the path is available
	void popCurrentVertex();

	//Gets current nearest vertex the car is headed for
	int getCurrentVertexID();

	//Assigns new queue to path (when path is recalculated) and deletes the old one
	void assignRoute(std::queue<int> q);

	//Gets ID of Car
	int getID();

private:
	//Stores the route as queue of ids
	std::queue<int> route;

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

