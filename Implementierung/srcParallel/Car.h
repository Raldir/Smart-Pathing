#pragma once

#include <iostream>
#include <queue>
#include "main.h"

class Vertex;
class Edge;

class Car {

public:
	Car(int id);

	Car(int overfl, int distTravelled, std::queue<int> r);

	Car() {};

	~Car();

	//Updates position of car for one tick on the edge.
	void Update(float nextCarPosition);

	void UpdateWithOverflow(float nextCarPosition);

	bool hasOverflow();
	int getOverflow();

	//Updates position based on overflow
	//void updateWithOverflowPosition(float nextCarPosition);

	//Set position of car
	void setPosition(float newPosition);
	float getCurrentPosition();

	//Pops current Vertex so the next point on the path is available
	void popCurrentVertex();
	//Gets the vertex the car is headed for
	int getCurrentVertexID();
	//Gets the vertex the car wants to transition to on intersection
	int getNextVertexID();

	int getDestination();

	//Assigns new queue to path (when path is recalculated) and deletes the old one
	void assignRoute(std::queue<int> q);

	std::queue<int> getRoute();

	void addDistanceTravelled(float edgeLength);
	float getDistanceTravelled();

	//Moment car was born
	int getSpawnTick();

	//Get and set currentTick
	void setCurrentTick(int tick);
	int getCurrentTick();

	//Gets ID of Car
	int getID();

	void markAsDeleted();

	bool isMarkedAsDeleted();

private:
	/*
		Stores the route as queue of ids
		Next entry always shows vertex the car is heading for
	*/
	std::queue<int> route;

	//Current Position on edge
	float currentPosition;
	/*
		When reaching the maximum allow position on the edge the leftover
		distance is stored in the overflow variable to use
		it e.g. for crossing an intersection
		Probably only needed for intersection crossing and second update
	*/
	float overflow;

	float distanceTravelled;

	//Tick when car was spawned
	int spawnTick;
	//Tick des letzten Updates
	int currentTick;

	int _ID;

	bool toBeDeleted;
};

