#pragma once

#include <iostream>
#include <queue>
#include "main.h"


class Car {

public:
	Car(int id);

	//Pops current Vertex so the next point on the path is available
	//void pathPopCurrentVertex();

	//Gets current next destination
	//Vertex* getCurrentVertex();

	//Assigns new queue to path (when path is recalculated)
	//void pathAssignNewQueue();

	int _ID;

private:
	//Stores the path as queue of ids
	std::queue<int> path;
	
};

struct {

};
