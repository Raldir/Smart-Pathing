#include <iostream>
#include <utility>
#include "Car.h"

Car::Car(int id) {
	_ID = id;
}

Car::~Car()
{
	//TODO
}

void Car::updatePosition(float nextCriticalPosition) {

	//Just to be sure because overflow is only needed when you cross an intersection
	if (overflowPosition > 0) {

		//Now we can safely calculate new overflow
		overflowPosition = 0;
	}

	float newPosition = currentPosition + _CAR_SPEED_PER_TICK;

	if (newPosition >= (nextCriticalPosition - _CAR_MINIMUM_GAP)) {

		//Calculate overflow for crossing an intersection
		overflowPosition = newPosition - nextCriticalPosition;

		//Drive to the max position
		currentPosition = nextCriticalPosition;
	}
	else {
		//If no obstacles are found
		currentPosition = newPosition;
	}
}

float Car::getCurrentPosition() {
	return currentPosition;
}

void Car::assignRoute(std::queue<int> q) {
	route = q;
}

void Car::popCurrentVertex() {
	route.pop();
}

int Car::getCurrentVertexID() {

	if (!route.empty()) {
		return route.front();
	}
	else {
		return NULL;
	}
}

int Car::getID() {
	return _ID;
};