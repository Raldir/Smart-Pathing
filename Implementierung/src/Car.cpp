#include <iostream>
#include <utility>
#include "Car.h"

Car::Car(int id) {
	_ID = id;
}

void Car::updatePosition(float nextCriticalPosition) {

	//Just to be sure because overflow is only needed when you cross an intersection
	if (overflowPosition > 0) {
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

int Car::getCurrentPosition() {
	return currentPosition;
}

void Car::assignRoute(std::queue<Vertex*> q) {
	route = q;
}

void Car::popCurrentVertex() {
	route.pop();
}

Vertex* Car::getCurrentVertex() {
	return route.front();
}

int Car::getID() { return _ID; };