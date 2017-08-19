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

void Car::Update(float nextCarPosition) {

	//Just to be sure because overflow is only needed when you cross an intersection
	if (overflow != 0) {
		//Now we can safely calculate new overflow
		overflow = 0;
	}

	float newPosition = currentPosition + _CAR_SPEED_PER_TICK;

	if (newPosition >= (nextCarPosition - _CAR_MINIMUM_GAP)) {

		//Drive to the max position
		currentPosition = nextCarPosition - _CAR_MINIMUM_GAP;

		overflow = newPosition - currentPosition;
	}
	else {
		//If no obstacles are found
		currentPosition = newPosition;

		overflow = 0;
	}
}

void Car::updateWithOverflowPosition(float nextCarPosition) {

	if (overflow > 0) {
		if (overflow >= (nextCarPosition - _CAR_MINIMUM_GAP)) {
			//Drive to the max position
			currentPosition = nextCarPosition - _CAR_MINIMUM_GAP;
		}
		else {
			//If no obstacles are found
			currentPosition = overflow;

			overflow = 0;
		}
	}
}

void Car::setPosition(float newPosition) {
	currentPosition = newPosition;
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

int Car::getNextVertexID()
{
	if (!route.empty()) {
		auto routeCopy = route;
		routeCopy.pop();
		if (!route.empty()) {
			return route.front();
		}
		else {
			return NULL;
		}
	}
	else {
		return NULL;
	}
}

int Car::getID() {
	return _ID;
};