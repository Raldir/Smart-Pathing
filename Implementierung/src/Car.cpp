#include <iostream>
#include <utility>
#include "Car.h"

Car::Car(int id) {
	_ID = id;
}

void Car::updatePosition(float nextCriticalPosition) {

	float newPosition = currentPosition + _CAR_SPEED_PER_TICK;

	if (newPosition >= (nextCriticalPosition - _CAR_MINIMUM_GAP)) {

	}
}

void Car::updateCurrentEdge(Edge* edge) {
	currentEdge = edge;
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