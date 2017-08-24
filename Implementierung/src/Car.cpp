#include <iostream>
#include <utility>
#include "Car.h"

Car::Car(int cT) {
	currentTick = cT;
	spawnTick = cT;

	toBeDeleted = false;
}

Car::~Car()
{
	//TODO
}

void Car::Update(float nextCarPosition) {

	float nextCriticalPosition = nextCarPosition - _CAR_MINIMUM_GAP;

	//Just to be sure because overflow is only needed when you cross an intersection
	if (overflow != 0) {
		//Now we can safely calculate new overflow
		overflow = 0;
	}

	//Calculate new position
	float oldPosition = currentPosition;
	float newPosition = currentPosition + _CAR_SPEED_PER_TICK;

	if (newPosition >= nextCriticalPosition) {

		//Drive to the max position
		currentPosition = nextCriticalPosition;
		overflow = newPosition - currentPosition;
	}
	else {
		//If no obstacles are found
		currentPosition = newPosition;

		overflow = 0;
	}
}

void Car::UpdateWithOverflow(float nextCarPosition) {

	//Next critical position where car cannot move further
	float nextCriticalPosition = nextCarPosition - _CAR_MINIMUM_GAP;

	if (overflow > 0) {
		//Calculate new position
		float newPosition = overflow + currentPosition;

		if (newPosition >= nextCriticalPosition) {
			//Drive to the max position
			currentPosition = nextCarPosition - _CAR_MINIMUM_GAP;

			//Calculate new overflow
			overflow = newPosition - currentPosition;
		}
		else {
			//If no obstacles are found
			currentPosition = newPosition;

			overflow = 0;
		}
	}
}

bool Car::hasOverflow() {
	return overflow > 0;
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

void Car::addDistanceTravelled(float edgeLength) {
	distanceTravelled += edgeLength;
}

float Car::getDistanceTravelled() {
	return distanceTravelled;
}

int Car::getSpawnTick()
{
	return spawnTick;
}

void Car::setCurrentTick(int tick) {
	currentTick = tick;
}

int Car::getCurrentTick()
{
	return currentTick;
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

int Car::getNextVertexID() {
	if (!route.empty()) {
		auto copy = route;
		copy.pop();
		if (!copy.empty()) {
			return copy.front();
		} 
		else {
			return NULL;
		}
	}
	else {
		return NULL;
	}
}

int Car::getDestination() {
	return route.back();
}

int Car::getID() {
	return _ID;
};

void Car::markAsDeleted() {
	toBeDeleted = true;
}

bool Car::isMarkedAsDeleted() {
	return toBeDeleted;
}