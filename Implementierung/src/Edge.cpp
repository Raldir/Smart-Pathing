#include <iostream>
#include <queue>
#include <utility>
#include "ObserverPattern.h"
#include "Vertex.h"
#include "Car.h"
#include "Edge.h"
#include "Main.h"

Edge::Edge(float length, int capacity, int id) : _LENGTH(length) {

	_carQueueCapacity = capacity;
	_ID = id;
	lastTickIsFull = false;

	timetableSpan = calculateTimetableSpan(_TIMETABLE_SPAN);
	timetableInterval = _LENGTH / _CAR_SPEED;
	if (timetableInterval < 1) timetableInterval = 1;
}

Edge::Edge(float length, int id) : _LENGTH(length) {
	//TODO Capacität nicht ganz korrekt, da auch bspw. für ein Auto ein Gap existieren muss damit es Kapazität 1 hat
	_carQueueCapacity = int(length / _CAR_MINIMUM_GAP) + 1;
	_ID = id;
	lastTickIsFull = false;

	timetableSpan = calculateTimetableSpan(_TIMETABLE_SPAN);
	timetableInterval = _LENGTH / _CAR_SPEED;
	if (timetableInterval < 1) timetableInterval = 1;
}

Edge::Edge(float length, int id, std::pair<Vertex*, Vertex*> nodes) : _LENGTH(length) {
	_carQueueCapacity = int(length / _CAR_MINIMUM_GAP) + 1;
	startVertex = nodes.first;
	endVertex = nodes.second;
	_ID = id;

	timetableSpan = calculateTimetableSpan(_TIMETABLE_SPAN);
	timetableInterval = _LENGTH / _CAR_SPEED;
	if (timetableInterval < 1) timetableInterval = 1;
	lastTickIsFull = false;
}

void Edge::Update(int currentTick) {

	hasOverflowCars = false;

	//Copy carQueue
	std::deque<Car*> copy = carQueue;

	//Set the first critical as end of street (with gap compensating because end of street theoretically has a gap behind it)
	float nextCarPosition = _LENGTH + _CAR_MINIMUM_GAP;

	//Iterate through carQueue
		for (std::deque<Car*>::iterator it2 = copy.begin(); it2 != copy.end();it2++) {
			Car* car = *(it2);

			//Update car's position
			if (currentTick != car->getCurrentTick()) {
				car->Update(nextCarPosition);
			}
			//std::cout << "Car " << car->getID() << " on pos " << car->getCurrentPosition() << std::endl;

			car->setCurrentTick(currentTick);

			//After each updated car looks whether or not edge is empty or a car needs to be transfered
			notifyVerticies();

			//Testing wheter or not car has transitioned
			if (car->getCurrentVertexID() == endVertex->getID() && !car->isMarkedAsDeleted()) {
				//std::cout << "Can be TRANSITIONED " << car->hasOverflow() << std::endl;
				nextCarPosition = car->getCurrentPosition();

				//If this car has remaining overflow and is still on this edge
				if (hasOverflowCars == false && car->hasOverflow()) {
					hasOverflowCars = true;
				}
			}
			//If it has the street end is the next crititcal position
			else {
				nextCarPosition = _LENGTH + _CAR_MINIMUM_GAP;
			}

			//Delete car
			//if (car->isMarkedAsDeleted()) {
			//	it2 = carQueue.erase(it2);
			//	delete car;
			//}
			//else it2++;

			//Reveal next car
}
//Ripple-Update
//startVertex->ContinueUpdate(_ID);
}

void Edge::UpdateOverflow() {

	if (hasOverflowCars) {

		//Reset this flag
		hasOverflowCars = false;

		std::deque<Car*> copy = carQueue;

		//Get first car position for beginning of loop
		float nextCarPosition = _LENGTH + _CAR_MINIMUM_GAP;

		//Iterate through carQueue
		//while (!copy.empty()) {
		for (std::deque<Car*>::iterator it2 = copy.begin(); it2 != copy.end();it2++) {
			Car* car = *(it2);

			//Update car's position
			car->UpdateWithOverflow(nextCarPosition);

			//After each updated car
			//TODO Look wheter or not overflow etc. is handled on other side
			notifyVerticies();

			//If car is still on edge
			if (car->getCurrentVertexID() == endVertex->getID()) {
				nextCarPosition = car->getCurrentPosition();

				//If this car has remaining overflow and is still on this edge
				if (hasOverflowCars == false && car->hasOverflow()) {
					hasOverflowCars = true;
				}
			}
			//If it has the street end is the next crititcal position
			else {
				nextCarPosition = _LENGTH + _CAR_MINIMUM_GAP;
			}

			//Reveal next car
			//copy.pop();
		}
	}
}

/*
###### TIMETABLE
*/

/**
Adds weight to the timetable at specified time
*/
void Edge::addWeightTimetable(int timeStamp, int weight) {

	//Gibt gültigen TimeStamp für timetable aus
	int timeStampAdjusted = calculateTimestamp(timeStamp);

	timetable[timeStampAdjusted % timetableSpan] = timetable[timeStampAdjusted % timetableSpan] + weight;
}

void Edge::removeWeightTimetable(int timeStamp, int weight) {
	addWeightTimetable(timeStamp, weight * -1);
}

int Edge::getWeightTimetable(int timeStamp) {
	return timetable[calculateTimestamp(timeStamp)];
}

//Rundet ab auf nächst kleineren wert
int Edge::calculateTimestamp(int timeStamp) {

	return (timeStamp / timetableInterval) * timetableInterval;
}

int Edge::calculateInterval(int interval) {
	return interval;
}

int Edge::calculateTimetableSpan(int i) {
	return i;
}

/*
###### QUEUE
*/

///<summary>
///Removes first car from queue
///</summary>
Car * Edge::popCar() {
	std::cout << "DELETED CAR";
	if (!carQueue.empty()) {
		//Save pointer for car in front of queue
		Car* carPtr = *carQueue.begin();
		carQueue.erase(carQueue.begin());

		//Remove car from queue
		//carQueue.pop();

		return carPtr;
	}
	else {
		return NULL;
	}
}

int Edge::numberOfCars()
{
	return carQueue.size();
}

void Edge::pushCar(Car* car) {

	float backPosition;

	if (!carQueue.empty() && car != carQueue.front())
	{
		backPosition = carQueue.back()->getCurrentPosition();
	}
	else {
		backPosition = _LENGTH + _CAR_MINIMUM_GAP;
	}

	car->setPosition(0.0);
	carQueue.push_back(car);

	car->UpdateWithOverflow(backPosition);

	if (car->hasOverflow()) {
		hasOverflowCars = true;
	}
}

Car * Edge::getFrontCar() {

	if (!carQueue.empty()) {
		return carQueue.front();
	}
	else {
		return NULL;
	}
}

///<summary>
///
///</summary>
bool Edge::isFull() {

	if (!carQueue.empty()) {
		//std::cout << "check is full";
		return carQueue.back()->getCurrentPosition() <= _CAR_MINIMUM_GAP || carQueue.size() >= _carQueueCapacity;
	}
	else {
		return false;
	}
}

void Edge::printCars() {

	std::deque<Car*> q = carQueue;

	std::cout << "EDGE " << _ID << std::endl;

	//Iterate through queue and printing cars
	//Wofür hier eine DoWhile?!?
	//if (!q.empty()) {
	//	do {
	//		//std::cout << "Car " << q.front()->getID() << " on pos " << q.front()->getCurrentPosition() << std::endl;
	//		//std::cout << "Car on pos " << q.front()->getCurrentPosition() << std::endl;
	//		q.pop();
	//	} while (!q.empty());
	//}
}

float Edge::getLength() {
	return _LENGTH;
}

int Edge::getID() {
	return _ID;
}

/*
###### OBSERVER
*/

void Edge::registerObserver(Vertex * vertex, std::string indicator) {
	if (indicator == "end")
	{
		endVertex = vertex;
	}
	else if (indicator == "start") {
		startVertex = vertex;
	}
}

void Edge::removeObserver(std::string indicator) {

	if (indicator == "end") {
		endVertex = NULL;
	}
	else if (indicator == "start") {
		startVertex = NULL;
	}
}

///<summary>
///Notifies obversers
///</summary>
void Edge::notifyVerticies() {

	//Wenn die Edge voll ist
	/*if (isFull() != lastTickIsFull && startVertex != NULL) {
		startVertex->setIsEdgeFull(_ID, isFull());

		lastTickIsFull = isFull();

		std::cout << "EDGE " << _ID << " toggled lastTickIsFull flag to " << lastTickIsFull << std::endl;
	}*/

	//If a car has reached the end of the street
	if (!carQueue.empty() && getFrontCar()->getCurrentPosition() >= _LENGTH) {
		//std::cout << "Called Vertex " << endVertex->getID() << "to transfer Car " << this->getFrontCar()->getID() << std::endl;

		endVertex->transferCar(_ID);
	}
}

std::pair<Vertex*, Vertex*> Edge::getVertices()
{
	return std::make_pair(startVertex, endVertex);
}

bool Edge::hasOverflow() {
	return hasOverflowCars && endVertex->getTrafficLight()->canCross(_ID);
}
