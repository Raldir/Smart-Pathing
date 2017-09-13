#include <iostream>
#include <queue>
#include <utility>
#include "ObserverPattern.h"
#include "Vertex.h"
#include "Car.h"
#include "Edge.h"
#include "main.h"
#include <algorithm> 
#include <deque>

Edge::Edge(float length, int capacity, int id) : _LENGTH(length), _carQueueCapacity(capacity), _ID(id) {

	timetableSpan = calculateTimetableSpan(_TIMETABLE_SPAN);
	timetableInterval = _LENGTH / _CAR_SPEED;
	if (timetableInterval < 1) timetableInterval = 1;
}

Edge::Edge(float length, int id, std::pair<Vertex*, Vertex*> nodes) : Edge(length, int(length / _CAR_MINIMUM_GAP) + 1, id) {

	startVertex = nodes.first;
	endVertex = nodes.second;
}

void Edge::Update(int currentTick) {

	//Reset this flag to prevent this edge from being updated unnecessarily
	hasOverflowCars = false;

	//Set the first critical as end of street (with gap compensating because end of street theoretically has a gap behind it)
	float nextCarPosition = _LENGTH + _CAR_MINIMUM_GAP;

	//Get a copy of the queue to work on
	std::deque<Car*> copy = carQueue;

	//Iterate through carQueue
	for (std::deque<Car*>::iterator it2 = copy.begin(); it2 != copy.end();) {
		Car* car = *(it2);

		//When car has already been updated by another edge, don't update it in the first Update phase
		if (currentTick != car->getCurrentTick()) {
			car->Update(nextCarPosition);
		}
		//std::cout << "Car " << car->getID() << " on pos " << car->getCurrentPosition() << std::endl;

		//After update, refresh tick counter of car
		car->setCurrentTick(currentTick);

		//Calls upon end vertex to transfer car when certain conditions are met
		notifyVertex();

		//Testing wheter or not car has transitioned and is not marked as to be deleted
		if (car->getCurrentVertexID() == endVertex->getID() && !car->isMarkedAsDeleted()) {
			//std::cout << "Can be TRANSITIONED " << car->hasOverflow() << std::endl;
			nextCarPosition = car->getCurrentPosition();

			//If this car has remaining overflow
			if (!hasOverflowCars && car->hasOverflow()) {
				hasOverflowCars = true;
			}
		}
		//If car has transferred the street end is the next crititcal position
		else {
			nextCarPosition = _LENGTH + _CAR_MINIMUM_GAP;
		}

		//Delete car
		if (car->isMarkedAsDeleted()) {
			std::cout << "VERTEX " << endVertex->getID() << ", EDGE " << _ID << ", DELETE CAR!" << std::endl;
			it2 = copy.erase(it2);
			carQueue.erase(std::remove(carQueue.begin(), carQueue.end(), car));
			delete car;
		}
		//Put iterator on next car in queue
		else it2++;
	}
	//Ripple-Update
	//startVertex->ContinueUpdate(_ID);
}

void Edge::UpdateOverflow() {

	//Redundant check
	if (hasOverflowCars) {

		//Reset this flag
		hasOverflowCars = false;

		//Get first car position for beginning of loop
		float nextCarPosition = _LENGTH + _CAR_MINIMUM_GAP;

		//Get a copy of the queue to work on
		std::deque<Car*> copy = carQueue;

		//Iterate through carQueue
		//while (!copy.empty()) {
		for (std::deque<Car*>::iterator it2 = copy.begin(); it2 != copy.end();) {
			Car* car = *(it2);

			//Update car's position
			car->UpdateWithOverflow(nextCarPosition);

			//Checks wheter or not a transfer can take place or car is to be deleted
			notifyVertex();

			//If car is still on edge
			if (car->getCurrentVertexID() == endVertex->getID() && !car->isMarkedAsDeleted()) {
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
			if (car->isMarkedAsDeleted()) {
				std::cout << "VERTEX " << endVertex->getID() << ", EDGE " << _ID << ", DELETE CAR!" << std::endl;
				it2 = copy.erase(it2);
				carQueue.erase(std::remove(carQueue.begin(), carQueue.end(), car));
				delete car;
			}
			//Put iterator on next car in queue
			else it2++;
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

	if (timetable[calculateTimestamp(timeStamp)] == 0) {
		timetable.erase(calculateTimestamp(timeStamp));
	}
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

int Edge::getEdgeCapacity() {
	return _carQueueCapacity;
}
/*
###### QUEUE
*/

///<summary>
///Removes first car from queue
///</summary>
Car * Edge::popCar() {
	std::cout << "DELETED CAR";
	std::cout << "CarQueue Size " << carQueue.size() << std::endl;
	if (!carQueue.empty()) {
		//Save pointer for car in front of queue
		Car* carPtr = *carQueue.begin();
		carQueue.pop_front();

		//Remove car from queue
		//carQueue.pop();
		std::cout << "CarQueue Size After " << carQueue.size() << std::endl;
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

	//Get position of current last car in edge
	float backPosition;

	//Get correct position for update
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
		//std::cout <<"Current VertexID of Front in Car: " <<carQueue.front()->getCurrentVertexID()<<std::endl;
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
void Edge::notifyVertex() {

	//Wenn die Edge voll ist
	/*if (isFull() != lastTickIsFull && startVertex != NULL) {
		startVertex->setIsEdgeFull(_ID, isFull());

		lastTickIsFull = isFull();

		std::cout << "EDGE " << _ID << " toggled lastTickIsFull flag to " << lastTickIsFull << std::endl;
	}*/

	//If a car has reached the end of the street with a small margin
	if (!carQueue.empty() && getFrontCar()->getCurrentPosition() >= _LENGTH * 0.98) {

		std::cout << "Car transfer initiated by vertex " << endVertex->getID() << " from edge " << _ID << std::endl;
		//std::cout << "Called Vertex " << endVertex->getID() << "to transfer Car " << this->getFrontCar()->getID() << std::endl;
		endVertex->transferCar(_ID);
	}
	else {
		std::cout << "Edge " << _ID << " to vertex " << endVertex->getID() << ", transfer not possible!" << std::endl;
	}
}

std::pair<Vertex*, Vertex*> Edge::getVertices()
{
	return std::make_pair(startVertex, endVertex);
}

bool Edge::hasOverflow() {
	return hasOverflowCars && endVertex->getTrafficLight()->canCross(_ID);
}
