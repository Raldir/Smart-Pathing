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
}

Edge::Edge(float length, int id) : _LENGTH(length) {
	//TODO Capacit�t nicht ganz korrekt, da auch bspw. f�r ein Auto ein Gap existieren muss damit es Kapazit�t 1 hat
	_carQueueCapacity = int(length / _CAR_MINIMUM_GAP);
	_ID = id;

	lastTickIsFull = false;
}

Edge::Edge(float length, int id, std::pair<Vertex*, Vertex*> nodes) : _LENGTH(length) {
	_carQueueCapacity = int(length / _CAR_MINIMUM_GAP);
	startVertex = nodes.first;
	endVertex = nodes.second;
	_ID = id;

	lastTickIsFull = false;
}

void Edge::Update() {
	hasOverflowCars = false;

	//Copy carQueue
	std::queue<Car*> copy = carQueue;

	//Set the first critical as end of street
	float nextCarPosition = _LENGTH;

	//Iterate through carQueue
	while (!copy.empty()) {
		Car* car = copy.front();

		//Update car's position
		car->Update(nextCarPosition);

		//If this car has remaining overflow
		if (car->hasOverflow()) {
			//If the first relevant car Position has not been set yet
			if (firstOverflowCarPosition == NULL) {
				firstOverflowCarPosition = nextCarPosition;
			}
			//Push car on queue for further processing by second Update;
			overflowQueue.push(car);
		}

		//After each updated car
		//TODO Look whether or not overflow etc. is handled on other side
		notifyVerticies();

		//Testing wheter or not car has transitioned
		if (car->getCurrentVertexID() == endVertex->getID()) {
			nextCarPosition = car->getCurrentPosition();
		}
		//If it has the street end is the next crititcal position
		else {
			nextCarPosition = _LENGTH;
		}
		
		//Reveal next car
		copy.pop();
	}
	//Ripple-Update
	//startVertex->ContinueUpdate(_ID);

	if (overflowQueue.size() > 0) {
		hasOverflowCars = true;
	}
}

void Edge::UpdateOverflow() {

	if (hasOverflowCars) {
		Car* car = overflowQueue.front();

		float nextCarPosition = firstOverflowCarPosition;

		//Iterate through carQueue
		while (!overflowQueue.empty()) {
			Car* car = overflowQueue.front();

			//Update car's position
			car->UpdateWithOverflow(nextCarPosition);

			//After each updated car
			//TODO Look wheter or not overflow etc. is handled on other side
			notifyVerticies();

			//Testing wheter or not car has transitioned
			if (car->getCurrentVertexID() == endVertex->getID()) {
				nextCarPosition = car->getCurrentPosition();
			}
			//If it has the street end is the next crititcal position
			else {
				nextCarPosition = _LENGTH;
			}

			//Reveal next car
			overflowQueue.pop();
		}
	}

	if (!overflowQueue.empty()) {
		overflowQueue = std::queue<Car*>();
	}
	hasOverflowCars = false;
}

/*
###### TIMETABLE
*/

/**
Adds weight to the timetable at specified time
*/
void Edge::addWeightTimetable(int timeStamp, int weight) {

	//Gibt g�ltigen TimeStamp f�r timetable aus
	int timeStampAdjusted = calculateTimestamp(timeStamp);

	timetable[timeStampAdjusted % timetableSpan] = timetable[timeStampAdjusted % timetableSpan] + weight;
}

void Edge::removeWeightTimetable(int timeStamp, int weight) {
	addWeightTimetable(timeStamp, weight * -1);
}

int Edge::getWeightTimetable(int timeStamp) {
	return timetable[calculateTimestamp(timeStamp)];
}

//Rundet ab auf n�chst kleineren wert
int Edge::calculateTimestamp(int timeStamp) {

	return (timeStamp / timetableInterval) * timetableInterval;
}

int Edge::calculateInterval(int crossingDistanceGraph) {
	return crossingDistanceGraph;
}

/*
###### QUEUE
*/

///<summary>
///Removes first car from queue
///</summary>
Car * Edge::popCar() {

	if (!carQueue.empty()) {
		//Save pointer for car in front of queue
		Car* carPtr = carQueue.front();

		//Remove car from queue
		carQueue.pop();

		return carPtr;
	}
	else {
		return NULL;
	}
}

void Edge::pushCar(Car* car) {

	carQueue.push(car);

	car->setPosition(0.0);
	car->UpdateWithOverflow(carQueue.back()->getCurrentPosition());

	if (isFull()) {
		notifyVerticies();
	}
}

Car * Edge::getFrontCar() {

	if (!carQueue.empty()) 	{
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

	return carQueue.size() >= _carQueueCapacity && carQueue.back()->getCurrentPosition() > _CAR_MINIMUM_GAP;
}

void Edge::printCars() {

	std::queue<Car*> q = carQueue;

	//Iterate through queue and printing cars
	if (!q.empty()) {
		do {
			std::cout << "EDGE " << _ID << ", car " << q.front()->getID() << " on Position " << q.front()->getCurrentPosition() << std::endl;
			q.pop();
		} while (!q.empty());
	}
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
	if (isFull() != lastTickIsFull) {
		startVertex->setIsEdgeFull(_ID, isFull());

		lastTickIsFull = isFull();

		std::cout << "Toggled lastTickIsFull flag to " << lastTickIsFull << std::endl;
	}

	//If a car has reached the end of the street
	if (getFrontCar()->getCurrentPosition() >= _LENGTH) {
		std::cout << "Called Vertex " << endVertex->getID() << "to transfer Car" << std::endl;

		endVertex->transferCar(_ID);
	}
}

std::pair<Vertex*, Vertex*> Edge::getVertices()
{
	return std::make_pair(startVertex, endVertex);
}

bool Edge::hasOverflow() {
	return hasOverflowCars;
}
