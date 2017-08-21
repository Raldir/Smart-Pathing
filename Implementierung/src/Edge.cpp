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
	//TODO Capacität nicht ganz korrekt, da auch bspw. für ein Auto ein Gap existieren muss damit es Kapazität 1 hat
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

void Edge::Update(int currentTick) {

	hasOverflowCars = false;

	//Emtpy overflowQueue
	overflowQueue = std::queue<Car*>();

	//Copy carQueue
	std::queue<Car*> copy = carQueue;

	//Set the first critical as end of street (with gap compensating because end of street theoretically has a gap behind it)
	float nextCarPosition = _LENGTH + _CAR_MINIMUM_GAP;

	//Iterate through carQueue
	while (!copy.empty()) {
		Car* car = copy.front();

		//Update car's position
		if (currentTick != car->getCurrentTick()) {
			car->Update(nextCarPosition);
		}

		car->setCurrentTick(currentTick);

		//After each updated car
		//TODO Look whether or not overflow etc. is handled on other side
		notifyVerticies();

		//Testing wheter or not car has transitioned
		if (car->getCurrentVertexID() == endVertex->getID()) {
			nextCarPosition = car->getCurrentPosition();

			//If this car has remaining overflow and is still on this edge
			if (car->hasOverflow()) {
				//If the first relevant car Position has not been set yet
				if (firstOverflowCarPosition == NULL) {

					//Give end of street with gap correction or normal car position without correction
					if (carQueue.front() == car) {
						//End of street in front
						firstOverflowCarPosition = nextCarPosition + _CAR_MINIMUM_GAP;
					}
					else {
						//Normal car in front
						firstOverflowCarPosition = nextCarPosition;
					}
				}
				//Push car on queue for further processing by second Update;
				overflowQueue.push(car);
			}
		}
		//If it has the street end is the next crititcal position
		else {
			nextCarPosition = _LENGTH + _CAR_MINIMUM_GAP;
		}
		
		//Reveal next car
		copy.pop();
	}
	//Ripple-Update
	//startVertex->ContinueUpdate(_ID);

	if (!overflowQueue.empty()) {
		hasOverflowCars = true;
	}
	else {
		hasOverflowCars = false;
	}
}

void Edge::UpdateOverflow() {
	while (!overflowPushBuffer.empty()) {
		overflowQueue.push(overflowPushBuffer.front());
		overflowPushBuffer.pop();
	}

	if (hasOverflowCars) {
		//Copy overflowQueue
		auto copy = overflowQueue;

		//Empty overflow queue, so a new one can be built
		overflowQueue = std::queue<Car*>();

		//Get first car position for beginning of loop
		float nextCarPosition = firstOverflowCarPosition;

		//Set it to null so the first car to have overflow gets the correct critical position for the next iteration
		firstOverflowCarPosition = NULL;

		//Iterate through carQueue
		while (!copy.empty()) {
			Car* car = copy.front();

			//Update car's position
			car->UpdateWithOverflow(nextCarPosition);

			//After each updated car
			//TODO Look wheter or not overflow etc. is handled on other side
			notifyVerticies();

			//Testing wheter or not car has transitioned
			if (car->getCurrentVertexID() == endVertex->getID()) {
				nextCarPosition = car->getCurrentPosition();

				//If this car has remaining overflow and is still on this edge
				if (car->hasOverflow()) {
					
					//If the first relevant car OverflowPosition has not been set yet
					if (firstOverflowCarPosition == NULL) {

						//Give end of street with gap correction or normal car position without correction
						if (carQueue.front() == car) {
							//End of street in front
							firstOverflowCarPosition = nextCarPosition + _CAR_MINIMUM_GAP;
						}
						else {
							//Normal car in front
							firstOverflowCarPosition = nextCarPosition;
						}
					}
					//Push car on queue for further processing by second Update;
					overflowQueue.push(car);
				}
			}
			else {
				nextCarPosition = _LENGTH + _CAR_MINIMUM_GAP;
			}

			//Reveal next car
			copy.pop();
		}
	}

	if (!overflowQueue.empty()) {
		hasOverflowCars = true;
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

	if (car != carQueue.back())
	{
		car->UpdateWithOverflow(carQueue.back()->getCurrentPosition());
	}
	else {
		car->UpdateWithOverflow(_LENGTH + _CAR_MINIMUM_GAP);
	}

	if (car->hasOverflow()) {
		overflowPushBuffer.push(car);
	}

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

	if (!carQueue.empty()) {
		return carQueue.back()->getCurrentPosition() > _CAR_MINIMUM_GAP;
	}
	else {
		return false;
	}
}

void Edge::printCars() {

	std::queue<Car*> q = carQueue;

	std::cout << "EDGE " << _ID << std::endl;

	//Iterate through queue and printing cars
	if (!q.empty()) {
		do {
			std::cout << "Car " << q.front()->getID() << " on pos " << q.front()->getCurrentPosition() << std::endl;
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
	if (isFull() != lastTickIsFull && startVertex != NULL) {
		startVertex->setIsEdgeFull(_ID, isFull());

		lastTickIsFull = isFull();

		std::cout << "EDGE " << _ID << " toggled lastTickIsFull flag to " << lastTickIsFull << std::endl;
	}

	//If a car has reached the end of the street
	if (getFrontCar()->getCurrentPosition() >= _LENGTH) {
		std::cout << "Called Vertex " << endVertex->getID() << "to transfer Car " << this->getFrontCar()->getID() << std::endl;

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
