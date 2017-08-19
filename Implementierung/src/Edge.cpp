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
}

Edge::Edge(float length, int id) : _LENGTH(length) {
	//TODO Capacität nicht ganz korrekt, da auch bspw. für ein Auto ein Gap existieren muss damit es Kapazität 1 hat
	_carQueueCapacity = int(length / (_CAR_LENGTH + _CAR_MINIMUM_GAP));
	_ID = id;
}

Edge::Edge(float length, int id, std::pair<Vertex*, Vertex*> nodes) : _LENGTH(length) {
	_carQueueCapacity = int(length / (_CAR_LENGTH + _CAR_MINIMUM_GAP));
	startVertex = nodes.first;
	endVertex = nodes.second;
	_ID = id;
}

void Edge::Update() {
	//Copy carQueue
	std::queue<Car*> copy = carQueue;

	//Set the first critical as end of street
	float nextCarPosition = _LENGTH;

	//Iterate through carQueue
	while (!copy.empty()) {
		Car* car = copy.front();

		//Taking overflow position or speed as updating method
		/*if (cycleNumber <= 1) {
			car->Update(nextCarPosition);
		}
		else {
			car->updateWithOverflowPosition(nextCarPosition);
		}*/

		//Update car's position
		car->Update(nextCarPosition);

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

		//After each updated car
		notifyVerticies();
	}

	//Tell vertex to update preceding vertecies
	startVertex->ContinueUpdate(_ID);
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
	car->updateWithOverflowPosition(carQueue.back()->getCurrentPosition());
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

	return carQueue.size() >= _carQueueCapacity;
}

void Edge::printCars() {

	std::queue<Car*> q = carQueue;

	//Iterate through queue and printing cars
	if (!q.empty()) {
		do {
			std::cout << "Print Car:" << q.front()->getID() << " on Position " << q.front()->getCurrentPosition() << std::endl;
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

	if (isFull() != lastTickIsFull) {
		startVertex->setEdgeIsFull(_ID, isFull());

		lastTickIsFull = isFull();

		std::cout << "Toggled lastTickIsFull flag to " << lastTickIsFull << std::endl;
	}

	//TODO Implementiere Notification bei Transfer
	if (getFrontCar()->getCurrentPosition() >= _LENGTH) {
		std::cout << "Called Vertex " << endVertex->getID() << "to transfer Car" << std::endl;

		endVertex->transferCar(_ID);
	}
}

std::pair<Vertex*, Vertex*> Edge::getVertices()
{
	return std::make_pair(startVertex, endVertex);
}

