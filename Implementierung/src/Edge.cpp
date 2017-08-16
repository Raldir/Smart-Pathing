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

	//TODO Initalize timetable correctly
	//int timetable[_SIMULATION_TIME];
}

Edge::Edge(float length, int id) : _LENGTH(length) {
	//TODO Capacität nicht ganz korrekt, da auch bspw. für ein Auto ein Gap existieren muss damit es Kapazität 1 hat
	_carQueueCapacity = int(length / (_CAR_LENGTH + _CAR_MINIMUM_GAP));
	_ID = id;


	//TODO Initalize timetable correctly
	//int timetable[_SIMULATION_TIME];
}

Edge::Edge(float length, int id, std::pair<Vertex*, Vertex*> nodes) : _LENGTH(length) {
	_carQueueCapacity = int(length / _CAR_LENGTH);
	startVertex = nodes.first;
	endVertex = nodes.second;
	_ID = id;


	//TODO Initalize timetable correctly
	//int timetable[_SIMULATION_TIME];
}

/*
###### TIMETABLE
*/


//TODO Timetable stuff
/**
Adds weight to the timetable at specified time
*/
void Edge::addWeightTimetable(int time, int weight) {

	//Add weight to timetable
	//timetable[time] += weight;
}

void Edge::removeWeightTimetable(int time, int weight) {

	//Remove weight by using add function
	//Edge::addWeightTimetable(time, weight * -1);
}

/*
###### QUEUE
*/

///<summary>
///Removes first car from queue
///</summary>
Car * Edge::popCar() {

	if (!carQueue.empty())
	{
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

void Edge::removeObserver(Vertex * vertex, std::string indicator) {
	//TODO gucken wie geht nullpointer
	//vertex = NULL;
}

///<summary>
///Notifies obversers
///</summary>
void Edge::notifyVerticies(Edge* edge) {

	if (isFull() != lastTickIsFull) {
		startVertex->setEdgeIsFull(_ID, isFull());

		lastTickIsFull = isFull();

		std::cout << "Toggled lastTickIsFull flag to " << lastTickIsFull << std::endl;
	}

	//TODO Implementiere Notification bei Transfer
	if (getFrontCar()->getCurrentPosition() == 0) {
		std::cout << "Called Vertex " << endVertex->getID() << "to transfer Car" << std::endl;

		endVertex->transferCar(_ID);
	}
}

std::pair<Vertex*, Vertex*> Edge::getVertices()
{
	return std::make_pair(startVertex, endVertex);
}

