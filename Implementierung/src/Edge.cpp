#include <iostream>
#include <queue>
#include <utility>
#include "ObserverPattern.h"
#include "Vertex.h"
#include "Car.h"
#include "Edge.h"

Edge::Edge(float length, int capacity, int id) : _LENGTH(length) {

	_carQueueCapacity = capacity;

	_ID = id;


	//TODO Initalize timetable correctly
	//int timetable[_SIMULATION_TIME];
}

float Edge::getLength()
{
	return _LENGTH;
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

	//Save pointer for car in front of queue
	Car * carPtr = carQueue.front();

	//Remove car from queue
	carQueue.pop();

	return carPtr;
}

void Edge::pushCar(Car* car) {

	carQueue.push(car);
}

Car * Edge::getFrontCar() {

	return carQueue.front();
}

/*
	Tells us whether or not space is still available (also accounting for minimum distance
*/
bool Edge::isFull() {

	bool queueFull = carQueue.size() >= _carQueueCapacity;

	/*
		If the gap behind the cars stretches over the edge of the street (smaller than 0)
		then there is no place for another car
	*/
	bool spaceFull = 0 > (carQueue.back()->getCurrentPosition() - _CAR_MINIMUM_GAP);

	return queueFull || spaceFull;
}

void Edge::printCars() {

	std::queue<Car*> q = carQueue;

	std::cout << "Printe nun Autos auf der Queue" << std::endl;

	//Iterate through queue and printing cars
	if (!q.empty()) {
		do {
			std::cout << "Print Car:" << q.front()->getID() << " on Position " << q.front()->getCurrentPosition() << std::endl;
			q.pop();
		} while (!q.empty());
	}

}

int Edge::getID() {
	return _ID;
}


/*
###### OBSERVER
*/

void Edge::registerObserver(ObserverVertex * vertex) {
	this->vertex = vertex;
}

void Edge::removeObserver(ObserverVertex * vertex) {
	//TODO gucken wie geht nullpointer
	//vertex = NULL;
}

///<summary>
///Notifies obversers
///</summary>
void Edge::notifyVertex(Edge* edge) {
	//Gives 
}

ObserverVertex* Edge::getObserver() {
	return vertex;
}

