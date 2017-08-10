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

void Edge::pushCar(Car * car) {

	carQueue.push(car);
}

Car * Edge::getFrontCar() {

	return carQueue.front();
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
	do {
		std::cout << "Car" << getFrontCar()->getID() << "\n";
		q.pop();
	} while (!q.empty());
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
void Edge::notifyVertex(Car* car) {
	//Gives 
	vertex->TakeCar(this, car);
}

int Edge::getObserver() {
	return vertex->getID();
}
