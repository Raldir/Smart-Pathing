#include "Edge.h"
#include <iostream>
#include <queue>
#include <utility>

void Edge::Edge(float length) {
	_LENGTH = length;
}

/**
Adds weight to the timetable at specified time
*/
void Edge::addWeight(int time, int weight) {

}

/**
	Puts car into queue
*/
void Edge::addCar(Car* car) {
	carQueue.push(*car);
}

///<summary>
///Removes first car from queue
///</summary>
void Edge::removeCar() {

	//Save pointer for car in front of queue
	Car* car = carQueue.front();

	//Remove car from queue
	carQueue.pop();
}

///<summary>
///Notifies obversers
///</summary>
void Edge::notify() {

}

class TrafficLight {

	void toggle() {

	}
};

class Intersection {

};

class Car {

};

