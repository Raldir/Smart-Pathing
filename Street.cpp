#include <iostream>
#include <queue>
#include <utility>

class Street {

private:	
	//Capacity of street
	int capacity;

	//Length of street
	const int _LENGTH;

	int* timetable = new int[_LENGTH * movementSpeed];

	//Queue cars are waiting in
	std::queue<Car*> carQueue;


	std::pair <Intersection, Intersection> nodes;
	
public:

	/**
		Adds weight to the timetable at specified time
	*/
	void addWeight(int time, int weight) {

	}

	/**
		Puts car into queue
	*/
	void addCar(Car *car) {
		carQueue.push(*car);
	}

	void removeCar() {
		
		//Save pointer for car in front of queue
		Car *car = carQueue.front();

		//
		carQueue.pop();
	}

};

class TrafficLight {

	void Switch() {

	}
};

class Intersection {

};

class Car {

};

