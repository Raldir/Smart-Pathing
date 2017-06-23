#pragma once

#include <iostream>
#include <queue>
#include <utility>
#include "main.h"

class Edge {

	private:

		//Kapazi‰t bei leerer Starﬂe
		int baseCapacity;

		//Kapazit‰t der Straﬂe
		int currentCapacity;

		//Queue in der die Autos gespeichert werden.
		std::queue<Car*> carQueue;

		//L‰nge der Straﬂe
		const float _LENGTH;

		//Timetable in dem die Gewichte f¸r jeden Zeitabschnitt gespeichert werden.
		int* timetable[_LENGTH * _CAR_SPEED]; 
		
		//Kreuzungen an denen die Straﬂe liegt.
		std::pair <Intersection, Intersection> nodes;

	public:
		//Constructor
		Edge(float length, );

		//Observer Pattern
		void notify();

		void addCar();

		void removeCar();

};