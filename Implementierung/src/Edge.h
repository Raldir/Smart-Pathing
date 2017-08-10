#pragma once

#include <iostream>
#include <queue>
#include <utility>
#include "main.h"


class Car;
class Vertex;
class ObserverVertex;

class Edge : SubjectEdge {

	private:

		//Länge der Straße
		float _LENGTH;

		//Anzahl der Autos in der Queue
		int carQueueCounter;

		//Maximale Kapazität der Queue
		int _carQueueCapacity;

		//Queue in der die Autos gespeichert werden.
		std::queue<Car*> carQueue;

		//Timetable in dem die Gewichte für jeden Zeitabschnitt gespeichert werden.
		int timetable[10];

	public:
		//Constructor
		Edge(float length, int capacity, int id);

		//Adds and removes vertex the edge is pointing to
		virtual void registerObserver(ObserverVertex * vertex);
		virtual void removeObserver(ObserverVertex * obs);

		//Adds and remove weights on timetable at specified time
		void addWeightTimetable(int time, int weight);
		void removeWeightTimetable(int time, int weight);

		//Pusht car auf queue
		void pushCar(Car *car);

		//Entfernt car von queue
		Car * popCar();

		//Gibt Auto an erster Stelle in Queue aus
		Car * getFirstCar();

		//Checkt ob Edge voll ist
		bool isFull();

		void printCars();

		//Id
		int _ID;

		virtual void registerObserver(ObserverVertex * obs) {};
		virtual void removeObserver(ObserverVertex * obs) {};

		//Notifies attached Vertex that car has reached position 0
		virtual void notifyVertex(Car* car);

		virtual int getObserver();
};