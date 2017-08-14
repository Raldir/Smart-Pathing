#pragma once

#include <iostream>
#include <queue>
#include <utility>
#include "main.h"
#include "ObserverPattern.h"


class Car;
class Vertex;

class Edge : virtual SubjectEdge {

public:
	//Constructor
	Edge(float length, int capacity, int id);

	Edge(float length, int id);

	Edge(float length, int id, std::pair<Vertex*, Vertex*>);


	float getLength();

	//Adds and remove weights on timetable at specified time
	void addWeightTimetable(int time, int weight);
	void removeWeightTimetable(int time, int weight);

	//Pusht car auf queue
	void pushCar(Car* car);

	//Entfernt car von queue
	Car* popCar();

	//Gibt Auto an erster Stelle in Queue aus
	Car* getFrontCar();

	//Checkt ob Edge voll ist
	bool isFull();

	void printCars();

	virtual int getID() override;

	virtual void registerObserver(Vertex * obs, std::string indicator) override;
	virtual void removeObserver(Vertex * obs, std::string indicator) override;

	//Notifies attached Vertex that car has reached position 0
	virtual void notifyVerticies(Edge* edge) override;

	virtual std::pair<Vertex*, Vertex*> getVerticies() override;

private:

	//Länge der Straße
	float _LENGTH;

	//Id
	int _ID;

	//Anzahl der Autos in der Queue
	int carQueueCounter;

	//Maximale Kapazität der Queue
	int _carQueueCapacity;

	//Queue in der die Autos gespeichert werden.
	std::queue<Car*> carQueue;

	//Timetable in dem die Gewichte für jeden Zeitabschnitt gespeichert werden.
	int timetable[10];

	//Indicates if the edge was full last tick
	bool lastTickIsFull;
};