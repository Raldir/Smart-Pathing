#pragma once

#include <iostream>
#include <queue>
#include <map>
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

	void Update(int currentTick);
	void UpdateOverflow();

	float getLength();

	//Adds and remove weights on timetable at specified time
	void addWeightTimetable(int timeStamp, int weight);
	void removeWeightTimetable(int timeStamp, int weight);

	int getWeightTimetable(int weight);

	int calculateTimestamp(int time);
	int calculateInterval(int crossingDistanceGraph);

	int calculateTimetableSpan(int i);

	//Pusht car auf queue
	void pushCar(Car* car);

	//Entfernt car von queue und gibt sie zurück
	Car* popCar();

	int numberOfCars();
	//Gibt Auto an erster Stelle in Queue aus
	Car* getFrontCar();

	//Checkt ob Edge voll ist
	bool isFull();

	void printCars();

	virtual int getID() override;

	virtual void registerObserver(Vertex * obs, std::string indicator) override;
	virtual void removeObserver(std::string indicator) override;

	//Notifies attached Vertex that car has reached position 0 or that the edge is full
	virtual void notifyVerticies() override;

	virtual std::pair<Vertex*, Vertex*> getVertices() override;

	//Indicates wheter there are cars still to be updated for the second update wave
	bool hasOverflow();

private:

	//Länge der Straße
	float _LENGTH;

	//Id
	int _ID;

	//Maximale Kapazität der Queue
	int _carQueueCapacity;

	//Queue in der die Autos gespeichert werden.
	std::deque<Car*> carQueue;

	//Indicates if the edge was full last tick
	bool lastTickIsFull;

	//Timetable in dem die Gewichte für jeden Zeitabschnitt gespeichert werden.
	std::map <int,int> timetable;

	//Zeitspanne
	int timetableSpan;
	//Interval of timetable
	int timetableInterval;

	//Are there still cars with overflow to be calculated in the queue?
	bool hasOverflowCars;
	//Position of car in front of first car with overflow
	float firstOverflowCarPosition;

	//Tick after most recent updates
	int currentTick;
};