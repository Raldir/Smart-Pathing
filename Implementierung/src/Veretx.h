#pragma once

#include <iostream>
#include "main.h"

class Vertex {

	public:
		void Vertex();

		TrafficLight* trafficLight;

		void transferCar(Car *car, Street *street);

		/*
			Funktionalität für Graphengeneration
		*/
		void addIncomingEdges(Street *street);

		void addOutgoingEdges(Street *street);

	private:

		Edge* incomingEdges[];
		Edge* outgoingEdges[];
};