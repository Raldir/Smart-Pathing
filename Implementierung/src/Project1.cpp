#include <iostream>
#include <queue>
#include <vector>
#include "main.h"
#include "Edge.h"
#include "Vertex.h"
#include "Car.h"
#include "Project1.h"
#include "VertexFileReader.h"
#include "EdgeFileReader.h"
#include "RoutingTable.h"
#include "Graph.h"
#include <time.h>
#include <list>
#include <fstream>
#include <math.h>    // for sqrt



int main() {

	//testChristoph();
	testRami();

	return 0;
}


void testRami()
{
	Graph* g = new Graph();
	RoutingTable* table = new RoutingTable(g, 7);
	system("PAUSE");
  
}

void testChristoph() {

	std::cout << "TEST COUT" << std::endl;

	const int vertexAmount = 3;

	//Prints Edges of Vertex
	Vertex vertices[3];
	Vertex* vertexPtrs[3];

	for (int i = 0; i < vertexAmount; i++) {

		Vertex vertex(i, 0, 0);
		std::cout << "Vertex mit ID: " << vertex.getID() << " erstellt." << std::endl;

		vertices[i] = vertex;

		vertexPtrs[i] = &vertices[i];

		std::cout << "VertexPtr Test: VertexOriginalID: " << vertices[i].getID() << ". VertexPtrID: " << vertexPtrs[i]->getID() << std::endl;
	}

	Edge edge(10, 1, std::make_pair(vertexPtrs[0], vertexPtrs[1]));
	Edge edge2(100, 10, 2);
	Edge edge0(100, 5, 0);
	Edge edge3(100, 15, 3);

	edge0.registerObserver(vertexPtrs[0], "end");
	edge.registerObserver(vertexPtrs[0], "start");
	edge.registerObserver(vertexPtrs[1], "end");
	edge2.registerObserver(vertexPtrs[1], "start");
	edge2.registerObserver(vertexPtrs[2], "end");
	edge3.registerObserver(vertexPtrs[2], "start");

	Edge* edgePtr[4];

	//Assign Pointer to edge
	edgePtr[1] = &edge;
	edgePtr[2] = &edge2;
	edgePtr[0] = &edge0;
	edgePtr[3] = &edge3;

	vertexPtrs[0]->addIncomingEdges(edgePtr[0]);
	vertexPtrs[0]->addOutgoingEdges(edgePtr[1]);
	vertexPtrs[1]->addIncomingEdges(edgePtr[1]);
	vertexPtrs[1]->addOutgoingEdges(edgePtr[2]);
	vertexPtrs[2]->addIncomingEdges(edgePtr[2]);
	vertexPtrs[2]->addOutgoingEdges(edgePtr[3]);

	for (Vertex* v : vertexPtrs) {
		v->printEdges();
	}

	const int carAmount = 4;

	Car* carsPtr[4];
	Car carStorage[4];

	for (int c = 0; c < carAmount; c++) {
		Car car(c);

		carStorage[c] = car;
	}

	for (int i = 0; i < carAmount; i++) {

		std::cout << "Counter: " << i << std::endl;
		std::cout << "Made car with ID " << carStorage[i].getID() << std::endl;

		carsPtr[i] = &carStorage[i];
		std::cout << "Pointer points to car :" << carsPtr[i]->getID() << std::endl;

		for (int j = 0; j <= i; j++) {

			std::cout << "Car: " << carsPtr[j]->getID() << " on queue." << std::endl;
		}
	}

	//Transfer car


	//Prints Cars in Edge
	edgePtr[1]->printCars();

	std::deque<int> d{ vertexPtrs[0]->getID(), vertexPtrs[1]->getID(), vertexPtrs[2]->getID() };

	//ROUTING TABLES
	RoutingTable table(new Graph(), 7);

	std::queue<int> q(d);

	carsPtr[0]->assignRoute(q);
	carsPtr[1]->assignRoute(q);

	//SUPER TRANSFER STUFF
	edgePtr[0]->pushCar(carsPtr[0]);
	std::cout << "Pushed car with ID: " << edgePtr[0]->getFrontCar()->getID() << " on Edge 0" << std::endl;
	edgePtr[0]->pushCar(carsPtr[1]);
	std::cout << "Position of Car 1: " << carsPtr[1]->getCurrentPosition() << std::endl;

	for (int i = 0; i < 2; i++) {

		for (Vertex* v : vertexPtrs) {
			v->Update();
		}

		std::cout << "Nach Update " << i << ":" << std::endl;
		for (Edge* ed : edgePtr) {
			ed->Update();
			ed->printCars();
		}

		std::cout << "Nach OverflowUpdate " << i << ":" << std::endl;
		for (Edge* ed : edgePtr) {
			ed->UpdateOverflow();
			ed->printCars();
		}
	}

	system("PAUSE"); //From C
}

