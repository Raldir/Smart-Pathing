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

#include <boost/graph/astar_search.hpp>
#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/random.hpp>
#include <boost/random.hpp>
#include <boost/graph/graphviz.hpp>
#include <time.h>
#include <list>
#include <fstream>
#include <math.h>    // for sqrt


using namespace boost;
using namespace std;

int main() {

	//testChristoph();
	testRami();


	return 0;
}


void testRami()
{
	Graph* g = new Graph();
	system("PAUSE");
  
}


void testChristoph() {

	std::cout << "TEST COUT" << std::endl;

	const int vertexAmount = 2;

	//Prints Edges of Vertex
	Vertex vertices[2];
	Vertex* vertexPtrs[2];

	for (int i = 0; i < vertexAmount; i++) {

		Vertex vertex(i,0,0);
		std::cout << "Vertex mit ID: " << vertex.getID() << " erstellt." << std::endl;

		vertices[i] = vertex;

		vertexPtrs[i] = &vertices[i];

		std::cout << "VertexPtr Test: VertexOriginalID: " << vertices[i].getID() << ". VertexPtrID: " << vertexPtrs[i]->getID() << std::endl;
	}

	Edge edge(10, 1, std::make_pair(vertexPtrs[0], vertexPtrs[1]));
	Edge edge2(100,10,2);
	Edge edge0(100,5,0);

	edge0.registerObserver(vertexPtrs[0],"end");

	Edge * edgePtr;
	Edge * edgePtr2;
	Edge * edgePtr0;

	//Assign Pointer to edge
	edgePtr = &edge;
	edgePtr2 = &edge2;
	edgePtr0 = &edge0;

	vertexPtrs[0]->addIncomingEdges(edgePtr0);
	vertexPtrs[0]->addOutgoingEdges(edgePtr);
	vertexPtrs[1]->addIncomingEdges(edgePtr);
	vertexPtrs[1]->addOutgoingEdges(edgePtr2);

	vertexPtrs[0]->printEdges();
	vertexPtrs[1]->printEdges();

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
	edgePtr->printCars();

	//ROUTING TABLES
	RoutingTable table(new Graph());

	std::deque<int> d{ vertexPtrs[0]->getID(), vertexPtrs[1]->getID() };
	std::queue<int> q(d);

	table.insertRoute(0,1,q);

	table.getRoute(0,1).empty();
	std::queue<int> route2 = table.getRoute(1, 0);

	carsPtr[0]->assignRoute(q);

	//SUPER TRANSFER STUFF
	//edgePtr0->pushCar(carsPtr[0]);
	//std::cout << "Pushed car with ID: "<< edgePtr0->getFrontCar()->getID() << " on Edge 0" << std::endl;

	//vertexPtrs[0]->transferCar(0);

	//std::cout << "Car " << edgePtr->getFrontCar()->getID() << "is on Edge 1" << std::endl;

	/*edgePtr0->pushCar(carsPtr[1]);
	std::cout << "Position of Car 1: " << carsPtr[1]->getCurrentPosition() << std::endl;
	edgePtr0->Update();
	std::cout << "Position of Car 1: " << carsPtr[1]->getCurrentPosition() << std::endl;*/


	system("PAUSE"); //From C
}

