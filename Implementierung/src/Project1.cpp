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


int main() {

	testChristoph();
	testRami();

	return 0;
}


void testRami() {
	std::vector<Vertex*> vertices = readVertexFile("nodes");
	std::vector<Edge*> edges = calculateEdges(vertices, "edges");
	//connectVertices(edges);
	system("PAUSE");
}


void testChristoph() {

	std::cout << "TEST COUT" << std::endl;

	const int vertexAmount = 2;

	//Prints Edges of Vertex
	Vertex vertices[vertexAmount];
	Vertex* vertexPtrs[vertexAmount];

	for (int i = 0; i < vertexAmount; i++) {

		Vertex vertex(i,0,0);
		std::cout << "Vertex mit ID: " << vertex.getID() << " erstellt." << std::endl;

		vertices[i] = vertex;

		vertexPtrs[i] = &vertices[i];

		std::cout << "VertexPtr Test: VertexOriginalID: " << vertices[i].getID() << ". VertexPtrID: " << vertexPtrs[i]->getID() << std::endl;
	}

	Edge edge(10, 10, std::make_pair(vertexPtrs[0], vertexPtrs[1]));
	Edge edge2(1,10,100);
	Edge * edgePtr;
	Edge * edgePtr2;

	//Assign Pointer to edge
	edgePtr = &edge;
	edgePtr2 = &edge2;

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

	system("PAUSE"); //From C
}

