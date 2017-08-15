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
	
	//Platz für Rami

	//testChristoph();	
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


	std::cout << "TEST COUT";

	Edge edge(10, 10, 12);
	Edge * edgePtr;

	//Assign Pointer to edge
	edgePtr = &edge;

	Vertex vertex(3000, 0 , 0);
	Vertex * vertexPtr;

	vertexPtr = &vertex;

	vertexPtr->addIncomingEdges(edgePtr);

	//Prints Edges of Vertex
	vertexPtr->printEdges();

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

