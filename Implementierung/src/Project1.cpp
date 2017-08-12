#include <iostream>
#include <queue>
#include <vector>
#include "main.h"
#include "Edge.h"
#include "Vertex.h"
#include "Car.h"
#include "Project1.h"


int main() {

	test1();	

	//Platz für Rami

	return 0;
}

void test1() {

	std::cout << "TEST COUT";

	Edge edge(10, 10, 12);
	Edge * edgePtr;

	//Assign Pointer to edge
	edgePtr = &edge;

	Vertex vertex(3000);
	Vertex * vertexPtr;

	vertexPtr = &vertex;

	vertexPtr->addIncomingEdges(edgePtr);

	//Prints Edges of Vertex
	vertexPtr->printEdges();

	const int carAmount = 4;

	Car* cars[4];
	Car carss[4];

	for (int c = 0; c < carAmount; c++) {
		Car car(c);

		carss[c] = car;
	}

	for (int i = 0; i < carAmount; i++) {

		std::cout << "Counter: " << i << std::endl;

		std::cout << "Made car with ID " << carss[i].getID() << std::endl;

		cars[i] = &carss[i];

		std::cout << "Pointer points to car :" << cars[i]->getID() << std::endl;

		for (int j = 0; j <= i; j++) {

			std::cout << "Car: " << cars[j]->getID() << " on queue." << std::endl;
		}
	}

	//Prints Cars in Edge
	edgePtr->printCars();

	system("PAUSE"); //From C
}

void test2() {

}

