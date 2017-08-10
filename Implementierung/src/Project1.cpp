#include <iostream>
#include <queue>
#include "main.h"
#include "Edge.h"
#include "Vertex.h"
#include "Car.h"

int main() {
	
	std::cout << "Hello";

	Car car(45);
	Car * carPtr;

	carPtr = &car;

	Edge edge(10, 10, 12);
	Edge * edgePtr;
	
	//Assign Pointer to edge
	edgePtr = &edge;
	
	Vertex vertex(89);
	Vertex * vertexPtr;

	vertexPtr = &vertex;

	vertexPtr->addIncomingEdges(edgePtr);

	vertexPtr->printEdges();

	edgePtr->pushCar(carPtr);
	edgePtr->pushCar(carPtr);
	edgePtr->pushCar(carPtr);
	edgePtr->pushCar(carPtr);

	edgePtr->printCars();

	std::cout << vertexPtr->getId();

	system("PAUSE");

	return 0;
}