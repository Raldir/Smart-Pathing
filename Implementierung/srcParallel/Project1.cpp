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
#include "Simulation.h"
#include <stdio.h>
#include "mpi.h"

#ifdef BOOST_NO_EXCEPTIONS
void
boost::throw_exception(std::exception const& ex)
{
	std::cout << ex.what() << std::endl;
	abort();
}
#endif

using namespace boost;
using boost::graph::distributed::mpi_process_group;


int main(int argc, char *argv[]) {

	//testChristoph();
	//Graph* _graph = new Graph();
	//std::vector<int> ids = _graph->createSpawnerIDVector();
	//RoutingTable* table = new RoutingTable(_graph, _NEAREST_NEIGHBOR, ids);
	//std::vector<std::vector<int>> matrix = table->getRoutingMatrix();

	//mpi::environment env(argc, argv);

	MPI_Init(&argc, &argv);
	//mpi::environment env(argc, argv);
	//test_distributed_dfs();
	int numberProcesses;
	int rank;
	MPI_Comm_rank(MPI_COMM_WORLD, &rank);
	MPI_Comm_size(MPI_COMM_WORLD, &numberProcesses);
	std::cout<<"RANK: "<<rank<<" NUMBER numberProcesses: " << numberProcesses<<std::endl;
	//Graph* graph = new Graph();
	//RoutingTable* table = new RoutingTable(graph, _NEAREST_NEIGHBOR);
	Simulation* s = new Simulation(numberProcesses, rank);

	MPI_Finalize();
	//testRami();

	
	return 0;
}


void testRami()
{
	//Simulation* s = new Simulation();
	//Graph* g = new Graph();
	//RoutingTable* table = new RoutingTable(g, 7);
	//system("PAUSE");

}


void testChristoph() {
	/*
		std::cout << "TEST COUT" << std::endl;

		const int vertexAmount = 3;

		std::vector<std::pair<int, int>> tLMap;

		tLMap.push_back(std::make_pair(0, 1));
		tLMap.push_back(std::make_pair(1, 2));
		tLMap.push_back(std::make_pair(2, 3));

		//Prints Edges of Vertex
		Vertex vertices[3];
		Vertex* vertexPtrs[3];
		TrafficLight tL(tLMap, 10, 0);

		for (int i = 0; i < vertexAmount; i++) {

			Vertex vertex(i + 1, 0, 0, tL);
			std::cout << "Vertex mit ID: " << vertex.getID() << " erstellt." << std::endl;

			vertices[i] = vertex;

			vertexPtrs[i] = &vertices[i];

			std::cout << "VertexPtr Test: VertexOriginalID: " << vertices[i].getID() << ". VertexPtrID: " << vertexPtrs[i]->getID() << std::endl;
		}

		Edge edge(100, 1, std::make_pair(vertexPtrs[0], vertexPtrs[1]));
		Edge edge2(100, 10, 2);
		Edge edge0(100, 10, 0);
		Edge edge3(100, 15, 3);

		edge0.registerObserver(vertexPtrs[0], "end");
		edge.registerObserver(vertexPtrs[0], "start");
		edge.registerObserver(vertexPtrs[1], "end");
		edge2.registerObserver(vertexPtrs[1], "start");
		edge2.registerObserver(vertexPtrs[2], "end");

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

		for (Vertex* v : vertexPtrs) {
			v->printEdges();
		}

		const int carAmount = 30;

		Car* carsPtr[30];
		Car carStorage[30];

		/*for (int c = 0; c < carAmount; c++) {
			Car car = new Car(c);

			carStorage[c] = car;
		}

		for (int i = 0; i < carAmount; i++) {

			std::cout << "Counter: " << i << std::endl;
			//std::cout << "Made car with ID " << carStorage[i].getID() << std::endl;

			carsPtr[i] = new Car();
			//std::cout << "Car: " << carsPtr[i]->getID() << " ready." << std::endl;

		}

		//Transfer car
		std::deque<int> d{ vertexPtrs[0]->getID(), vertexPtrs[1]->getID(), vertexPtrs[2]->getID() };

		//ROUTING TABLES
		RoutingTable table(new Graph(), 7);

		std::queue<int> q(d);

		for (Car* c : carsPtr) {
			c->assignRoute(q);
		}

		//carsPtr[0]->setPosition(20.0);
		//carsPtr[1]->setPosition(0.0);

		//SUPER TRANSFER STUFF

		//edgePtr[0]->pushCar(carsPtr[1]);
		//std::cout << "Position of Car 1: " << carsPtr[1]->getCurrentPosition() << std::endl;

		int k = 0;

		for (int i = 1; i < 50; i++) {
			if (carsPtr[k]->getRoute().size() > 4) {
				k++;
				continue;
			}
			bool hasOverflow = true;

			std::cout << "############## CYCLE " << i << std::endl;
			if (!edgePtr[0]->isFull() && carsPtr[k] != NULL) {
				edgePtr[0]->pushCar(carsPtr[k]);
				std::cout << "Pushed car on Edge 0" << std::endl;
				//std::cout << "Pushed car with ID: " << edgePtr[0]->getFrontCar()->getID() << " on Edge 0" << std::endl;
				k++;
			}

			for (Vertex* v : vertexPtrs) {
				v->Update();
			}

			std::cout << "Nach Update:" << std::endl;
			for (Edge* ed : edgePtr) {
				//std::cout << "EDGE " << ed->getID() << std::endl;
				ed->Update(i);
				ed->printCars();
			}

			int j = 0;

			while (j < 4) {
				std::cout << "Nach OverflowUpdate:" << std::endl;
				for (Edge* ed : edgePtr) {
					//std::cout << "EDGE " << ed->getID() << std::endl;
					ed->UpdateOverflow();
					ed->printCars();
				}
				j++;
			}
		}

		//TESTING reverseQueue
		/*std::queue<int> abc;

		for (int k = 0; k < 20; k++) {
			abc.push(k);
		}

		auto p = RoutingTable::reverseQueue(abc);

		while (!abc.empty()) {
			std::cout << abc.front() << std::endl;
			abc.pop();
		}

		while (!p.empty()) {
			std::cout << p.front() << std::endl;
			p.pop();
		}

		system("PAUSE"); //From C */
}


