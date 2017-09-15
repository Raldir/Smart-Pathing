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
//#include "mpi.h"

#include <iostream>
#include <boost/graph/use_mpi.hpp>
#include <boost/config.hpp>
#include <boost/throw_exception.hpp>
#include <boost/serialization/vector.hpp>
//#include <boost/graph/depth_first_search.hpp>
#include <boost/graph/dijkstra_shortest_paths.hpp>
#include <boost/graph/distributed/mpi_process_group.hpp>
#include <boost/graph/distributed/adjacency_list.hpp>
//#include <boost/test/minimal.hpp>

#ifdef BOOST_NO_EXCEPTIONS
void
boost::throw_exception(std::exception const& ex)
{
	std::cout << ex.what() << std::endl;
	abort();
}
#endif

using namespace boost;

int main(int argc, char *argv[]) {

	MPI_Init(NULL, NULL);

	int rank, world_size;

	MPI_Comm_rank(MPI_COMM_WORLD, &rank);
	MPI_Comm_size(MPI_COMM_WORLD, &world_size);

	std::cout << "Prozess " << rank << "Worldsize: " << world_size<<std::endl;
	Simulation* simulation = new Simulation(world_size, rank);
	MPI_Finalize();

	
	return 0;
}


/*****USED FOR SECOND ROUTINGMAP PARALLELISATION METHOD***************/
void parallelRoutingInitBoost(int argc, char *argv[]) {
	mpi::environment env(argc, argv);
	mpi::communicator world;
	std::cout << "I am process " << world.rank() << " of " << world.size()
		<< "." << std::endl;
	Simulation* simulation = new Simulation(world.rank(), world.size());
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
	std::map<int, int*> sendMap;
	std::map<int, int*> recvMap;

	for (int i = 0; i < world_size; i++) {
	sendMap[i] = new int[1];
	sendMap[i][0] = rank;

	recvMap[i] = new int[1];
	}

	std::cout << "Process " << rank << std::endl;
	for (int k = 0; k < world_size; k++) {
	std::cout << sendMap[k][0] << ", ";
	}
	std::cout << std::endl;

	MPI_Status* statusses = new MPI_Status[(world_size * 2) - 2];
	MPI_Request* req = new MPI_Request[(world_size * 2) - 2];

	int counter = 0;
	for (int s = 0; s < world_size; s++) {
	if (s != rank) {
	MPI_Isend(sendMap[s], 1, MPI_INT, s, 0, MPI_COMM_WORLD, &req[counter]);
	std::cout << "PROCESS " << rank << ", sent msg to " << s << std::endl;
	counter++;
	}

	}

	counter = 0;
	for (int r = 0; r < world_size; r++) {
	if (r != rank) {
	MPI_Irecv(recvMap[r], 1, MPI_INT, r, 0, MPI_COMM_WORLD, &req[(world_size - 1) + counter]);
	counter++;
	}
	}

	std::cout << "Und " << rank << " wartet";
	MPI_Waitall((world_size * 2) - 2, req, statusses);
	std::cout << "Und Prozess " << rank << " ist feddish" << std::endl;

	for (auto con : recvMap) {
	std::cout << "As Process " << rank << " from Process " << con.first << ", data received: " << *con.second << std::endl;
	}*/
}


