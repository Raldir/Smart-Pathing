#include "Simulation.h"
#include <iostream>
#include <fstream>
#include "mpi.h"

typedef std::vector<Edge*> edgeContainer;
typedef std::vector<Spawner*> spawnerContainer;
typedef std::vector<Vertex*> vertexContainer;

Simulation::Simulation()
{
	int rank;
	int root = 0;

	InitVectors();

	MPI_Comm_rank(MPI_COMM_WORLD, &rank);

	_currentTick = 0;

	_graph = new Graph();
	_routingTable = new RoutingTable(_graph, 3);
	initSpawner();
	std::cout << "Init completed";
	for (int i = 0; i < _SIMULATION_TICKS; i++) {
		std::cout << rank << " came to barrier on tick " << i << std::endl;
		MPI_Barrier(MPI_COMM_WORLD);
		_currentTick++;
		nextTick();

		writeResultsCurrentTick();
	}
}


Simulation::~Simulation()
{
}

void Simulation::writeResultsCurrentTick()
{
	std::ofstream results;
	edgeContainer edges = _graph->getEdges();
	results.open("../../Output/" + std::to_string(_currentTick) + ".txt");
	for (edgeContainer::iterator it2 = edges.begin(); it2 != edges.end(); it2++) {
		results << (*it2)->getID() << " " << (*it2)->numberOfCars() << " " << (*it2)->getEdgeCapacity() << '\n';
	}
	results.close();
}


void Simulation::nextTick()
{
	//VON CHRISTOPH
	int timeStamp = _currentTick %_TIMETABLE_SPAN;
	std::cout << "begin update" << '\n';
	vertexContainer vertices = _graph->getVertices();
	for (vertexContainer::iterator it2 = vertices.begin(); it2 != vertices.end(); it2++) {
		(*it2)->Update();
	}

	std::cout << "Vertex update completed" << '\n';
	for (Edge* ed : _graph->getEdges()) {
		ed->Update(_currentTick);
		//Methode zum Testen
		//ed->printCars();
	}

	//########### PARALLEL ###########

	//### FREE SPACE MSG ###

	//Prepare send buffer for free space of edges
	fillEdgeSpaceSendBuffer();

	//Send and receive messages in buffer
	exchangeEgdeFreeSpace();



	//Sending information about cars transitioning to different process
	for (auto process : outgoingConnections) {
		//TODO Encode car information into an array of ints
	}

	MPI_Barrier(MPI_COMM_WORLD);

	for (auto process : incomingConnections) {
		
		MPI_Status status;
		int recvBufferLength;

		MPI_Probe(process.first, 1, MPI_COMM_WORLD, &status);
		MPI_Get_count(&status, MPI_INT, &recvBufferLength);
		
		//Reserve car buffer
		std::vector<int> v;
		v.resize(recvBufferLength);
		carRecvBuffer[process.first] = &v;

		//Actual receive of car information
		MPI_Irecv(&carRecvBuffer, recvBufferLength, MPI_INT, process.first, 1, MPI_COMM_WORLD, NULL);
	}

	//########### PARALLEL ###########

	std::cout << "Edge update Phase 1 completed" << '\n';
	std::vector<Edge*> remainingEdges = _graph->getEdges();
	for (int i = 0; i < _graph->getEdges().size(); i++) {
		for (edgeContainer::iterator it2 = remainingEdges.begin(); it2 != remainingEdges.end();) {
			(*it2)->UpdateOverflow();
			//(*it2)->printCars();
			if (!(*it2)->hasOverflow()) {
				it2 = remainingEdges.erase(it2);
			}
			else it2++;
		}
	}

	std::cout << "Edge update Phase 2 completed" << '\n';
	spawnerContainer spawners = _graph->getSpawner();
	for (spawnerContainer::iterator it2 = spawners.begin(); it2 != spawners.end(); it2++) {
		//std::cout << "hello";
		//VON CHRISTOPH --> gibt tick an update weiter
		(*it2)->Update(timeStamp);
	}
	std::cout << "Tick " << _currentTick << "finished" << '\n';
}

void Simulation::initSpawner() {
	std::vector<Spawner*> spawner = _graph->getSpawner();
	std::vector<std::pair<Spawner*, int>> vertexPriorities;
	for (std::vector<Spawner*>::iterator it2 = spawner.begin(); it2 != spawner.end(); it2++) {
		vertexPriorities.push_back(std::pair<Spawner*, int>((*it2), rand() % VERTEX_PRIORITY_DIVERGENCE));
	}
	for (std::vector<Spawner*>::iterator it2 = spawner.begin(); it2 != spawner.end(); it2++) {
		(*it2)->linkRoutingTable(_routingTable);
		(*it2)->linkVertexPriorities(vertexPriorities);
	}
}

/*
	Fill the send buffer which contains the free space of edges with edges sorted in ascending order
*/
void Simulation::fillEdgeSpaceSendBuffer() {

	/*

	//Fill sending buffer for every process (con.first)
	for (std::pair<int, std::vector<int>> outCon : outgoingConnections) {

		//Go through the vector of edges and get the amount of free space in them
		for (int edge = 0; edge < outCon.second.size(); edge++) {

			//Get free space amount from edge (must be in the same process)
			int freeSpaceAmount = _graph->getEdge(outCon.second[edge])->getFreeSpaceAmount();

			//Push number into buffer at right place
			edgeSpaceSendBuffer[outCon.first]->push_back(freeSpaceAmount);
		}
	}*/
}

void Simulation::exchangeEgdeFreeSpace() {

	int requestCounter = 0;
	//ISend free space amount to every process
	for (auto process : edgeSpaceSendBuffer) {
		//Send buffer, count, type, dest, tag, comm, request
		MPI_Isend(process.second, process.second->size(), MPI_INT, process.first, 0, MPI_COMM_WORLD, &sendReq[requestCounter]);
		requestCounter++;
	}

	requestCounter = 0;
	//Receive free space amount of every edge in connected processes
	for (auto process : edgeSpaceRecvBuffer) {
		//Recv buffer, count, type, dest, tag, comm, request
		MPI_Irecv(process.second, process.second->max_size(), MPI_INT, process.first, 0, MPI_COMM_WORLD, &recvReq[requestCounter]);
		requestCounter++;
	}

	//Wait for ISends and IRecv to finish 
	MPI_Status *recvStatus = new MPI_Status[incomingConnections.size()];
	MPI_Status *sendStatus = new MPI_Status[incomingConnections.size()];
	MPI_Waitall(outgoingConnections.size(), sendReq, sendStatus);
	MPI_Waitall(incomingConnections.size(), recvReq, recvStatus);
}

void Simulation::InitVectors()
{
	//Prepare receive buffer for every edge from every process
	for (auto in : incomingConnections) {

		//Vector that will receive space of edges in a single process connected to this process
		std::vector<int> edgeSpacesRecv;

		//Set vector size to amount of edges
		int edgeAmount = in.second.size();
		edgeSpacesRecv.resize(edgeAmount);

		//Assign vector of edge spaces for this process
		edgeSpaceRecvBuffer[in.first] = &edgeSpacesRecv;
	}

	//Prepare vector buffers for outgoing connection
	for (auto out : outgoingConnections) {

		//Vector that will send space of edges in this process for another process
		std::vector<int> edgeSpaceSend;

		//Set vector size to amount of edges;
		int edgeAmount = out.second.size();
		edgeSpaceSend.resize(edgeAmount);

		//Assign vector of edge spaces for this process
		edgeSpaceSendBuffer[out.first] = &edgeSpaceSend;
	}
}


