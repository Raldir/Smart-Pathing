#include "Simulation.h"
#include <iostream>
#include <fstream>
#include "mpi.h"
#include <ctime>
#include "Car.h"


typedef std::vector<Edge*> edgeContainer;
typedef std::vector<Spawner*> spawnerContainer;
typedef std::vector<Vertex*> vertexContainer;

Simulation::Simulation(int world_size, int rank)
{
	int root = 0;

	InitEdgeFreeSpaceBuffers();

	MPI_Comm_rank(MPI_COMM_WORLD, &rank);

	_currentTick = 0;
	_rank = rank;
	_world_size = world_size;
	_graph = new Graph();
	clock_t begin = clock();
	parallelRouting();
	clock_t end = clock();
	double elapsed_secs = double(end - begin) / CLOCKS_PER_SEC;
	std::cout << "Gesamte Zeit: " << elapsed_secs << std::endl;

	//initSpawner();
	/*for (int i = 0; i < _SIMULATION_TICKS; i++) {
		std::cout << rank << " came to barrier on tick " << i << std::endl;
		_currentTick++;
		nextTick();

		writeResultsCurrentTick();
	}*/
}


void Simulation::parallelRouting() {

	//TODO Besser: Alle zu allen senden;
	std::vector<int> ids = _graph->createSpawnerIDVector();
	//std::cout << ids.size() << std::endl;
	//system("Pause");
	int* send_buf = &ids[0];
	std::vector<int> splitting = splitGraphSize(_world_size);
	int* send_cnt = &splitting[0];
	//for (int i = 0; i < splitting.size(); i++) {
	//	std::cout << splitting[i] << " ";
	//}
	std::cout << std::endl;
	int* displays = new int[_world_size];
	int recv_cnt = send_cnt[_rank];
	int* recv_buf = new int[recv_cnt];
	std::vector<int> location;
	//std::cout << ( sizeof(*recv_buf)) << std::endl;
	location = splitGraphLocation(splitting);
	displays = &location[0];
	MPI_Scatterv(send_buf, send_cnt, displays, MPI_INT, recv_buf, recv_cnt, MPI_INT, 0, MPI_COMM_WORLD);
	std::cout << recv_cnt << std::endl;
	/*for (int i = 0; i < recv_cnt; i++) {
		std::cout << recv_buf[i] << " ";
	}*/
	//std::cout<<std::endl;
	std::vector<int> v(recv_buf, recv_buf + recv_cnt);
	RoutingTable routingTable(_graph, _NEAREST_NEIGHBOR, v);
	std::vector<std::vector<int>> matrix = routingTable.getRoutingMatrix();
	std::vector<std::vector<int>> matrixK_nn = routingTable.getKNearestMatrix();
	executeGatherRouting(matrix, displays, send_cnt, _graph->getNumberVertices(), _graph->getSpawner().size() - 1);

	//EVERY PROCESS CALCULATES THE MATRIX BASED ON THE RECEIVING VALUES OF THE OTHER PROCESSES
	std::map<int, std::map<int, std::queue<int>>> routingTableC;
	int lastC = 0;
	for (int i = 0; i < _world_size; i++) {
		std::queue<int> route;
		//GEGEBENNENFALLS IST HIER DER INDEX UM 1 ZU KLEIN UND schneidet einen knoten ab
		//Normalerweise sollte man immer ienen über haben, da anzahl der knoten die gesamtzahl istu n man nicht einen Kreis fährt.
		int last = 0;

		for (int j = 0; j < receive_elementC[i]; j++) {
			if (receive_buf[j + lastC] == -1 && last != -1) {
				routingTableC[route.front()][route.back()] = route;
				clear(route);
				last = -1;
			}
			else if (receive_buf[j + lastC] == -1) {
				last = receive_buf[j + lastC];
			}
			else {
				route.push(receive_buf[j + lastC]);
				last = receive_buf[j + lastC];
			}
		}
		lastC += receive_elementC[i];
	}



	std::vector<std::vector<int>> matrixC = routingTable.getRoutingCosts();
	int totalRow = _graph->getSpawner().size();
	executeGatherRouting(matrixC, displays, send_cnt, 3, totalRow);
	/*for (int i = 0; i < receive_elementC[_rank]; i++) {
		std::cout << receive_buf[i] << " ";
		if (i % 3 == 0 && i !0) std::cout << std::endl;
	}*/
	std::cout << std::endl;
	std::map<int, std::map<int, float>> routingCosts;
	lastC = 0;
	for (int i = 0; i < _world_size; i++) {
		std::vector<int> route;
		for (int j = 0; j < receive_elementC[i]; j++) {
			if (route.size() < 3) {
				route.push_back(receive_buf[j + lastC]);
			}
			else {
				//std::cout << route.front() << " " << route.back() << std::endl;
				routingCosts[route.front()][route.back()] = float(route[1]);
				route.clear();
				j--;
			}
		}
		lastC += receive_elementC[i];
	}
	_routingTable = new RoutingTable(_graph, _NEAREST_NEIGHBOR, routingTableC, routingCosts);
	std::queue<int> empty;
	for (int i = 0; i < ids.size(); i++) {
		_routingTable->insertRoute(ids[i], ids[i], empty);
	}
	//momentan sequentiell
	_routingTable->calculateKNearest();
}

void Simulation::executeGatherRouting(std::vector<std::vector<int>> matrix, int* displays, int*splitting, int cols, int totalRows) {
	int rows = matrix.size();
	int** routes_arr = setupHMM(matrix, rows, cols);
	int num_elem = rows * cols;
	receive_buf = new int[cols * totalRows * (totalRows)* splitting[_rank]];
	receive_elementC = new int[_world_size];

	receive_displs = new int[_world_size];
	//std::cout << cols * totalRows * (totalRows - 1) << " " << splitting[_rank] * (totalRows - 1) * cols<<std::endl;
	for (int i = 0; i < _world_size; i++) {
		//Jedes Receivte Stück besteht aus insgesamt Anzahl der zugewiesenen Spawners mult. mit Anzahl aller Spawner mult. mit Länge der max. Route
		receive_elementC[i] = splitting[i] * totalRows * cols;
		//std::cout << displays[i] << " " << _rank << std::endl;
		receive_displs[i] = displays[i] * totalRows * cols;
	}

	for (int i = 0; i < _world_size; i++) {
		MPI_Gatherv(&(routes_arr[0][0]), num_elem, MPI_INT, receive_buf, receive_elementC, receive_displs, MPI_INT, i, MPI_COMM_WORLD);
	}
}



void Simulation::clear(std::queue<int> &q)
{
	std::queue<int> empty;
	std::swap(q, empty);
}

int Simulation::getMaxCol(std::vector < std::vector<int>> &vals) {
	int max = INT_MIN;
	for (int i = 0; i < vals.size(); i++) {
		for (int j = 0; j < vals[i].size(); j++) {
			if (vals[i][j] > max) max = vals[i][j];
		}
	}
	return max;
}

int** Simulation::setupHMM(std::vector<std::vector<int>> &vals, int N, int M)
{
	int *buffer = new int[M*N];
	int **temp = new int*[N];
	for (int i = 0; i < N; ++i)
		temp[i] = buffer + i*M;
	//int** temp;
	//temp = new int*[N];
	for (unsigned i = 0; (i < N); i++)
	{
		for (unsigned j = 0; (j < M); j++)
		{
			//Stroes -1, in case the vector is smaller than the largest array
			if (j >= vals[i].size()) temp[i][j] = -1;
			else temp[i][j] = vals[i][j];
		}
	}
	return temp;
}


std::vector<int> Simulation::splitGraphSize(int numberProcesses) {
	std::vector<Spawner*> spawners = _graph->getSpawner();
	std::vector<int> splitedGraphSize;
	//system("Pause");
	int eachSpawnerNumber = int(spawners.size() / numberProcesses);
	int overflow = spawners.size() - eachSpawnerNumber * numberProcesses;
	for (int j = 0; j < numberProcesses; j++) {
		int margin = eachSpawnerNumber;
		if (overflow > 0) {
			margin = eachSpawnerNumber + 1;
			overflow--;
		}
		splitedGraphSize.push_back(margin);
	}
	return splitedGraphSize;
}

std::vector<int> Simulation::splitGraphLocation(std::vector<int> buffer) {
	std::vector<int> splitedGraphLocation;
	int count = 0;
	for (int j = 0; j < buffer.size(); j++) {
		splitedGraphLocation.push_back(count);
		count += buffer[j];
	}
	return splitedGraphLocation;
}

std::vector<std::vector<int>> Simulation::splitGraph(int numberProcesses) {
	std::vector<Spawner*> spawners = _graph->getSpawner();
	std::vector<std::vector<int>> splitedGraph;
	int eachSpawnerNumber = int(spawners.size() / numberProcesses);
	int overflow = spawners.size() - eachSpawnerNumber * numberProcesses;
	for (int i = 0; i < numberProcesses; i++) {
		int margin = eachSpawnerNumber;
		if (overflow > 0) margin = eachSpawnerNumber + 1;
		for (int j = 0; j < margin; j++) {
			splitedGraph[i].push_back(spawners[eachSpawnerNumber * i + j]->getID());
			overflow--;
		}
	}
	return splitedGraph;
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
	int timeStamp = _currentTick %_TIMETABLE_SPAN;
	std::cout << "begin update" << '\n';
	vertexContainer vertices = _graph->getVertices();

	//Update Vertices (TrafficLight)
	for (vertexContainer::iterator it2 = vertices.begin(); it2 != vertices.end(); it2++) {
		(*it2)->Update();
	}

	//########### PARALLEL ###########

	//Prepare send buffer for free space of edges
	fillEdgeSpaceSendBuffer();
	//Send and receive messages in buffer
	exchangeEgdeFreeSpace();

	//First Update
	std::cout << "Vertex update completed" << '\n';
	for (Edge* ed : _graph->getEdges()) {
		ed->Update(_currentTick);
	}

	//SENDING CAR INFORMATION
	sendCarInformation();
	receiveCarInformation();

	std::cout << "Edge update Phase 1 completed" << '\n';
	std::vector<Edge*> remainingEdges = _graph->getEdges();
	for (int i = 0; i < _graph->getEdges().size(); i++) {

		//After every Update, exchange the space
		fillEdgeSpaceSendBuffer();
		exchangeEgdeFreeSpace();

		for (edgeContainer::iterator it2 = remainingEdges.begin(); it2 != remainingEdges.end();) {
			(*it2)->UpdateOverflow();
			if (!(*it2)->hasOverflow()) {
				it2 = remainingEdges.erase(it2);
			}
			else it2++;
		}

		sendCarInformation();
		receiveCarInformation();
	}

	//After every Update, exchange the space
	fillEdgeSpaceSendBuffer();
	exchangeEgdeFreeSpace();

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

	//Fill sending buffer for every process (con.first)
	for (auto &outCon : outgoingConnections) {

		//Go through the vector of edges and get the amount of free space in them
		for (std::vector<int>::iterator it = outCon.second.begin(); it != outCon.second.end(); it++) {

			//Get free space amount from edge (must be in the same process)
			int freeSpaceAmount = _graph->getEdge(outCon.second[*it])->getFreeSpaceAmount();
			//Push free space amount into the corresponding place inside buffer
			edgeSpaceSendBuffer[outCon.first][*it] = freeSpaceAmount;
		}
	}
}

void Simulation::exchangeEgdeFreeSpace() {

	reqCounter = 0;
	//ISend free space amount to every process
	for (auto &process : edgeSpaceSendBuffer) {
		//Send buffer, count, type, dest, tag, comm, request
		MPI_Isend(process.second, outgoingConnections[process.first].size(), MPI_INT, process.first, 0, MPI_COMM_WORLD, &req[reqCounter]);
		reqCounter++;
	}

	//Receive free space amount of every edge in connected processes
	for (auto &process : edgeSpaceRecvBuffer) {
		//Recv buffer, count, type, dest, tag, comm, request
		MPI_Irecv(process.second, incomingConnections[process.first].size(), MPI_INT, process.first, 0, MPI_COMM_WORLD, &req[reqCounter]);
		reqCounter++;
	}

	//Wait for ISends and IRecv to finish 
	MPI_Status *status = new MPI_Status[incomingConnections.size() + outgoingConnections.size()];
	MPI_Waitall(outgoingConnections.size() + incomingConnections.size(), req, status);
}

void Simulation::sendCarInformation() {

	/*
		Data to be send
		first int -> processID
		second int -> edgeID
		vector -> cars that want to transition to the edge and process
	*/
	std::map<int, std::map<int, std::vector<Car*>>> send;

	//Counter for every buffer
	std::map<int, int> bufferSizeMap;

	//Gather information and size for the buffer
	for (auto &process : outgoingConnections) {

		//Counter for size of carSendBuffer
		int size = 0;

		//first int -> edgeID, vector -> cars
		std::map<int, std::vector<Car*>> edgeBuffer;
		//Go through every edge connected with that process
		for (int &edgeID : process.second) {

			//Get vector of cars for edge from start Vertex
			std::vector<Car*> cars = _graph->getEdge(edgeID)->getVertices().first->popTransitioningCars(edgeID);

			//Go through every car and increase counter for buffer, including the route of every car
			for (Car* car : cars) {
				//4 --> edge, overflowPosition, distanceTravelled und -1 als Begrenzung
				size += 3 + car->getRoute().size() + 1;
			}

			//Assign vector of cars for edge to send buffer
			edgeBuffer[edgeID] = cars;
		}

		bufferSizeMap[process.first] = size;
		send[process.first] = edgeBuffer;
	}

	//Extract information from cars into sendBuffer
	for (auto &process : send) {

		//Delete and reassign sendBuffer
		delete carSendBuffer[process.first];
		carSendBuffer[process.first] = new int[bufferSizeMap[process.first]];

		//Place where information is read inside buffer for this process
		int bufferCounter = 0;

		//Put car information into buffer
		for (auto edgeMap : process.second) {
			//Go through every car and extract information
			for (Car* car : edgeMap.second) {
				//ID of Edge
				carSendBuffer[process.first][bufferCounter] = edgeMap.first;
				//Overflow Position of Car
				carSendBuffer[process.first][++bufferCounter] = car->getOverflow();
				//Distance Travelled
				carSendBuffer[process.first][++bufferCounter] = car->getDistanceTravelled();

				std::queue<int> queue = car->getRoute();
				//Feed route of car into buffer
				for (int vertex = 0; vertex < queue.size(); vertex++) {
					carSendBuffer[process.first][++bufferCounter] = queue.front();

					queue.pop();
				}
				//Set stopping point for this car in buffer
				carSendBuffer[process.first][++bufferCounter] = -1;
			}
		}
	}

	//Set global reqCounter to zero
	reqCounter = 0;

	for (auto process : carSendBuffer) {
		MPI_Isend(process.second, bufferSizeMap[process.first], MPI_INT, process.first, 1, MPI_COMM_WORLD, &req[reqCounter++]);
	}
}

void Simulation::receiveCarInformation() {

	std::map<int, int> recvBufferLength;

	for (auto &con : incomingConnections) {

		MPI_Status status;
		MPI_Probe(con.first, 1, MPI_COMM_WORLD, &status);
		//Get size of message
		MPI_Get_count(&status, MPI_INT, &recvBufferLength[con.first]);

		delete carRecvBuffer[con.first];
		carRecvBuffer[con.first] = new int[recvBufferLength[con.first]];

		MPI_Irecv(carRecvBuffer[con.first], recvBufferLength[con.first], MPI_INT, con.first, 1, MPI_COMM_WORLD, &req[reqCounter++]);
	}

	//Read buffer and store it in new car objects again
	for (auto &process : carRecvBuffer) {

		int counter = 0;
		//Go through every int in the buffer
		while (counter < recvBufferLength[process.first]) {

			//Parse through the array of ints
			int* buffer = process.second;

			int edgeID = buffer[counter++];
			int overflowPosition = buffer[counter++];
			int distanceTravelled = buffer[counter++];

			int currentID = 0;
			std::queue<int> route;

			//Get the route until a -1 is read
			while (currentID != -1) {
				currentID = buffer[counter++];
				route.push(currentID);
			}

			//Create car with parameters and push it on edge
			Car* car = new Car(overflowPosition, distanceTravelled, route);
			_graph->getEdge(edgeID)->pushCar(car);
		}
	}
}

void Simulation::InitEdgeFreeSpaceBuffers() {

	//Prepare receive buffer for every edge from every process
	for (auto in : incomingConnections) {

		//Set vector size to amount of edges
		int edgeAmount = in.second.size();

		//Assign vector of edge spaces for this process
		edgeSpaceRecvBuffer[in.first] = new int[edgeAmount];
	}

	//Prepare vector buffers for outgoing connection
	for (auto out : outgoingConnections) {

		//Set vector size to amount of edges;
		int edgeAmount = out.second.size();

		//Assign vector of edge spaces for this process
		edgeSpaceSendBuffer[out.first] = new int[edgeAmount];
	}
}

void Simulation::InitConnections(std::map<int, int> vertexVector)
{
	std::vector<int> incoming;
	std::vector<int> outgoing;
}

int Simulation::getEnd(std::queue<int> route) {
	int temp = 0;
	for (int i = 0; i < route.size(); i++) {
		if (route.front() == -1) {
			return temp;
		}
		else {
			temp = route.front();
			route.pop();
		}
	}
	return temp;
}
