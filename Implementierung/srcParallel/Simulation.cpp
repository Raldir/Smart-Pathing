#include "Simulation.h"
#include <iostream>
#include <fstream>
#include "mpi.h"

typedef std::vector<Edge*> edgeContainer;
typedef std::vector<Spawner*> spawnerContainer;
typedef std::vector<Vertex*> vertexContainer;

Simulation::Simulation(int world_size, int rank)
{
	_currentTick = 0;
	_rank = rank;
	_world_size = world_size;
	_graph = new Graph();
	//std::cout << _rank<<std::endl;
	parallelRouting();
	system("Pause");
	initSpawner();
	for (int i = 0; i < _SIMULATION_TICKS; i++) {
		nextTick();
		writeResultsCurrentTick();
	}

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
	for (int i = 0; i < recv_cnt; i++) {
		std::cout << recv_buf[i] << " ";
	}
	//std::cout<<std::endl;
	std::vector<int> v(recv_buf, recv_buf + recv_cnt);
	RoutingTable routingTable(_graph, _NEAREST_NEIGHBOR, v);
	std::vector<std::vector<int>> matrix = routingTable.getRoutingMatrix();
	//for (int i = 0; i < matrix.size(); i++) {
	//	std::cout << matrix[i][0] << " " << _rank<<std::endl;
	//}
	int rows = matrix.size();
	int cols = _graph->getNumberVertices();
	int** routes_arr = setupHMM(matrix, rows, cols);
	//std::cout << cols << " "<< rows << std::endl;
	int num_elem = rows * cols;

	int totalRows = _graph->getSpawner().size();
	int* receive_buf = new int[cols * totalRows * (totalRows - 1) * splitting[_rank]];
	int* receive_elementC = new int[_world_size];

	int* receive_displs = new int[_world_size];
	//std::cout << cols * totalRows * (totalRows - 1) << " " << splitting[_rank] * (totalRows - 1) * cols<<std::endl;
	for (int i = 0; i < _world_size; i++) {
		//Jedes Receivte Stück besteht aus insgesamt Anzahl der zugewiesenen Spawners mult. mit Anzahl aller Spawner mult. mit Länge der max. Route
		receive_elementC[i] = splitting[i] * (totalRows - 1) * cols;
		//std::cout << displays[i] << " " << _rank << std::endl;
		receive_displs[i] = displays[i] * (totalRows - 1) * cols;
	}
	//if (_rank == 0) {
	//	for (int i = 0; i < splitting.size(); i++) {
	//		//Jedes Receivte Stück besteht aus insgesamt Anzahl der zugewiesenen Spawners mult. mit Anzahl aller Spawner mult. mit Länge der max. Route
	//		receive_elementC[i] = splitting[i] * (totalRows - 1) * cols;
	//		receive_displs[i] = displays[i] * (totalRows - 1) * cols;
	//	}
	//}

	//MPI_Gatherv(&(routes_arr[0][0]), num_elem, MPI_INT, receive_buf, receive_elementC, receive_displs, MPI_INT, 0, MPI_COMM_WORLD);
	for (int i = 0; i < _world_size; i++) {
		MPI_Gatherv(&(routes_arr[0][0]), num_elem, MPI_INT, receive_buf, receive_elementC, receive_displs, MPI_INT, i, MPI_COMM_WORLD);
	}
	std::cout << receive_displs[_rank] << " " << _rank << std::endl;
	//for (int i = 0; i < receive_elementC[_rank]; i++) {
	//	std::cout << receive_buf[i] << std::endl;
	//}
	std::map<int, std::map<int, std::queue<int>>> routingTableC;
	int lastC = 0;
	if (_rank != -1) {
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
		_routingTable = new RoutingTable(_graph, _NEAREST_NEIGHBOR, routingTableC);
		std::vector<std::vector<int>> matrix2 = _routingTable->getRoutingMatrix();
		for (int i = 0; i < matrix2.size(); i++) {
			for (int j = 0; j < matrix2[i].size(); j++) {
				std::cout << matrix2[i][j] << " ";
			}
			std::cout << std::endl;
		}
	}
	//if (_rank == 0) {
	//	std::map<int, std::map<int, std::queue<int>>> routingTableC;
	//	for (int i = 0; i < splitting.size(); i++) {
	//		std::queue<int> route;
	//		//GEGEBENNENFALLS IST HIER DER INDEX UM 1 ZU KLEIN UND schneidet einen knoten ab
	//		//Normalerweise sollte man immer ienen über haben, da anzahl der knoten die gesamtzahl istu n man nicht einen Kreis fährt.
	//		int last = 0;
	//		for (int j = 0; j < receive_elementC[i]; j++) {
	//			if (receive_buf[j] == -1 && last != -1) {
	//				routingTableC[route.front()][route.back()] = route;
	//				clear(route);
	//				last = -1;
	//			}
	//			else if (receive_buf[j] == -1) {
	//				last = receive_buf[j];
	//			}
	//			else {
	//				route.push(receive_buf[j]);
	//				last = receive_buf[j];
	//			}
	//		}
	//	}
	//	_routingTable = new RoutingTable(_graph, _NEAREST_NEIGHBOR, routingTableC);
	//}
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
	//VON CHRISTOPH
	_currentTick++;
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
