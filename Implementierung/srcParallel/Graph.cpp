/*############################################

Author: Rami Aly and Christoph Hueter
Date : 20.09.17
Libraries and Licences:
Boost Library 1.60, MPI Boost Library 1.60, Open Streetmaps andMPI in use.
All used Maps are licensed under the Open Street Maps License.

#############################################*/

#include "Graph.h"
#include <iostream>
#include <vector>
#include "VertexFileReader.h"
#include "EdgeFileReader.h"
#include "main.h"
#include "mpi.h"
#include <limits.h>

typedef std::vector<Vertex*> vertexContainer;
typedef std::vector<Spawner*> spawnerContainer;
typedef std::vector<Edge*> edgeContainer;


Graph::Graph()
{
	MPI_Comm_rank(MPI_COMM_WORLD, &_rank);

	_vertices = readVertexFile("../../dev/OutputMap/nodes");
	_edges = calculateEdges(_vertices, "../../dev/OutputMap/edges");
	connectVertices(_edges);
	initGraphProperties();
	_vertexMap = vertexMap(_vertices);
	_edgeMap = edgeMap(_edges);
}

std::vector<Spawner*> Graph::getSpawner()
{
	return _spawner;
}

Edge* Graph::getEdge(int edgeID) {
	return _edgeMap[edgeID];
}


std::vector<Edge*> Graph::getEdges()
{
	return _edges;
}

std::map<int, Vertex*> Graph::getVertexMap()
{
	return _vertexMap;
}


std::map<int, Spawner*> Graph::createSpawnerMap()
{
	std::map<int, Spawner*> spawner;
	for (spawnerContainer::iterator it = _spawner.begin(); it != _spawner.end(); it++) {
		spawner[(*it)->getID()] = (*it);
	}
	return spawner;
}

std::vector<int> Graph::createSpawnerIDVector()
{
	std::vector<int> spawner;
	for (int i = 0; i < _spawner.size(); i++) {
		spawner.push_back(_spawner[i]->getID());
	}
	return spawner;
}

float Graph::getMaxX()
{
	return _maxX;
}

float Graph::getMaxY()
{
	return _maxY;
}

std::vector<Vertex*> Graph::getVertices()
{
	return _vertices;
}

void Graph::initGraphProperties() {
	int maxX = INT_MIN;
	int maxY = INT_MIN;
	for (vertexContainer::iterator it = _vertices.begin(); it != _vertices.end(); it++) {
		//TODO Chnge method of choosing spawner, adding variable or sth to choose density etc...
		//Selection of spawner is currently static every vertex that is at the border of the graph
		if ((*it)->getEdges().size() == 2) {
			Spawner* spawner = new Spawner((*it)->getID(), (*it)->getX(), (*it)->getY());
			spawner->setIncomingEdges((*it)->getIncomingEdges());
			spawner->setOutigoingEdges((*it)->getOutgoingEdges());
			_spawner.push_back(spawner);
		}

		//Stores the highest x and y value of all vertices
		if ((*it)->getX() > maxX) {
			maxX = (*it)->getX();
		}
		if ((*it)->getY() > maxY) {
			maxY = (*it)->getY();
		}
	}
	_maxX = maxX;
	_maxY = maxY;
	createTrafficLights();
}

float Graph::distance(int vertex1, int vertex2, std::queue<int> route) {
	float distance = 0;
	int origin = vertex1;
	//Collect the sum length of all Edges on the route
	while (route.size() > 0) {
		distance += _vertexMap[origin]->outgoingNeighbor(route.front())->getLength();
		origin = route.front();
		if (route.front() == vertex2)  break;
		route.pop();
	}
	return distance;
}

void Graph::addWeightToTimeTables(int startID, int destID, int currentTimeTableIndex, std::queue<int> route) {
	//Copy route
	std::queue<int> tempqueue = route;
	//remove first element, because it is the origin
	tempqueue.pop();
	int origin = startID;
	float totaldistance = 0;
	while (tempqueue.size() > 0) {
		int tempgoal = tempqueue.front();
		//Used as a heuristik to approximate the time and as such the index of the timetable 
		totaldistance += distance(origin, tempgoal, tempqueue);
		std::pair<int, int> timetableValuePair = calculateTimetableValues(currentTimeTableIndex, totaldistance);
		//Add Value to the timetable at the calculated index
		_vertexMap[origin]->outgoingNeighbor(tempgoal)->addWeightTimetable(timetableValuePair.first, timetableValuePair.second);
		origin = tempqueue.front();
		if (tempqueue.front() == route.back()) return;
		tempqueue.pop();
	}
}

void Graph::createTrafficLights() {
	//Create TrafficLight for every vertex
	for (vertexContainer::iterator it = _vertices.begin(); it != _vertices.end(); it++) {
		edgeContainer inEdges = (*it)->getIncomingEdges();
		std::vector<std::pair<int, int>> tLMap;
		/*Traffic lights will always be green when there is only one one effektive direction and/or street.
		Otherwise handle situation for a crossing and T-Road
		*/
		if (inEdges.size() > 1) {
			tLMap.push_back(std::make_pair(inEdges[0]->getID(), inEdges[1]->getID()));
		}
		if (inEdges.size() == 4) {
			tLMap.push_back(std::make_pair(inEdges[2]->getID(), inEdges[3]->getID()));
		}
		if (inEdges.size() == 3) {
			tLMap.push_back(std::make_pair(inEdges[2]->getID(), inEdges[2]->getID()));
		}
		else if (inEdges.size() == 1) {
			tLMap.push_back(std::make_pair(inEdges[0]->getID(), inEdges[0]->getID()));
		}

		//nicht "wirklich" zufällig, soll aber reichen (Bei jeder durchführung wird gleiche generierungsufnktion
		//für zufallzahl genommen
		int start = rand() % TRAFFICLIGHT_DURATION;
		//Create TrafficLight and add it to the Vertex
		TrafficLight tl(tLMap, TRAFFICLIGHT_DURATION, start);
		(*it)->setTrafficLight(tl);
	}
}

std::pair<int, int> Graph::calculateTimetableValues(int intitialTimetableIndex, float totalDist) {
	return std::make_pair(intitialTimetableIndex + (totalDist / _CAR_SPEED_PER_TICK), _CAR_RELEVANCE);
}

int Graph::getSumWeightFromTimeTables(int startID, int destID, int currentTimeTableIndex, std::queue<int> route) {
	std::map<int, float> costs;
	std::queue<int> tempqueue = route;
	int origin = startID;
	//Remove first since the origin is stored in it
	tempqueue.pop();
	int timeTableValues = 0;
	float totaldistance = 0;
	while (tempqueue.size() > 0) {
		int tempgoal = tempqueue.front();
		//Calculate right index of Timetable
		totaldistance += distance(origin, tempgoal, tempqueue);
		//Get Value of index and add to sum
		timeTableValues += _vertexMap[origin]->outgoingNeighbor(tempgoal)->
			getWeightTimetable(currentTimeTableIndex
				+ (totaldistance / _CAR_SPEED_PER_TICK));
		origin = tempqueue.front();
		//reached the destination
		if (tempqueue.front() == route.back()) return timeTableValues;
		tempqueue.pop();
	}
	return timeTableValues;
}

float Graph::distance_heuristicOverID(size_t start, size_t goal) {
	//TODO Refactoring, already used in similar manner in Routingtable
	std::pair<float, float> korStart = _vertexMap[start]->getPosition();
	std::pair<float, float> korEnd = _vertexMap[goal]->getPosition();
	return sqrt(pow((korStart.first - korEnd.first), 2.0f) +
		pow((korStart.second - korEnd.second), 2.0f));
}

int Graph::getNumberVertices()
{
	return _vertices.size();
}

int Graph::getNumberEdges()
{
	return _edges.size();
}

/*
############################
		FOR PARALLEL
		SIMULATION
############################
*/

/*
	Initializes local vertex and edge vectors and maps
*/
void Graph::InitLocalVerticesEdges()
{
	//Get all spawners in a map
	std::map<int, Spawner*> globalSpawnerMap = createSpawnerMap();

	//id, vertex
	std::map<int, Vertex*> localVertexMap;
	//id, edge
	std::map<int, Edge*> localEdgeMap;

	std::vector<Edge*> localEdges;
	std::vector<Vertex*> localVertices;
	std::vector<Spawner*> localSpawners;

	//Go through every vertex of the map and filter all relevant local vertices and edges
	for (std::pair<const int, int> &processPair : _vertexProcessMap) {
		//If vertex is a local vertex
		if (_rank == processPair.second) {

			//Get vertex pointer and incoming edges attached to it
			Vertex* vertex = _vertexMap.find(processPair.first)->second;

			//Get only incoming edges
			std::vector<Edge*> incomingEdges = vertex->getIncomingEdges();

			//Iterate through edges attached to vertex
			for (std::vector<Edge*>::iterator edgeIt = incomingEdges.begin(); edgeIt != incomingEdges.end(); edgeIt++) {
				//Push edge into vector and map with ID
				localEdges.push_back(*edgeIt);
				localEdgeMap[(*edgeIt)->getID()] = *edgeIt;
			}

			//Map vertex int to vertex pointer
			localVertices.push_back(vertex);
			localVertexMap[processPair.first] = vertex;

			//If this vertex is a spawner add it to the spawner vector
			std::map<int, Spawner*>::iterator spawner = globalSpawnerMap.find(processPair.first);
			if (spawner != globalSpawnerMap.end()) {
				localSpawners.push_back((*spawner).second);
			}
		}
	}

	/*
		Assign each vertex/map
	*/
	_localVertexMap = localVertexMap;
	_localEdgeMap = localEdgeMap;

	_localVertices = localVertices;
	_localEdges = localEdges;
	_localSpawners = localSpawners;
}


void Graph::calculateVertexProcessMap()
{
	//Assign new map with resolved conflicts
	_vertexProcessMap = solveVertexProcessConflicts(_vertexProcessConflictMap);
}

//Insert vertexID and a process claiming this vertex
void Graph::insertVertexProcessPair(int vertexID, int processID) {
	_vertexProcessConflictMap[vertexID].push_back(processID);
}

//Assigns every Vertex a single processID
std::map<int, int> Graph::solveVertexProcessConflicts(std::map<int, std::vector<int>> vertexProcessesMap) {

	/*
	first int -> vertex
	second int -> process
	*/
	std::map<int, int> procVertexMap;

	//Check if vertex has already been assigned to a process before
	for (auto &conflictVertexMap : vertexProcessesMap) {
		//If a conflict is found find the process to claim this vertex
		procVertexMap[conflictVertexMap.first] = solveProcessConflict(conflictVertexMap.first, conflictVertexMap.second);
	}
	return procVertexMap;
}

int Graph::solveProcessConflict(int vertexID, std::vector<int> processes) {
	int world_size;
	MPI_Comm_size(MPI_COMM_WORLD, &world_size);

	std::sort(processes.begin(), processes.end());
	processes.erase(std::unique(processes.begin(), processes.end()), processes.end());

	int value = vertexID % processes.size();

	//return *std::max_element(processes.begin(), processes.end());
	return processes[value];
}

/*
	Returns connections used in Simulation.cpp
	first map -> incoming
	second map -> outgoing
*/
std::pair<std::map<int, std::vector<int>>, std::map<int, std::vector<int>>> Graph::getProcessConnectionVectors()
{
	//first int -> processID, second int -> edgeID
	std::map<int, std::vector<int>> incomingEdges;
	std::map<int, std::vector<int>> outgoingEdges;

	//## Check vertices and assign each edge to a processID ##

	//Go through vertexMap of the local process and add the edges attached to them 
	for (std::pair<const int, Vertex*> &vertexPair : _localVertexMap) {

		std::vector<Edge*> out = vertexPair.second->getOutgoingEdges();

		//Get every outgoing edge of current vertex
		for (Edge* &edge : out) {
			//Get ID and Process of Vertex
			int endVertexID = edge->getVertices().second->getID();
			int endVertexProcess = _vertexProcessMap[endVertexID];

			//VERTEX IN DIFFERENT PROCESS
			if (endVertexProcess != _rank) {

				int reverseEdgeID = vertexPair.second->incomingNeighbor(endVertexID)->getID();

				//Add both edges of the same street between the two vertices to the correct vector
				outgoingEdges[endVertexProcess].push_back(edge->getID());
				incomingEdges[endVertexProcess].push_back(reverseEdgeID);
			}
		}
	}

	std::cout << _rank << ": got incoming and outgoing edges" << std::endl;

	//Sort vectors in ascending order
	for (auto &v : incomingEdges) {
		std::sort(v.second.begin(), v.second.end(), std::less<int>());
	}

	//Sort vectors in ascending order
	for (auto &v : outgoingEdges) {
		std::sort(v.second.begin(), v.second.end(), std::less<int>());
	}

	std::cout << _rank << ", globalSpawners: " << _spawner.size() << std::endl;

	return std::make_pair(incomingEdges, outgoingEdges);
}

std::pair<std::map<int, Vertex*>, std::map<int, Edge*>> Graph::getLocalVertexEdgeMaps()
{
	return std::make_pair(_localVertexMap, _localEdgeMap);
}

std::pair<std::vector<Vertex*>, std::vector<Edge*>> Graph::getLocalVerticesEdges()
{
	return std::make_pair(_localVertices, _localEdges);
}

std::vector<Spawner*> Graph::getLocalSpawners()
{
	return _localSpawners;
}









