#include "Graph.h"
#include <iostream>
#include <vector>
#include "VertexFileReader.h"
#include "EdgeFileReader.h"



using namespace boost;
typedef std::vector<Vertex*> vertexContainer;
typedef std::vector<Spawner*> spawnerContainer;


Graph::Graph()
{
	_vertices = readVertexFile("nodes");
	_edges = calculateEdges(_vertices, "edges");
	connectVertices(_edges);
	initGraphProperties();
	_vertexMap = vertexMap(_vertices);
	_edgeMap = edgeMap(_edges);
}

std::vector<Spawner*> Graph::getSpawner()
{
	return _spawner;
}

std::vector<Edge*> Graph::getEdges()
{
	return _edges;
}

std::map<int, Vertex*> Graph::getVertexMap()
{
	return _vertexMap;
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
		//TODO Wert nicht statisch setzen sondern dynamisch bestimmen. Veilleicht später die n wenigsten verbundenen Knoten nehmen
		if ((*it)->getEdges().size() == 2) {
			_spawner.push_back(new Spawner((*it)->getID(), (*it)->getX(), (*it)->getY()));
		}

		if ((*it)->getX() > maxX) {
			maxX = (*it)->getX();
		}
		if ((*it)->getY() > maxY) {
			maxY = (*it)->getY();
		}
		//	std::vector<Edge*> edges = (*it)->getEdges();
		//	//TODO Laufzeit ist katastrophal, bessere Lösung überlegen
		//	int edgecount = edges.size();
		//	for (std::vector<Edge*>::iterator it2 = edges.begin(); it2 != edges.end(); it2++){
		//		for (std::vector<Edge*>::iterator it3 = edges.begin(); it3 != edges.end(); it3++) {
		//			if ((*it2)->getVertices().first->getID() == (*it3)->getVertices().second->getID()) {
		//				edgecount --;
		//				break;
		//			}
		//		}
		//	}
	}
	_maxX = maxX;
	_maxY = maxY;
}

float Graph::distance(int vertex1, int vertex2, std::queue<int> route) {
	float distance = 0;
	int origin = vertex1;
	route.pop();
	while (route.size() > 0) {
		distance +=_vertexMap[origin]->outgoingNeighbor(route.front())->getLength();
		origin = route.front();
		if (route.front() == vertex2)  break;
		route.pop();
	}
	return distance;
}

void Graph::addWeightToTimeTables(int startID, int destID, int currentTimeTableIndex, std::queue<int> route) {
	std::queue<int> tempqueue = route;
	//remove first element
	int origin = startID;
	float totaldistance = 0;
	while (tempqueue.size() > 0) {
		int tempgoal = tempqueue.front();
		totaldistance += distance(origin, tempgoal, route);
		_vertexMap[origin]->outgoingNeighbor(tempgoal)->addWeightTimetable(currentTimeTableIndex
			+ (totaldistance / _CAR_SPEED_PER_TICK), _CAR_RELEVANCE);
		origin = tempqueue.front();
		tempqueue.pop();
	}
}

int Graph::getSumWeightFromTimeTables(int startID, int destID, int currentTimeTableIndex, std::queue<int> route) {
	std::map<int, float> costs;
	std::queue<int> tempqueue = route;
	int origin = startID;
	//tempqueue.pop();
	int timeTableValues = 0;
	float totaldistance = 0;
	while (tempqueue.size() > 0) {
		int tempgoal = tempqueue.front();
		totaldistance += distance(origin, tempgoal, route);
		timeTableValues += _vertexMap[origin]->outgoingNeighbor(tempgoal)->
			getWeightTimetable(currentTimeTableIndex
				+ (totaldistance / _CAR_SPEED_PER_TICK));
			origin = tempqueue.front();
			tempqueue.pop();
	}
	return timeTableValues;
}

float Graph::distance_heuristic2(size_t start, size_t goal) {
	std::pair<float, float> korStart = _vertexMap[start]->getPosition();
	std::pair<float, float> korEnd = _vertexMap[goal]->getPosition();
	return sqrt(pow((korStart.first - korEnd.first), 2.0f) +
		pow((korStart.second - korEnd.second), 2.0f));
}
