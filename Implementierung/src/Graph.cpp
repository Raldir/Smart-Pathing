#include "Graph.h"
#include <iostream>
#include <vector>
#include "VertexFileReader.h"
#include "EdgeFileReader.h"
#include "main.h"



using namespace boost;
typedef std::vector<Vertex*> vertexContainer;
typedef std::vector<Spawner*> spawnerContainer;
typedef std::vector<Edge*> edgeContainer;


Graph::Graph()
{
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
			Spawner* spawner = new Spawner((*it)->getID(), (*it)->getX(), (*it)->getY());
			spawner->setIncomingEdges((*it)->getIncomingEdges());
			spawner->setOutigoingEdges((*it)->getOutgoingEdges());
			_spawner.push_back(spawner);

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
	createTrafficLights();
}

float Graph::distance(int vertex1, int vertex2, std::queue<int> route) {
	//std::cout <<"Start" << vertex1 <<" Goal"<< vertex2<< '\n';
	float distance = 0;
	int origin = vertex1;
	//route.pop();
	//std::cout << "OutgoingNeighbor:" << route.front() << '\n';
	while (route.size() > 0) {
		//std::cout<<" EdgetoNextNeighbor: "<< _vertexMap[origin]->outgoingNeighbor(route.front())->getID()<<std::endl;
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
	tempqueue.pop();
	int origin = startID;
	float totaldistance = 0;
	while (tempqueue.size() > 0) {
		int tempgoal = tempqueue.front();
		totaldistance += distance(origin, tempgoal, tempqueue);
		//VON CHRISTOPH
		/*_vertexMap[origin]->outgoingNeighbor(tempgoal)->addWeightTimetable(currentTimeTableIndex
			+ (totaldistance / _CAR_SPEED_PER_TICK), _CAR_RELEVANCE/totaldistance);*/
		std::pair<int, int> timetableValuePair = calculateTimetableValues(currentTimeTableIndex, totaldistance);
		_vertexMap[origin]->outgoingNeighbor(tempgoal)->addWeightTimetable(timetableValuePair.first, timetableValuePair.second);
		origin = tempqueue.front();
		if(tempqueue.front() == route.back()) return;
		tempqueue.pop();
	}
}

void Graph::createTrafficLights() {
	for (vertexContainer::iterator it = _vertices.begin(); it != _vertices.end(); it++) {
		edgeContainer inEdges = (*it)->getIncomingEdges();
		std::vector<std::pair<int, int>> tLMap;
		//for (edgeContainer::iterator it2 = inEdges.begin(); it2 != inEdges.end(); it2++) {
		if(inEdges.size() > 1){
			//std::cout << inEdges[0]->getID() << "\n";
			tLMap.push_back(std::make_pair(inEdges[0]->getID(), inEdges[1]->getID()));
		}
		if (inEdges.size() == 4 ) {
			tLMap.push_back(std::make_pair(inEdges[2]->getID(), inEdges[3]->getID()));
		}
		if (inEdges.size() == 3) {
			tLMap.push_back(std::make_pair(inEdges[2]->getID(), inEdges[2]->getID()));
		}
		else if (inEdges.size() == 1){
			tLMap.push_back(std::make_pair(inEdges[0]->getID(), inEdges[0]->getID()));
		}
		//for (std::vector<std::pair<int, int>> ::iterator it = tLMap.begin(); it != tLMap.end(); ++it) {
		//	std::cout << (*it).first << " " << (*it).second << "\n";
		//}

		//nicht "wirklich" zufällig, soll aber reicehn (Bei jeder durchführung wird gleiche generierungsufnktion
		//für zufallzahl genommen
		int start = rand() % TRAFFICLIGHT_DURATION;
		std::cout << "rand:" << start;
		TrafficLight tl(tLMap, TRAFFICLIGHT_DURATION, start);
		(*it)->setTrafficLight(tl);
	}
}
//Calculate values to enter into add and remove function for timetables
std::pair<int,int> Graph::calculateTimetableValues(int intitialTimetableIndex, float totalDist) {
	return std::make_pair(intitialTimetableIndex + (totalDist/ _CAR_SPEED_PER_TICK), _CAR_RELEVANCE);
}

int Graph::getSumWeightFromTimeTables(int startID, int destID, int currentTimeTableIndex, std::queue<int> route) {
	//std::cout << " New Goal : Start" << startID << " Goal" << destID << " " << route.front() <<'\n';
	std::map<int, float> costs;
	std::queue<int> tempqueue = route;
	int origin = startID;
	tempqueue.pop();
	int timeTableValues = 0;
	float totaldistance = 0;
	while (tempqueue.size() > 0) {
		int tempgoal = tempqueue.front();
		//std::cout << "CurrentnextVertex:" << route.front() << '\n';
		totaldistance += distance(origin, tempgoal, tempqueue);
		//std::cout << "distance" << totaldistance << '\n';
		timeTableValues += _vertexMap[origin]->outgoingNeighbor(tempgoal)->
			getWeightTimetable(currentTimeTableIndex
				+ (totaldistance / _CAR_SPEED_PER_TICK));
			origin = tempqueue.front();
			//std::cout << "reached end" << std::endl;
			if (tempqueue.front() == route.back()) return timeTableValues;
			//std::cout << "nextIteration for getSumWeight"<< std::endl;
			tempqueue.pop();
	}
	return timeTableValues;
}

float Graph::distance_heuristicOverID(size_t start, size_t goal) {
	std::pair<float, float> korStart = _vertexMap[start]->getPosition();
	std::pair<float, float> korEnd = _vertexMap[goal]->getPosition();
	return sqrt(pow((korStart.first - korEnd.first), 2.0f) +
		pow((korStart.second - korEnd.second), 2.0f));
}
