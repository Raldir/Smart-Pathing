#include "Graph.h"
#include <iostream>
#include <algorithm>
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
	filterSpawner();
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

std::vector<Vertex*> Graph::getVertices()
{
	return _vertices;
}


void Graph::filterSpawner() {
	for (vertexContainer::iterator it = _vertices.begin(); it != _vertices.end(); it++) {
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
			//TODO Wert nicht statisch setzen sondern dynamisch bestimmen. Veilleicht später die n wenigsten verbundenen
			//Knoten nehmen
		if ((*it)->getEdges().size() == 2) {
			std::cout << "hello";
			_spawner.push_back(new Spawner((*it)->getID(), (*it)->getX(), (*it)->getY()));
		}
	}
}

float Graph::distance_heuristic2(size_t start, size_t goal) {
	std::pair<float, float> korStart = _vertexMap[start]->getPosition();
	std::pair<float, float> korEnd = _vertexMap[goal]->getPosition();
	return sqrt(pow((korStart.first - korEnd.first), 2.0f) +
		pow((korStart.second - korEnd.second), 2.0f));
}


