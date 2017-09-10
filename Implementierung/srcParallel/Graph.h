#pragma once
#include <boost/graph/astar_search.hpp>
#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/random.hpp>
#include <boost/random.hpp>
#include <boost/graph/graphviz.hpp>
#include <time.h>
#include <list>
#include <fstream>
#include <math.h>    // for sqrt
#include "Edge.h"
#include "Vertex.h"
#include "Spawner.h"


class Graph
{
public:
	Graph();

	std::vector<Spawner*> getSpawner();
	Edge* getEdge(int edgeID);
	std::vector<Edge*> getEdges();
	std::vector<Vertex*> getVertices();
	std::map<int, Vertex*> getVertexMap();
	float distance(int vertex1, int vertex2, std::queue<int> route);
	void addWeightToTimeTables(int startID, int destID, int currentTimeTableIndex, std::queue<int> route);
	std::map<int, Spawner*> createSpawnerMap();
	static std::pair<int,int> calculateTimetableValues(int intitialTimetableIndex, float toaldistance);
	std::vector<int> createSpawnerIDVector();
	int getSumWeightFromTimeTables(int startID, int destID, int currentTimeTableIndex, std::queue<int> route);
	float distance_heuristicOverID(size_t start, size_t goal);
	int getNumberVertices();
	int getNumberEdges();
	float getMaxX();
	float getMaxY();

private:
	void initGraphProperties();
	void createTrafficLights();

	float _maxX;
	float _maxY;
	std::vector<Spawner*> _spawner;
	std::vector<Edge*> _edges;
	std::vector<Vertex*> _vertices;
	std::map<int, Edge*> _edgeMap;
	std::map<int, Vertex*> _vertexMap;
};


