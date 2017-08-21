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
	std::vector<Edge*> getEdges();
	std::vector<Vertex*> getVertices();
	std::map<int, Vertex*> getVertexMap();

private:
	void filterSpawner();
	float distance_heuristic2(size_t start, size_t goal);

	std::vector<Spawner*> _spawner;
	std::vector<Edge*> _edges;
	std::vector<Vertex*> _vertices;
	std::map<int, Edge*> _edgeMap;
	std::map<int, Vertex*> _vertexMap;
};


