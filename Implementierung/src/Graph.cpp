#include "Graph.h"
#include <boost/config.hpp>
#include <iostream>
#include <boost/graph/adjacency_list.hpp>
#include <algorithm>
#include <vector>
#include "VertexFileReader.h"
#include "EdgeFileReader.h"

#include "boost/graph/graph_traits.hpp"

using namespace boost;


Graph::Graph()
{
	_vertices = readVertexFile("nodes");
	_edges = calculateEdges(_vertices, "edges");
	connectVertices(_edges);
	_spawner = filterSpawner();
}


std::vector<Vertex*> Graph::filterSpawner() {
	for (std::vector<Vertex*>::iterator it2 = _vertices.begin(); it2 != _vertices.end(); it2++) {
				
	}
}

Graph::~Graph()
{
}
