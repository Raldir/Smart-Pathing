#include "Graph.h"
#include <boost/config.hpp>
#include <iostream>
#include <algorithm>
#include <vector>
#include "VertexFileReader.h"
#include "EdgeFileReader.h"

#include <boost/graph/astar_search.hpp>
#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/random.hpp>
#include <boost/random.hpp>
#include <boost/graph/graphviz.hpp>
#include <time.h>
#include <list>
#include <fstream>
#include <math.h>    // for sqrt

using namespace boost;


Graph::Graph()
{
	_vertices = readVertexFile("nodes");
	_edges = calculateEdges(_vertices, "edges");
	connectVertices(_edges);
	filterSpawner();
	_vertexMap = vertexMap(_vertices);
	_edgeMap = edgeMap(_edges);
	_routingTable = new RoutingTable();
}


void Graph::filterSpawner() {
	for (std::vector<Vertex*>::iterator it2 = _vertices.begin(); it2 != _vertices.end(); it2++) {
		if ((*it2)->getIncomingEdges().empty() || (*it2)->getOutgoingEdges().empty()) {
			_spawner.push_back(*it2);
		}
	}
}

float Graph::distance_heuristic(int start, int goal) {
	std::pair<int, int> korStart = _vertexMap[start]->getPosition();
	std::pair<int, int> korEnd = _vertexMap[goal]->getPosition();
	return sqrt(pow((korStart.first - korEnd.first), 2.0f) +
		pow((korStart.second - korEnd.second), 2.0f));
}


struct found_goal {};

template <class Vertex>
class astar_goal_visitor : public boost::default_astar_visitor
{
public:
	astar_goal_visitor(Vertex goal) : m_goal(goal) {}
	template <class Graph>
	void examine_vertex(Vertex u, Graph& g) {
		if (u == m_goal)
			throw found_goal();
	}
private:
	Vertex m_goal;
};

void Graph::calculateRoutingPaths(){
	// specify some types
	typedef adjacency_list<listS, vecS, directedS, no_property,
		property<edge_weight_t, int> > mygraph_t;
	typedef property_map<mygraph_t, edge_weight_t>::type WeightMap;
	typedef mygraph_t::vertex_descriptor vertex;
	typedef mygraph_t::edge_descriptor edge_descriptor;
	typedef mygraph_t::vertex_iterator vertex_iterator;

	unsigned int num_edges = unsigned int(_edges.size());
	std::vector<int> vertices_ID;
	for (std::vector<Vertex*>::iterator it = _vertices.begin(); it != _vertices.end(); it++) {
		vertices_ID.push_back((*it)->getID());
	}

	// create graph
	mygraph_t g(vertices_ID.size());
	WeightMap weightmap = get(edge_weight, g);
	for (std::vector<Edge*>::iterator it = _edges.begin(); it != _edges.end(); it++) {
		{
			edge_descriptor e; bool inserted;
			tie(e, inserted) = add_edge((*it)->getVertices().first->getID(),
				(*it)->getVertices().second->getID(), g);
			weightmap[e] = int((*it)->getLength());
		}
	}
	for (std::vector<Vertex*>::iterator it2 = _spawner.begin(); it2 != _spawner.end(); it2++) {
		int start = (*it2)->getID();
		for (std::vector<Vertex*>::iterator it = _spawner.begin(); it != _spawner.end(); it++) {
			int goal = (*it)->getID();
			/*if (_routingTable->getRoute(goal, start) == NULL) {
				break;
			}*/

			std::vector<mygraph_t::vertex_descriptor> p(num_vertices(g));
			std::vector<int> d(num_vertices(g));
			try {
				// call astar named parameter interface
				astar_search
					(g, start,
						distance_heuristic(start, goal),
						predecessor_map(&p[0]).distance_map(&d[0]).
						visitor(astar_goal_visitor<vertex>(goal)));


			}
			catch (found_goal fg) { // found a path to the goal
				std::list<vertex> shortest_path;
				for (vertex v = goal;; v = p[v]) {
					shortest_path.push_front(v);
					if (p[v] == v)
						break;
				}
				std::cout << "Shortest path from " << _vertexMap[start]->getID()  << " to "
					<< _vertexMap[goal]->getID() << ": ";
				std::list<vertex>::iterator spi = shortest_path.begin();
				std::cout << _vertexMap[start]->getID();
				for (++spi; spi != shortest_path.end(); ++spi)
					std::cout << " -> " << _vertexMap[*spi]->getID();
				std::cout << std::endl << "Total travel time: " << d[goal] << std::endl;
			}

			std::cout << "Didn't find a path from " << _vertexMap[start]->getID() << "to"
				<< _vertexMap[goal]->getID() << "!" << std::endl;
		}
	}


	

}

Graph::~Graph()
{
}
