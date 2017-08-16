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
	 _routingTable = new RoutingTable();
}


std::vector<Vertex*> Graph::filterSpawner() {
	for (std::vector<Vertex*>::iterator it2 = _vertices.begin(); it2 != _vertices.end(); it2++) {
		if ((*it2)->getIncomingEdges().empty() || (*it2)->getOutgoingEdges.empty()) {
			_spawner.push_back((*it2));
		}
	}
}

void Graph::calculateRoutingPaths(){
	// specify some types
	typedef adjacency_list<listS, vecS, directedS, no_property,
		property<edge_weight_t, int> > mygraph_t;
	typedef property_map<mygraph_t, edge_weight_t>::type WeightMap;
	typedef mygraph_t::vertex_descriptor vertex;
	typedef mygraph_t::edge_descriptor edge_descriptor;
	typedef mygraph_t::vertex_iterator vertex_iterator;

	unsigned int num_edges = _edges.size();
	std::vector<int> vertices_ID;
	for (std::vector<Vertex*>::iterator it = _vertices.begin(); it != _vertices.end(); it++) {
		vertices_ID.push_back((*it)->getID);
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
			//CHRISTOPH: Habe hier bei leerer queue fall von NULL zu empty verändert
			if (_routingTable->getRoute(goal, start).empty()) {
				break;
			}

		}
	}

	ofstream dotfile;
	dotfile.open("test-astar-cities.dot");
	write_graphviz(dotfile, g,
		city_writer<const char **, location*>
		(name, locations, 73.46, 78.86, 40.67, 44.93,
			480, 400),
		time_writer<WeightMap>(weightmap));


	vector<mygraph_t::vertex_descriptor> p(num_vertices(g));
	vector<cost> d(num_vertices(g));
	try {
		// call astar named parameter interface
		astar_search
			(g, start,
				distance_heuristic<mygraph_t, cost, location*>
				(locations, goal),
				predecessor_map(&p[0]).distance_map(&d[0]).
				visitor(astar_goal_visitor<vertex>(goal)));


	}
	catch (found_goal fg) { // found a path to the goal
		list<vertex> shortest_path;
		for (vertex v = goal;; v = p[v]) {
			shortest_path.push_front(v);
			if (p[v] == v)
				break;
		}
		cout << "Shortest path from " << name[start] << " to "
			<< name[goal] << ": ";
		list<vertex>::iterator spi = shortest_path.begin();
		cout << name[start];
		for (++spi; spi != shortest_path.end(); ++spi)
			cout << " -> " << name[*spi];
		cout << endl << "Total travel time: " << d[goal] << endl;
		return 0;
	}

	cout << "Didn't find a path from " << name[start] << "to"
		<< name[goal] << "!" << endl;
	return 0;

}

Graph::~Graph()
{
}
