/*############################################

Author: Rami Aly and Christoph Hueter
Date : 20.09.17
Libraries and Licences:
Boost Library 1.60, MPI Boost Library 1.60, Open Streetmaps and MPI in use.
All used Maps are licensed under the Open Street Maps License.

#############################################*/

//TODO WIEDER EINKOMMENTIEREN

#include "RoutingTable.h"

/*#include <boost/graph/use_mpi.hpp>
#include <boost/config.hpp>
#include <boost/throw_exception.hpp>
#include <boost/serialization/vector.hpp>
#include <boost/graph/dijkstra_shortest_paths.hpp>
#include <boost/graph/distributed/mpi_process_group.hpp>
#include <boost/graph/distributed/adjacency_list.hpp>*/
//#include <boost/test/minimal.hpp>


#include <boost/graph/astar_search.hpp>
#include <boost/graph/adjacency_list.hpp>

typedef std::vector<Vertex*> vertexContainer;
typedef std::vector<Spawner*> spawnerContainer;

using namespace boost;
//using boost::graph::distributed::mpi_process_group;

//Reverse Queue
std::queue<int> RoutingTable::reverseQueue(std::queue<int> queue)
{
	std::queue<int> revQueue;

	if (queue.empty()) {
		return queue;
	}

	int vertex = queue.front();
	queue.pop();

	revQueue = reverseQueue(queue);
	revQueue.push(vertex);
	return revQueue;
}

//Comperator that compares two pairs by it second value in ascending
bool RoutingTable::comp(const std::pair<int, float> &a, const std::pair<int, float> &b) {
	return a.second < b.second;
}

//Templates for the Distance heuristic, used for shortest paths A_star
template <class Graph, class CostType, class LocMap>
class distance_heuristic : public astar_heuristic<Graph, CostType>
{
public:
	typedef typename graph_traits<Graph>::vertex_descriptor Vertex;
	distance_heuristic(LocMap l, Vertex goal)
		: m_location(l), m_goal(goal) {}
	CostType operator()(Vertex u)
	{
		CostType dx = m_location[m_goal]->getPosition().first - m_location[u]->getPosition().first;
		CostType dy = m_location[m_goal]->getPosition().second - m_location[u]->getPosition().second;
		return ::sqrt(dx * dx + dy * dy);
	}
private:
	LocMap m_location;
	Vertex m_goal;
};

struct found_goal {};// exception for termination

//Tempate for astar visitor, needed to detect that shortest path was found
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

void RoutingTable::setCost(int originID, int destID, float cost)
{
	costMatrix[originID][destID] = cost;
}

float RoutingTable::getCost(int originID, int destID)
{
	return costMatrix[originID][destID];
}

/*
The Constructor for the Routingtable, that will calculate each Route parallel but the number of Routes sequential
*/
RoutingTable::RoutingTable(Graph* graph, int numberNearestNeighbors) {

	_graph = graph;
	_numberNearestNeighbors = numberNearestNeighbors;
	std::vector<Spawner*> _spawner = graph->getSpawner();
	calculateRoutesParallel(_spawner);
}

/*
The Constructor for the Routingtable, that will calculate each Route sequential (used for sequential distributed calculations
*/
RoutingTable::RoutingTable(Graph* graph, int numberNearestNeighbors, std::map<int, std::map<int, std::queue<int>>> routingMatrixP, std::map<int, std::map<int, float>> costs) {

	_graph = graph;
	_numberNearestNeighbors = numberNearestNeighbors;
	routingMatrix = routingMatrixP;
	costMatrix = costs;
}

/*
The Constructor for the Routingtable, that will calculate each Route sequential (used for sequential distributed calculations
*/
RoutingTable::RoutingTable(Graph* graph, int numberNearestNeighbors, std::vector<int> verticesID) {

	_graph = graph;
	_numberNearestNeighbors = numberNearestNeighbors;
	std::vector<Spawner*> spawner;
	for (int i = 0; i < verticesID.size(); i++) {
		spawner.push_back(_graph->createSpawnerMap()[verticesID[i]]);
	}
	calculateRoutes(spawner);
}



/*
Calculate the Routingtable sequential(however it is implemented so that seperate calculations and merging of different routes through other
processes is feasible
*/
void RoutingTable::calculateRoutes(std::vector<Spawner*> _spawner) {

	//loads and stores data of the graph
	std::vector<Spawner*> spawners = _graph->getSpawner();
	std::vector<Edge*> _edges = _graph->getEdges();
	std::map<int, Vertex*> _vertexMap = _graph->getVertexMap();
	std::vector<Vertex*> _vertices = _graph->getVertices();

	//Defines an adjacency list, defines the type, storage type and properties of the Graph(syntactically)
	typedef adjacency_list<listS, vecS, directedS, no_property,
		property<edge_weight_t, int> > mygraph_t;
	typedef property_map<mygraph_t, edge_weight_t>::type WeightMap;
	typedef mygraph_t::vertex_descriptor vertex;
	typedef mygraph_t::edge_descriptor edge_descriptor;

	int num_edges = (_edges.size());

	// creates Graph
	mygraph_t g(_vertices.size());
	WeightMap weightmap = get(edge_weight, g);
	//Adds Edges to the graph and add weight to the weightmap;
	for (std::vector<Edge*>::iterator it = _edges.begin(); it != _edges.end(); it++) {
		{
			edge_descriptor e; bool inserted;
			tie(e, inserted) = add_edge((*it)->getVertices().first->getID(),
				(*it)->getVertices().second->getID(), g);
			weightmap[e] = int((*it)->getLength());
		}
	}
	for (spawnerContainer::iterator it2 = _spawner.begin(); it2 != _spawner.end(); it2++) {
		int start = (*it2)->getID();
		std::map<int, float> distances;
		for (spawnerContainer::iterator it = spawners.begin(); it != spawners.end(); it++) {
			int goal = (*it)->getID();

			//stores the flight distances between startnode to destination node
			distances[goal] = _vertexMap[start]->distanceTo(_vertexMap[goal]);
			//used for the costmatrix
			int sumDistance = 0;
			//Bei Parallelisierung wird diese Zeile nicht beachtet, was in der theorie zu einem zweifachen aufwand führt
			//if (!(getRoute(goal, start).empty()))	continue;

			//Used to store the predecessors and distance that are calculated with the astar
			std::vector<mygraph_t::vertex_descriptor> p(num_vertices(g));
			std::vector<float> d(num_vertices(g));
			try {
				// call astar 
				astar_search
				(g, start,
					distance_heuristic<mygraph_t, float, std::map<int, Vertex*>>
					(_vertexMap, goal),
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
				//Little visual output
				std::cout << "Shortest path from " << _vertexMap[start]->getID() << " to "
					<< _vertexMap[goal]->getID() << ": ";
				//iterate through the found shortest path and store the vertices
				std::list<vertex>::iterator spi = shortest_path.begin();
				std::cout << _vertexMap[start]->getID();
				std::queue<int> route;
				int lastElement;
				for (++spi; spi != shortest_path.end(); ++spi) {
					std::cout << " -> " << _vertexMap[int(*spi)]->getID();
					if (!route.empty()) {
						sumDistance += _vertexMap[(*spi)]->incomingNeighbor(lastElement)->getLength();
						lastElement = (*spi);
					}
					else {
						route.push(_vertexMap[start]->getID());
						lastElement = (*spi);
					}
					route.push(_vertexMap[int(*spi)]->getID());

				}
				//Insert the Route and Cost into the Routingtable
				insertRoute(start, goal, route);
				setCost(start, goal, sumDistance);
				continue;
			}
			//If no path is found
			std::cout << "Didn't find a path from " << _vertexMap[start]->getID() << "to"
				<< _vertexMap[goal]->getID() << "!" << std::endl;
		}
		size_t m = _numberNearestNeighbors;
		//Create Vector with the distances of each vertex to the origin vertex and select the k vertices with the sortest distance
		std::vector<std::pair<int, float>> v{ distances.begin(), distances.end() };
		if (int(v.size()) < _numberNearestNeighbors) {
			m = v.size();
		}
		std::partial_sort(v.begin(), v.begin() + m, v.end(), &comp);
		for (std::vector<std::pair<int, float>> ::iterator it = v.begin(); it != v.begin() + m; it++) {
			//add them to the vector
			k_nn[start].push_back((*it).first);
			std::cout << (*it).first << ">";
		}
		std::cout << std::endl;
	}
}


void RoutingTable::insertRoute(int originID, int destID, std::queue<int> route)
{
	//Pushes originID into first map and int queue pair into the second
	routingMatrix[originID][destID] = route;

	//Pushes queue on symmetrical pair
	//################################CAN Be uncommented if parallelism is not used to improve performance########################
	//routingMatrix[destID][originID] = reverseQueue(route);

	std::cout << "Added queue from " << originID << " to " << destID << std::endl;
}

void RoutingTable::removeRoute(int originID, int destID)
{
	//Erases queue origin to destination and reverse
	routingMatrix[originID].erase(destID);
	routingMatrix[destID].erase(originID);

	std::cout << "Deleted queue from " << originID << " to " << destID << " and reverse." << std::endl;
}

void RoutingTable::replaceRoute(int originID, int destID, std::queue<int> route)
{
	removeRoute(originID, destID);

	insertRoute(originID, destID, route);

}


int RoutingTable::calculateBestGoal(int startID, int destID, int currentTimeTableIndex)
{

	//TODO BIG REFACTORING, bad design
	std::map<int, float> costs;
	//Iterate through the k nearest goals beside the main exist to see wether the cost would be smaller
	for (int goalID : k_nn[destID]) {
		//Would take the same route for exit as entrance, not valid(at least currently)
		if (goalID == startID || getRoute(startID, goalID).empty()) {
			continue;
		}
		int timeTableValue = _graph->getSumWeightFromTimeTables(startID, goalID, currentTimeTableIndex, routingMatrix[startID][goalID]);
		//Berechnet zusatzkosten, dafür das Zeil weiter weg vom eigentlichen ausgangsziel ist, nutze hierfür distanzheuristik
		float extraCosts = _graph->distance_heuristicOverID(destID, goalID) / 2;
		costs[goalID] = costMatrix[startID][goalID] + timeTableValue + extraCosts;
	}
	//Catch exeception when the Route is empty
	if (costs.empty()) {
		return -1;
	}
	//Select the goal with the smallest cost
	std::vector<std::pair<int, float>> v{ costs.begin(), costs.end() };
	std::partial_sort(v.begin(), v.begin() + 1, v.end(), &comp);
	std::cout << "Best Goal for Car: " << v[0].first << std::endl;
	return v[0].first;
}

void RoutingTable::addCosts(int startID, int destID, int currentTimeTableIndex) {
	std::queue<int> tempqueue = routingMatrix[startID][destID];
	//Add a method that only adds on a specific range(becuase it get unprecise on a longer distance)
	_graph->addWeightToTimeTables(startID, destID, currentTimeTableIndex, tempqueue);
}

//Gibt Route zwischen origin und destination aus
std::queue<int> RoutingTable::getRoute(int originID, int destID) {

	std::queue<int> queue;

	RoutingMatrix::iterator iterOrigin = routingMatrix.find(originID);
	if (iterOrigin != routingMatrix.end()) {
		//Map where destinationID and queues are stored
		std::map<int, std::queue<int>> destinationMap = iterOrigin->second;
		std::map<int, std::queue<int>>::iterator iterDest = destinationMap.find(destID);
		if (iterDest != destinationMap.end()) {
			queue = destinationMap.find(destID)->second;
		}
	}
	else {
		std::cout << "No queue found in Routing Table from" << originID << " to " << destID << std::endl;
		return std::queue<int>();
	}

	return queue;
}

/*Converts RoutingMatrix to a 2 dimensional Vector with effektive no information loose, since every route stores
its start and destination*/
std::vector<std::vector<int>> RoutingTable::getRoutingMatrix()
{
	std::vector<std::vector<int>> matrix;
	for (auto const &ent1 : routingMatrix) {
		for (auto const &ent2 : ent1.second) {
			std::vector<int> route;
			std::queue<int> copy = routingMatrix[ent1.first][ent2.first];
			if (copy.size() == 0) {
				continue;
			}
			//std::cout << routingMatrix[ent1.first][ent2.first].size();
			for (int i = 0; i < routingMatrix[ent1.first][ent2.first].size(); i++) {
				//std::cout << ent1.first << " " << ent2.first<<std::endl;
				route.push_back(copy.front());
				copy.pop();
			}
			matrix.push_back(route);
		}
	}
	return matrix;
}

/*
Converts Costmatrix to a twodimensional Vector. Each vector stores the start, end and the cost to travel from start to the goal
*/
std::vector<std::vector<int>> RoutingTable::getRoutingCosts()
{
	std::vector<std::vector<int>> matrix;
	for (auto const &ent1 : costMatrix) {
		for (auto const &ent2 : ent1.second) {
			std::vector<int> costsOrigin;
			costsOrigin.push_back(ent1.first);
			costsOrigin.push_back(int(ent2.second));
			costsOrigin.push_back(ent2.first);
			matrix.push_back(costsOrigin);
		}
	}
	return matrix;
}

/*
Converts K nearest neighbor matrix to twodimensional Vector
*/
std::vector<std::vector<int>> RoutingTable::getKNearestMatrix()
{
	std::vector<std::vector<int>> matrix;
	for (auto const &ent1 : k_nn) {
		std::vector<int> k_nnOrigin;
		for (int i = 0; i < k_nn[ent1.first].size(); i++) {
			k_nnOrigin.push_back(ent1.second[i]);
		}
		matrix.push_back(k_nnOrigin);
	}
	return matrix;
}


//Used to select the name of vertex through distirbuted calculations
template<typename Vertex, typename Graph>
int get_vertex_name(Vertex v, const Graph& g, std::vector<int> vertex_names)
{
	return vertex_names[g.distribution().global(owner(v), local(v))];
}

//TODO WIEDER EINKOMMENTIEREN
/*
Parallelism which calculates each Route parallel but the number of Routes sequential
*/
void RoutingTable::calculateRoutesParallel(std::vector<Spawner*> _spawner) { /*
	//Init values of graph(needed since the algorithm uses seperate Graph)
	std::vector<Spawner*> spawners = _graph->getSpawner();
	std::vector<Edge*> _edges = _graph->getEdges();
	std::map<int, int> vertices_ID;
	std::vector<Vertex*> _vertices = _graph->getVertices();
	std::vector<int> vertex_names;
	//define a distributed, directed graph
	typedef adjacency_list<listS, distributedS<mpi_process_group, vecS>, directedS,
		no_property,                 // Vertex properties
		property<edge_weight_t, int> // Edge properties
	> graph_t;
	typedef graph_traits < graph_t >::vertex_descriptor vertex_descriptor;
	typedef graph_traits < graph_t >::edge_descriptor edge_descriptor;

	//assign values to id containers
	int count = 0;
	for (vertexContainer::iterator it = _vertices.begin(); it != _vertices.end(); it++) {
		vertices_ID[(*it)->getID()] = count;
		vertex_names.push_back((*it)->getID());
		count++;
	}
	//create Graph with above properties
	graph_t g(_graph->getNumberVertices());
	//Assign Edges to the Graph
	for (std::vector<Edge*>::iterator it = _edges.begin(); it != _edges.end(); it++) {
		add_edge(vertex((*it)->getVertices().first->getID(), g), vertex((*it)->getVertices().second->getID(), g), int((*it)->getLength()) + 1, g);
		std::cout << int((*it)->getLength());
	}
	//Syncronizes all Graphes of the processes, so they have the same information
	synchronize(g.process_group());

	//Calculate the shortest distance for each Spawner to every other Vertex in the Graph
	for (spawnerContainer::iterator it2 = _spawner.begin(); it2 != _spawner.end(); it2++) {
		vertex_descriptor s = vertex((*it2)->getID(), g);
		//Init Storage for the predecessors and distances to calculate the route afterwards
		std::vector<vertex_descriptor> p(num_vertices(g));
		std::vector<int> d(num_vertices(g));
		//Store the distributed Property map.
		auto parents =
			make_iterator_property_map(p.begin(), get(vertex_index, g));

		//*************NUR AUF Windows AUSKOMMENTIEREN; FUNKTIONIERT AUF WINDOWS NICHT!*******************************
		//Execute distruted Dijkstra
		//dijkstra_shortest_paths
		//	(g, s,
		//		predecessor_map(parents).
		//		distance_map(
		//			make_iterator_property_map(d.begin(), get(vertex_index, g)))
		//		);
		synchronize(g.process_group());

		//Since the distributed dijkstra calculates the sshortest distance form s to every other node in the graph we
		//have to iterate to every other spawner in the graph
		for (spawnerContainer::iterator it = _spawner.begin(); it != _spawner.end(); it++) {
			//Get the distributed vertex(stores owner etc..)
			vertex_descriptor v = vertex((*it)->getID(), g);
			std::vector<int> route;
			/*This part is crucial. Since the properties of The graph are distributed, because the Graph itself is distributed, the
			correct properties, in this case the predecessor of the shortest route is only stored in its owner. Every other process
			probably has a ghost value and the correct value must be requested.

			WIEDER EINKOMMENTIEREN * /

			request(parents, v);
			//After the request is made, it is necessary to synchronize the property map to get the value from its owner
			synchronize(parents);
			vertex_descriptor current = get(parents, v);
			//get the correct name
			route.push_back(get_vertex_name(v, g, vertex_names));
			route.push_back(get_vertex_name(current, g, vertex_names));
			//Some textual output
			if (process_id(g.process_group()) == 0) {
				std::cout << "pathOf(" << get_vertex_name(v, g, vertex_names) << ") = " << get_vertex_name(current, g, vertex_names) << " ";
			}
			while (current != s) {
				//store the parent of the current vertex in same variable
				current = get(parents, current);
				//In case no path exists break
				if (std::find(route.begin(), route.end(), get_vertex_name(current, g, vertex_names)) != route.end()) {
					break;
				}
				route.push_back(get_vertex_name(current, g, vertex_names));
				if (process_id(g.process_group()) == 0) {
					std::cout << get_vertex_name(current, g, vertex_names) << " ";
				}
			}
			if (process_id(g.process_group()) == 0) {
				std::cout << std::endl;
			}
			//Because the iteration began from the destination, it is necessary to reverse the vector
			std::reverse(route.begin(), route.end());
			std::queue<int> routeQ;
			for (size_t i = 0; i < route.size(); i++) {
				routeQ.push(route[i]);
			}
			//Add Route to the Routingtable
			insertRoute(get_vertex_name(s, g, vertex_names), get_vertex_name(v, g, vertex_names), routeQ);
			//Kosten müssten noch berechent werden, soll aber für Weiteres nicht wichtig sein.....
		}
	}
	*/
}


//Calculates the k nearest neighbors sequential
void RoutingTable::calculateKNearest() {
	std::vector<Spawner*> spawners = _graph->getSpawner();
	//VERWENDE EVENTUELL DIE LUFTDISTANZ?? BESSERE HEURISTIK?
	for (spawnerContainer::iterator it2 = spawners.begin(); it2 != spawners.end(); it2++) {
		std::vector<std::pair<int, float>>  v;
		int start = (*it2)->getID();
		for (spawnerContainer::iterator it = spawners.begin(); it != spawners.end(); it++) {
			int goal = (*it)->getID();
			//Uses information of costmatrix
			v.push_back(std::pair<int, float>(goal, costMatrix[start][goal]));
		}
		int m = _numberNearestNeighbors;
		if (int(v.size()) < _numberNearestNeighbors) {
			m = v.size();
		}
		//Select the k vertices with the shortest distance to the selected vertex
		std::partial_sort(v.begin(), v.begin() + m, v.end(), &comp);
		for (std::vector<std::pair<int, float>> ::iterator it = v.begin(); it != v.begin() + m; it++) {
			k_nn[start].push_back((*it).first);
			std::cout << (*it).first << ">";
		}
		std::cout << std::endl;
	}
}
