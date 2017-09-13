#include "RoutingTable.h"
#include "mpi.h"
#include <algorithm>

typedef std::vector<Vertex*> vertexContainer;
typedef std::vector<Spawner*> spawnerContainer;


using namespace boost;

//using namespace boost::lambda;
//
//typedef float coord_t;
//typedef tuple<coord_t, coord_t> point_t;
//
//coord_t RoutingTable::distance_sq(Vertex* v1, Vertex* v2) { // or boost::geometry::distance
//	point_t a = make_tuple(v1->getX(), v1->getY());
//	point_t b = make_tuple(v2->getX(), v2->getY());
//		coord_t x = a.get<0>() - b.get<0>();
//		coord_t y = a.get<1>() - b.get<1>();
//		return x*x + y*y;
//}

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

bool RoutingTable::comp(const std::pair<int, float> &a, const std::pair<int, float> &b) {
	return a.second < b.second;
}

//TODO
void RoutingTable::insertProcessRoutes(std::pair<int, std::vector<std::pair<int, int>>> map)
{
	processRoutesMap[map.first] = map.second;
}

//Returns connections used in Simulation.cpp
std::pair<std::map<int, std::vector<int>>, std::map<int, std::vector<int>>> RoutingTable::getProcessConnectionVectors()
{
	std::pair<std::map<int, std::vector<int>>, std::map<int, std::vector<int>>> connectionPair;

	int rank;
	MPI_Comm_rank(MPI_COMM_WORLD, &rank);

	std::vector<int> localVertices;

	//Get vertecies which are in this process by reconstructing routes via routing table
	for (const std::vector<int> &vector : processRoutingMatrix) {
		//Go through every vertex in the vector
		for (const int &vertex : vector) {
			//If this vertex is not yet contained in localVertices
			if (std::find(localVertices.begin(), localVertices.end(), vertex) == localVertices.end()) {
				//Add vertex localVertices
				localVertices.push_back(vertex);
			}
		}
	}

	//first int -> processID, second int -> edgeID
	std::map<int, std::vector<int>> incomingEdges;
	std::map<int, std::vector<int>> outgoingEdges;

	//Map which contains every vertex of the graph
	std::map<int, Vertex*> globalVerticesMap = _graph->getVertexMap();

	//Go through vertices of the local process and add the edges attached to them 
	for (int &vertexID : localVertices) {
		//Get every outgoing edge of current vertex
		for (Edge* &edge : globalVerticesMap[vertexID]->getOutgoingEdges()) {

			Vertex* currentEndVertex = edge->getVertices().second;

			//If the endVertex of the edge is not inside this process (localVertices)
			if (std::find(localVertices.begin(), localVertices.end(), currentEndVertex->getID()) == localVertices.end()) {
				
				//TODO
				int currentEndVertexProcessID;
				//Add both edges of the same street between the two vertices to the proper vector
				outgoingEdges[currentEndVertexProcessID].push_back(edge->getID());
				incomingEdges[currentEndVertexProcessID].push_back(currentEndVertex->outgoingNeighbor(vertexID)->getID());
			}
		}
	}

	//Sort vectors in ascending order
	for (auto &v : incomingEdges) {
		std::sort(v.second.begin(), v.second.end(), std::less<int>());
	}

	//Sort vectors in ascending order
	for (auto &v : outgoingEdges) {
		std::sort(v.second.begin(), v.second.end(), std::less<int>());
	}

	connectionPair.first = incomingEdges;
	connectionPair.second = outgoingEdges;

	return connectionPair;
}

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

RoutingTable::RoutingTable(Graph* graph, int numberNearestNeighbors) {
	_graph = graph;
	_numberNearestNeighbors = numberNearestNeighbors;
	std::vector<Spawner*> _spawner = graph->getSpawner();
	calculateRoutes(_spawner);
}

RoutingTable::RoutingTable(Graph* graph, int numberNearestNeighbors, std::map<int, std::map<int, std::queue<int>>> routingMatrixP) {
	_graph = graph;
	_numberNearestNeighbors = numberNearestNeighbors;
	routingMatrix = routingMatrixP;
}


RoutingTable::RoutingTable(Graph* graph, int numberNearestNeighbors, std::vector<int> spawnerID) {
	_graph = graph;
	_numberNearestNeighbors = numberNearestNeighbors;
	std::vector<Spawner*> spawner;
	for (int i = 0; i < spawnerID.size(); i++) {
		spawner.push_back(_graph->createSpawnerMap()[spawnerID[i]]);
	}
	calculateRoutes(spawner);
}


void RoutingTable::calculateRoutes(std::vector<Spawner*> _spawner) {
	std::vector<Spawner*> spawners = _graph->getSpawner();
	std::vector<Edge*> _edges = _graph->getEdges();
	std::map<int, Vertex*> _vertexMap = _graph->getVertexMap();
	std::vector<Vertex*> _vertices = _graph->getVertices();

	typedef adjacency_list<listS, vecS, directedS, no_property,
		property<edge_weight_t, int> > mygraph_t;
	typedef property_map<mygraph_t, edge_weight_t>::type WeightMap;
	typedef mygraph_t::vertex_descriptor vertex;
	typedef mygraph_t::edge_descriptor edge_descriptor;

	unsigned int num_edges = unsigned int(_edges.size());
	std::vector<int> vertices_ID;
	for (vertexContainer::iterator it = _vertices.begin(); it != _vertices.end(); it++) {
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
	for (spawnerContainer::iterator it2 = _spawner.begin(); it2 != _spawner.end(); it2++) {
		int start = (*it2)->getID();
		std::map<int, float> distances;
		for (spawnerContainer::iterator it = spawners.begin(); it != spawners.end(); it++) {
			int goal = (*it)->getID();

			//Speicher alle Abstände von Knoten zum Startknoten
			distances[goal] = _vertexMap[start]->distanceTo(_vertexMap[goal]);
			int sumDistance = 0;
			//Bei Parallelisierung wird diese Zeile nicht beachtet, was in der theorie zu einem zweifachen aufwand führt
			//if (!(getRoute(goal, start).empty()))	continue;

			std::vector<mygraph_t::vertex_descriptor> p(num_vertices(g));
			std::vector<float> d(num_vertices(g));
			try {
				// call astar named parameter interface
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
				std::cout << "Shortest path from " << _vertexMap[start]->getID() << " to "
					<< _vertexMap[goal]->getID() << ": ";
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
				insertRoute(start, goal, route);
				setCost(start, goal, sumDistance);
				continue;
			}
			std::cout << "Didn't find a path from " << _vertexMap[start]->getID() << "to"
				<< _vertexMap[goal]->getID() << "!" << std::endl;
		}
		size_t m = _numberNearestNeighbors;
		//Erstelle liste mit abständen und mache partial sort
		std::vector<std::pair<int, float>> v{ distances.begin(), distances.end() };
		if (int(v.size()) < _numberNearestNeighbors) {
			m = v.size();
		}
		std::partial_sort(v.begin(), v.begin() + m, v.end(), &comp);
		for (std::vector<std::pair<int, float>> ::iterator it = v.begin(); it != v.begin() + m; it++) {
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
	//Would cause problems on Parallelism
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

//TODO BIG REFACTORING
int RoutingTable::calculateBestGoal(int startID, int destID, int currentTimeTableIndex)
{
	std::map<int, float> costs;
	for (int goalID : k_nn[destID]) {
		if (goalID == startID || getRoute(startID, goalID).empty()) {
			//std::cout << "Would take same";
			continue;
		}
		//std::cout << "CurrentGoalvertex " << goalID << std::endl;
		int timeTableValue = _graph->getSumWeightFromTimeTables(startID, goalID, currentTimeTableIndex, routingMatrix[startID][goalID]);
		//Berechnet zusatzkosten, dafür das Zeil weiter weg vom eigentlichen ausgangsziel ist, nutze hierfür distanzheuristik
		float extraCosts = _graph->distance_heuristicOverID(destID, goalID) / 2;
		costs[goalID] = costMatrix[startID][goalID] + timeTableValue + extraCosts;
		//std::cout << "calculated for one goalVertex" <<std::endl;
	}
	//std::cout << "afterAll ";
	if (costs.empty()) {
		return -1;
	}
	std::vector<std::pair<int, float>> v{ costs.begin(), costs.end() };
	std::partial_sort(v.begin(), v.begin() + 1, v.end(), &comp);
	std::cout << "Best Goal for Car: " << v[0].first << std::endl;
	return v[0].first;
}

void RoutingTable::addCosts(int startID, int destID, int currentTimeTableIndex) {
	std::queue<int> tempqueue = routingMatrix[startID][destID];
	//TODO mache queue so, dass man hier nicht poppen muss
	//tempqueue.pop();
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
