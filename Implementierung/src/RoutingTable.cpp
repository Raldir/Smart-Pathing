#include "RoutingTable.h"

typedef std::vector<Vertex*> vertexContainer;
typedef std::vector<Spawner*> spawnerContainer;

using namespace boost;

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



RoutingTable::RoutingTable(Graph* graph) {

	std::vector<Edge*> _edges = graph->getEdges();
	std::map<int, Vertex*> _vertexMap = graph->getVertexMap();
	std::vector<Vertex*> _vertices = graph->getVertices();
	std::vector<Spawner*> _spawner = graph->getSpawner();

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
		for (spawnerContainer::iterator it = _spawner.begin(); it != _spawner.end(); it++) {
			int goal = (*it)->getID();
			int sumDistance = 0;
			//Entscheidung: Berechne Hin u. Rückrouten seperat
			//if (!(_routingTable->getRoute(goal, start).empty())) {
			//	std::cout << "hello";
			//	continue;
			//}
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
						lastElement = (*spi);
					}
					route.push(_vertexMap[int(*spi)]->getID());

				}
				insertRoute(start, goal, route);
				setCost(start, goal, sumDistance);
				if (!route.empty())
					std::cout << route.front();
				std::cout << std::endl << "Total travel time: " << d[goal] << std::endl;
				continue;
			}
			std::cout << "Didn't find a path from " << _vertexMap[start]->getID() << "to"
				<< _vertexMap[goal]->getID() << "!" << std::endl;
		}
	}
}

void RoutingTable::insertRoute(int originID, int destID, std::queue<int> route)
{
	//Pushes originID into first map and int queue pair into the second
	routingMatrix[originID][destID] = route;
	
	//Pushes queue on symmetrical pair
	routingMatrix[destID][originID] = route;

	std::cout << "Added queue from " << originID << " to " << destID << " and reverse." << std::endl;
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

//Gibt Route zwischen origin und destination aus
std::queue<int>  RoutingTable::getRoute(int originID, int destID) {

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

void RoutingTable::setCost(int originID, int destID, float cost)
{
	costMatrix[originID][destID] = cost;
}

float RoutingTable::getCost(int originID, int destID)
{
	return costMatrix[originID][destID];
}
