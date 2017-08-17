#include <iostream>
#include <queue>
#include <vector>
#include "main.h"
#include "Edge.h"
#include "Vertex.h"
#include "Car.h"
#include "Project1.h"
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
using namespace std;

//int main() {
//
//	testChristoph();
//	testRami();
//
//	return 0;
//}


void testRami() {
}

	// auxiliary types
struct location
{
  float y, x; // lat, long
};
typedef float cost;

template <class Name, class LocMap>
class city_writer {
public:
  city_writer(Name n, LocMap l, float _minx, float _maxx,
              float _miny, float _maxy,
              unsigned int _ptx, unsigned int _pty)
    : name(n), loc(l), minx(_minx), maxx(_maxx), miny(_miny),
      maxy(_maxy), ptx(_ptx), pty(_pty) {}
  template <class Vertex>
  void operator()(ostream& out, const Vertex& v) const {
    float px = 1 - (loc[v].x - minx) / (maxx - minx);
    float py = (loc[v].y - miny) / (maxy - miny);
    out << "[label=\"" << name[v] << "\", pos=\""
        << static_cast<unsigned int>(ptx * px) << ","
        << static_cast<unsigned int>(pty * py)
        << "\", fontsize=\"11\"]";
  }
private:
  Name name;
  LocMap loc;
  float minx, maxx, miny, maxy;
  unsigned int ptx, pty;
};

template <class WeightMap>
class time_writer {
public:
  time_writer(WeightMap w) : wm(w) {}
  template <class Edge>
  void operator()(ostream &out, const Edge& e) const {
    out << "[label=\"" << wm[e] << "\", fontsize=\"11\"]";
  }
private:
  WeightMap wm;
};


// euclidean distance heuristic
template <class Graph, class CostType, class LocMap>
class distance_heuristic : public astar_heuristic<Graph, CostType>
{
public:
  typedef typename graph_traits<Graph>::vertex_descriptor Vertex;
  distance_heuristic(LocMap l, Vertex goal)
    : m_location(l), m_goal(goal) {}
  CostType operator()(Vertex u)
  {
    CostType dx = m_location[m_goal].x - m_location[u].x;
    CostType dy = m_location[m_goal].y - m_location[u].y;
    return ::sqrt(dx * dx + dy * dy);
  }
private:
  LocMap m_location;
  Vertex m_goal;
};


struct found_goal {}; // exception for termination

// visitor that terminates when we find the goal
template <class Vertex>
class astar_goal_visitor : public boost::default_astar_visitor
{
public:
  astar_goal_visitor(Vertex goal) : m_goal(goal) {}
  template <class Graph>
  void examine_vertex(Vertex u, Graph& g) {
    if(u == m_goal)
      throw found_goal();
  }
private:
  Vertex m_goal;
};


int main()
{
  
  // specify some types
  typedef adjacency_list<listS, vecS, undirectedS, no_property,
    property<edge_weight_t, cost> > mygraph_t;
  typedef property_map<mygraph_t, edge_weight_t>::type WeightMap;
  typedef mygraph_t::vertex_descriptor vertex;
  typedef mygraph_t::edge_descriptor edge_descriptor;
  typedef mygraph_t::vertex_iterator vertex_iterator;
  typedef std::pair<int, int> edge;
  
  // specify data
  enum nodes {
    Troy, LakePlacid, Plattsburgh, Massena, Watertown, Utica,
    Syracuse, Rochester, Buffalo, Ithaca, Binghamton, Woodstock,
    NewYork, N
  };
  const char *name[] = {
    "Troy", "Lake Placid", "Plattsburgh", "Massena",
    "Watertown", "Utica", "Syracuse", "Rochester", "Buffalo",
    "Ithaca", "Binghamton", "Woodstock", "New York"
  };
  location locations[] = { // lat/long
    {42.73, 73.68}, {44.28, 73.99}, {44.70, 73.46},
    {44.93, 74.89}, {43.97, 75.91}, {43.10, 75.23},
    {43.04, 76.14}, {43.17, 77.61}, {42.89, 78.86},
    {42.44, 76.50}, {42.10, 75.91}, {42.04, 74.11},
    {40.67, 73.94}
  };
  edge edge_array[] = {
    edge(Troy,Utica), edge(Troy,LakePlacid),
    edge(Troy,Plattsburgh), edge(LakePlacid,Plattsburgh),
    edge(Plattsburgh,Massena), edge(LakePlacid,Massena),
    edge(Massena,Watertown), edge(Watertown,Utica),
    edge(Watertown,Syracuse), edge(Utica,Syracuse),
    edge(Syracuse,Rochester), edge(Rochester,Buffalo),
    edge(Syracuse,Ithaca), edge(Ithaca,Binghamton),
    edge(Ithaca,Rochester), edge(Binghamton,Troy),
    edge(Binghamton,Woodstock), edge(Binghamton,NewYork),
    edge(Syracuse,Binghamton), edge(Woodstock,Troy),
    edge(Woodstock,NewYork)
  };
  unsigned int num_edges = sizeof(edge_array) / sizeof(edge);
  cost weights[] = { // estimated travel time (mins)
    96, 134, 143, 65, 115, 133, 117, 116, 74, 56,
    84, 73, 69, 70, 116, 147, 173, 183, 74, 71, 124
  };
  
  
  // create graph
  mygraph_t g(N);
  WeightMap weightmap = get(edge_weight, g);
  for(std::size_t j = 0; j < num_edges; ++j) {
    edge_descriptor e; bool inserted;
    tie(e, inserted) = add_edge(edge_array[j].first,
                                edge_array[j].second, g);
    weightmap[e] = weights[j];
  }
  
  
  // pick random start/goal
  mt19937 gen(time(0));
  vertex start = random_vertex(g, gen);
  vertex goal = random_vertex(g, gen);
  
  
  cout << "Start vertex: " << name[start] << endl;
  cout << "Goal vertex: " << name[goal] << endl;
  
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
  
  
  } catch(found_goal fg) { // found a path to the goal
    list<vertex> shortest_path;
    for(vertex v = goal;; v = p[v]) {
      shortest_path.push_front(v);
      if(p[v] == v)
        break;
    }
    cout << "Shortest path from " << name[start] << " to "
         << name[goal] << ": ";
    list<vertex>::iterator spi = shortest_path.begin();
    cout << name[start];
    for(++spi; spi != shortest_path.end(); ++spi)
      cout << " -> " << name[*spi];
    cout << endl << "Total travel time: " << d[goal] << endl;
    return 0;
  }
  
  cout << "Didn't find a path from " << name[start] << "to"
       << name[goal] << "!" << endl;
  return 0;
  
}


void testChristoph() {

	std::cout << "TEST COUT" << std::endl;

	const int vertexAmount = 2;

	//Prints Edges of Vertex
	Vertex vertices[vertexAmount];
	Vertex* vertexPtrs[vertexAmount];

	for (int i = 0; i < vertexAmount; i++) {

		Vertex vertex(i,0,0);
		std::cout << "Vertex mit ID: " << vertex.getID() << " erstellt." << std::endl;

		vertices[i] = vertex;

		vertexPtrs[i] = &vertices[i];

		std::cout << "VertexPtr Test: VertexOriginalID: " << vertices[i].getID() << ". VertexPtrID: " << vertexPtrs[i]->getID() << std::endl;
	}

	Edge edge(10, 10, std::make_pair(vertexPtrs[0], vertexPtrs[1]));
	Edge edge2(1,10,100);
	Edge * edgePtr;
	Edge * edgePtr2;

	//Assign Pointer to edge
	edgePtr = &edge;
	edgePtr2 = &edge2;

	vertexPtrs[0]->addOutgoingEdges(edgePtr);
	vertexPtrs[1]->addIncomingEdges(edgePtr);
	vertexPtrs[1]->addOutgoingEdges(edgePtr2);

	vertexPtrs[0]->printEdges();
	vertexPtrs[1]->printEdges();

	const int carAmount = 4;

	Car* carsPtr[4];
	Car carStorage[4];

	for (int c = 0; c < carAmount; c++) {
		Car car(c);

		carStorage[c] = car;
	}

	for (int i = 0; i < carAmount; i++) {

		std::cout << "Counter: " << i << std::endl;
		std::cout << "Made car with ID " << carStorage[i].getID() << std::endl;

		carsPtr[i] = &carStorage[i];
		std::cout << "Pointer points to car :" << carsPtr[i]->getID() << std::endl;

		for (int j = 0; j <= i; j++) {

			std::cout << "Car: " << carsPtr[j]->getID() << " on queue." << std::endl;
		}
	}

	//Transfer car


	//Prints Cars in Edge
	edgePtr->printCars();

	system("PAUSE"); //From C
}

