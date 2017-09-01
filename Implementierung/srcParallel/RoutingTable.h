#pragma once
#include <queue>
#include <map>
#include <algorithm>
#include <boost/graph/astar_search.hpp>
#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/random.hpp>
#include <boost/random.hpp>
#include <boost/graph/graphviz.hpp>
#include <time.h>
#include <list>
#include <fstream>
#include <math.h>  
#include "Graph.h"
#include <boost/lambda/lambda.hpp>  // _1
#include <boost/lambda/bind.hpp>    // bind()
#include <boost/tuple/tuple_io.hpp>

class Edge;
class Vertex;

typedef std::map<int, std::map<int, std::queue<int>>> RoutingMatrix;
typedef std::map<int, std::map<int, float>> CostMatrix;
typedef std::map<int, std::vector<int>> KNearestNeighborMatrix;


class RoutingTable {

public:
	//TODO Mulitple Routenfindungsalgorithmen implementieren

	RoutingTable(Graph* g, int numberNearestNeighbors);


	void insertRoute(int originID, int destID, std::queue<int> route);
	void removeRoute(int originID, int destID);
	
	void replaceRoute(int originID, int destID, std::queue<int> route);

	int calculateBestGoal(int startID, int destID, int currentTimeTableIndex);

	//ToDO Add  a method that only adds on a specific range
	void addCosts(int startID, int destID, int currentTimeTableIndex);

	//Get Route from origin to destination
	std::queue<int>  getRoute(int originID, int destID);

	void setCost(int originID, int destID, float cost);
	float getCost(int originID, int destID);


private:
	std::queue<int> reverseQueue(std::queue<int> queue);

	static bool comp(const std::pair<int, float> &a, const std::pair<int, float> &b);

	int dimension;

	//Routing Table "Matrix", die die Routen enthält
	/*
		First int --> ID of origin vertex
		Second int --> ID of destination vertex
	*/
	RoutingMatrix routingMatrix;
	CostMatrix costMatrix;
	KNearestNeighborMatrix k_nn;
	Graph* _graph;
};