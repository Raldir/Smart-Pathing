#pragma once
#include <queue>
#include <map>
#include <algorithm>
#include <time.h>
#include <limits.h>
#include <math.h>
#include "Graph.h"

class Edge;
class Vertex;

//Structures are build like this to ensure easy access to the route through origin and destination
typedef std::map<int, std::map<int, std::queue<int>>> RoutingMatrix;
typedef std::map<int, std::map<int, std::queue<int>>>::iterator RoutingMatrixIt;

typedef std::map<int, std::map<int, float>> CostMatrix;
typedef std::map<int, std::vector<int>> KNearestNeighborMatrix;


class RoutingTable {

public:

	//Constructor of RoutingTable
	RoutingTable(Graph* g, int numberNearestNeighbors);
	RoutingTable(Graph* g, int numberNearestNeighbors, std::vector<int> spawner);
	RoutingTable(Graph* graph, int numberNearestNeighbors, std::map<int, std::map<int, std::queue<int>>> routingMatrixP,
		std::map<int, std::map<int, float>> costs);

	//Fundamental insert/remove/insert operations for the Routingtable
	void insertRoute(int originID, int destID, std::queue<int> route);
	void removeRoute(int originID, int destID);
	void replaceRoute(int originID, int destID, std::queue<int> route);

	//Calculates if the inserted or an nearby alternative path is the optimal one
	int calculateBestGoal(int startID, int destID, int currentTimeTableIndex);
	void calculateRoutes(std::vector<Spawner*> _spawner);

	/*
	Parallelism which calculates each Route parallel but the number of Routes sequential
	*/
	void calculateRoutesParallel(std::vector<Spawner*> _spawner);
	
	void addCosts(int startID, int destID, int currentTimeTableIndex);

	//Get Route from origin to destination
	std::queue<int>  getRoute(int originID, int destID);

	/*Converts RoutingMatrix to a 2 dimensional Vector with effektive no information loose, since every route stores
	its start and destination*/
	std::vector<std::vector<int>> getRoutingMatrix();

	/*
	Converts Costmatrix to a twodimensional Vector. Each vector stores the start, end and the cost to travel from start to the goal
	*/
	std::vector<std::vector<int>> getRoutingCosts();

	/*
	Converts K nearest neighbor matrix to twodimensional Vector
	*/
	std::vector<std::vector<int>> getKNearestMatrix();


	//Calculates the k nearest neighbors sequential
	void calculateKNearest();

	//Set/get cost from origin to dest in Costmatrix
	void setCost(int originID, int destID, float cost);
    float getCost(int originID, int destID);


	std::pair<std::map<int, std::vector<int>>, std::map<int, std::vector<int>>> getProcessConnectionVectors();
	
	//Returns incomingConnection and outgoingConnection for every connected process
	void insertProcessRoutes(std::pair<int, std::vector<std::pair<int, int>>>);



private:
	//Reverse Queue
	std::queue<int> reverseQueue(std::queue<int> queue);

	//Comperator that compares two pairs by it second value in ascending
	static bool comp(const std::pair<int, float> &a, const std::pair<int, float> &b);


	int dimension;
	int _numberNearestNeighbors;

	//Routing Table "Matrix", die die Routen enthält
	/*
		First int --> ID of origin vertex
		Second int --> ID of destination vertex
	*/
	RoutingMatrix routingMatrix;
	CostMatrix costMatrix;
	KNearestNeighborMatrix k_nn;
	Graph* _graph;
	//Contains every route inside this process
	std::vector<std::vector<int>> processRoutingMatrix;

	std::map<int, std::vector<std::pair<int, int>>> processRoutesMap;
};
