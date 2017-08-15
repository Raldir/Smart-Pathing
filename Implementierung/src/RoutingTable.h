#pragma once

#include <iostream>
#include <vector>
#include <queue>
#include <map>

class Graph;
class Spawner;
class Vertex;

typedef std::map<int, std::map<int, std::queue<int>>> RoutingMatrix;

class RoutingTable {

public:
	//TODO Mulitple Routenfindungsalgorithmen implementieren

	RoutingTable() {}

	void insertRoute(int originID, int destID, std::queue<int> route);
	void removeRoute(int originID, int destID);
	
	void replaceRoute(int originID, int destID, std::queue<int> route);

	//Get Route from origin to destination
	std::queue<int> getRoute(int originID, int destID);

private:
	int dimension;

	//Routing Table "Matrix", die die Routen enthält
	/*
		First int --> ID of origin vertex
		Second int --> ID of destination vertex
	*/
	RoutingMatrix routingMatrix;
};