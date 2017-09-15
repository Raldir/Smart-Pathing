#pragma once
#include <time.h>
#include <list>
#include <fstream>
#include <math.h>    // for sqrt
#include "Edge.h"
#include "Vertex.h"
#include "Spawner.h"


class Graph
{
public:
	Graph();

	std::vector<Spawner*> getSpawner();
	Edge* getEdge(int edgeID);
	std::vector<Edge*> getEdges();
	std::vector<Vertex*> getVertices();
	std::map<int, Vertex*> getVertexMap();
	float distance(int vertex1, int vertex2, std::queue<int> route);
	void addWeightToTimeTables(int startID, int destID, int currentTimeTableIndex, std::queue<int> route);
	std::map<int, Spawner*> createSpawnerMap();
	static std::pair<int,int> calculateTimetableValues(int intitialTimetableIndex, float toaldistance);
	std::vector<int> createSpawnerIDVector();
	int getSumWeightFromTimeTables(int startID, int destID, int currentTimeTableIndex, std::queue<int> route);
	float distance_heuristicOverID(size_t start, size_t goal);
	int getNumberVertices();
	int getNumberEdges();
	float getMaxX();
	float getMaxY();

private:
	int _rank;

	void initGraphProperties();
	void createTrafficLights();

	/*
	Returns incomingConnection and outgoingConnection for every connected process
	Also correct potential conflicts
	*/
	void insertVertexProcessMap(std::map<int, std::vector<int>>);
	void InitLocalVerticesEdges();

	std::pair<std::map<int, std::vector<int>>, std::map<int, std::vector<int>>> getProcessConnectionVectors();
	std::pair<std::map<int, Vertex*>, std::map<int, Edge*>> getLocalVertexEdgeMaps();
	std::pair<std::vector<Vertex*>, std::vector<Edge*>> getLocalVerticesEdges();

	float _maxX;
	float _maxY;
	std::vector<Spawner*> _spawner;
	std::vector<Edge*> _edges;
	std::vector<Vertex*> _vertices;
	std::map<int, Edge*> _edgeMap;
	std::map<int, Vertex*> _vertexMap;

	//std::vector<int> localVerticesID;
	//std::vector<int> localEdgesID;

	//Direct pointers to vertices and edges
	std::vector<Vertex*> _localVertices;
	std::vector<Edge*> _localEdges;

	//Local Vertex Map
	std::map<int, Vertex*> _localVertexMap;
	std::map<int, Edge*> _localEdgeMap;

	/*
		first int -> vertex
		second int -> process
	*/
	std::map<int, int> _vertexProcessMap;

	/*
		first int -> vertex
		second int -> second
	*/
	std::map<int, int> solveVertexProcessConflicts(std::map<int, std::vector<int>>);
	int solveProcessConflict(std::vector<int> processes);
};


