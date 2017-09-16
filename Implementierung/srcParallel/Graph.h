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
	//Creates a Graph with Vertices and Edges, out of the Input files in "../../dev/OutputMap/nodes"
	Graph();

	std::vector<Spawner*> getSpawner();
	Edge* getEdge(int edgeID);
	std::vector<Edge*> getEdges();
	std::vector<Vertex*> getVertices();
	std::map<int, Vertex*> getVertexMap();

	//Calculates the distance between two Vertices and a given Route
	float distance(int vertex1, int vertex2, std::queue<int> route);

	//Adds Weight to each Edge on the route between the StartVertex and the DEstinationVertex in the position, choosen by heuristik
	void addWeightToTimeTables(int startID, int destID, int currentTimeTableIndex, std::queue<int> route);
	std::map<int, Spawner*> createSpawnerMap();

	//Calculate values to enter into add and remove function for timetables
	static std::pair<int,int> calculateTimetableValues(int intitialTimetableIndex, float toaldistance);

	std::vector<int> createSpawnerIDVector();

	//gets the Summation of the Values of the Timetables on the Route of the car
	int getSumWeightFromTimeTables(int startID, int destID, int currentTimeTableIndex, std::queue<int> route);

	//Calculate the airline distance between two vertices
	float distance_heuristicOverID(size_t start, size_t goal);

	int getNumberVertices();
	int getNumberEdges();
	float getMaxX();
	float getMaxY();

	std::pair<std::map<int, std::vector<int>>, std::map<int, std::vector<int>>> getProcessConnectionVectors();
	std::pair<std::map<int, Vertex*>, std::map<int, Edge*>> getLocalVertexEdgeMaps();
	std::pair<std::vector<Vertex*>, std::vector<Edge*>> getLocalVerticesEdges();
	std::vector<Spawner*> getLocalSpawners();

	//Old function for inserting map at once
	void calculateVertexProcessMap();
	void insertVertexProcessPair(int vertexID, int process);
	void InitLocalVerticesEdges();

private:
	int _rank;

	void initGraphProperties();
	void createTrafficLights();

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
	std::vector<Spawner*> _localSpawners;

	//Local Vertex Map
	std::map<int, Vertex*> _localVertexMap;
	std::map<int, Edge*> _localEdgeMap;

	/*
		first int -> vertex
		second int -> process
	*/
	std::map<int, int> _vertexProcessMap;
	std::map<int, std::vector<int>> _vertexProcessConflictMap;

	/*
		first int -> vertex
		second int -> second
	*/
	std::map<int, int> solveVertexProcessConflicts(std::map<int, std::vector<int>>);
	int solveProcessConflict(std::vector<int> processes);
};


