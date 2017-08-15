#pragma once
#include "Edge.h";
#include "Vertex.h";

class Graph
{
public:
	Graph();
	~Graph();

private:
	std::vector<Vertex*> filterSpawner();

	std::vector<Vertex*> _spawner;


	std::vector<Edge*> _edges;
	std:: vector<Vertex*> _vertices;

};

