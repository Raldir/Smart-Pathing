#pragma once
#include "Edge.h"
#include "Vertex.h"


/*
Creates the Edges and stores the right vertices in them
*/
	std::vector<Edge*> calculateEdges(std::vector<Vertex*> vertices, std::string path);

	/*
	Reads an Edgefile and stores the connections into a vector
	*/
	std::vector<std::pair<int, int>> readEdgeFile(std::string path);

	/*
	returns a Map with the edge id as the key to the corresponding edge
	*/
	std::map<int, Edge*> edgeMap(std::vector<Edge*>);