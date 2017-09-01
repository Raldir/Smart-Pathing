#pragma once
#include "Edge.h"
#include "Vertex.h"


	std::vector<Edge*> calculateEdges(std::vector<Vertex*> vertices, std::string path);

	std::vector<std::pair<int, int>> readEdgeFile(std::string path);

	std::map<int, Edge*> edgeMap(std::vector<Edge*>);