#pragma once
#include "Edge.h"
#include "Vertex.h"

class EdgeHandler
{
public:
	EdgeHandler(string path, vector<Vertex*> vertices);

	~EdgeHandler();
	std::vector<Edge*> calculateEdges(vector<Vertex*> vertices);
	void changePath(string path);

	std::vector<std::pair<int, int>> readFile(std::string path);

private:
	std::vector<Edge*> _Edges;
	string _path;
};

