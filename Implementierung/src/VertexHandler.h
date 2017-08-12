#pragma once
#include "Vertex.h"
#include <vector>

class VertexHandler
{
public:

	VertexHandler(std::vector<std::pair<int, int>> connections, std::string vertexpath);

	~VertexHandler();

	std::vector<Vertex*> readFile(std::string path);

	void connectVertices(std::vector<Vertex*>);
	

};

