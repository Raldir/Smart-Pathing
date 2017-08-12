#pragma once
#include "Vertex.h"

class VertexHandler
{
public:

	VertexHandler(vector<std::pair<int, int>> connections, string vertexpath);

	~VertexHandler();

	std::vector<Vertex*> readFile(std::string path);

	void connectVertices(std::vector<Vertex*>);
	

};

