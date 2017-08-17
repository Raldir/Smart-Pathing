#pragma once
#include "Vertex.h"
#include <vector>
#include "Edge.h"


std::vector<Vertex*> readVertexFile(std::string path);

std::map<int, Vertex*> vertexMap(std::vector<Vertex*>);

void connectVertices(std::vector<Edge*>);


