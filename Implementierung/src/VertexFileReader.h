#pragma once
#include "Vertex.h"
#include <vector>
#include "Edge.h"


std::vector<Vertex*> readVertexFile(std::string path);

void connectVertices(std::vector<Edge*>);


