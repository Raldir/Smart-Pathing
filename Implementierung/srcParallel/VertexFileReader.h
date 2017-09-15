#pragma once
#include "Vertex.h"
#include <vector>
#include "Edge.h"


//Reads the Vertices of a file accessed through given path and store them in a Vector
std::vector<Vertex*> readVertexFile(std::string path);

//returns a Map with the ID of the vertex as the key
std::map<int, Vertex*> vertexMap(std::vector<Vertex*>);

//Sets the ingoing and outgoing Edges for each vertex
void connectVertices(std::vector<Edge*>);


