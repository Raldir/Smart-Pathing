/*############################################

Author: Rami Aly and Christoph Hueter
Date : 20.09.17
Libraries and Licences:
Boost Library 1.60, MPI Boost Library 1.60, Open Streetmaps and MPI in use.
All used Maps are licensed under the Open Street Maps License.

#############################################*/

#include "VertexFileReader.h"
#include <sstream>
#include <fstream>

using namespace std;

vector<Vertex*> readVertexFile(string s){
	ifstream infile(s);
	vector<Vertex*> vertices;
	string line;
	//Iterate through each line of the given file
	while (getline(infile, line))
	{
		istringstream iss(line);
		int a;
		float b, c;
		if (!(iss >> a >> b >> c)) { break; } // error
		else {
			vertices.push_back(new Vertex(a, b, c));
		} 
	}
	return vertices;
}

void connectVertices(vector<Edge*> edges)
{
	//TODO Struktur überlegen, die Zirkelschluss verhindert
	for (vector<Edge*>::iterator it = edges.begin(); it != edges.end(); it++) {
			(*it)->getVertices().first->addOutgoingEdges(*it);
			(*it)->getVertices().second->addIncomingEdges(*it);
	}
	
}

map<int, Vertex*> vertexMap(std::vector<Vertex*> vertices) {
	map<int, Vertex*> vertexMap;
	for(vector<Vertex*>::iterator it = vertices.begin(); it != vertices.end(); it++){
		vertexMap[(*it)->getID()] = (*it);
	}
	return vertexMap;
}

