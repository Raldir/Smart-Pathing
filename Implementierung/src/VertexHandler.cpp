#include "VertexHandler.h"
#include <sstream>
#include <fstream>

using namespace std;

VertexHandler::VertexHandler(vector<pair<int, int>> connections, string vertexpath)
{
	vector<Vertex*> vertices = readFile(vertexpath);
	for (vector<pair<int, int>>::iterator it = connections.begin; it != connections.end(); it++) {
		int start = it->first;
		int end = it->second;
		for (vector<Vertex*>::iterator it2 = vertices.begin; it2 != vertices.end(); it2++) {
		//*it2->addOutgoingEdges(start);
		}
	}
}


VertexHandler::~VertexHandler()
{
}


vector<Vertex*> VertexHandler::readFile(string s){
	ifstream infile(s);
	vector<Vertex*> vertices;
	string line;
	while (getline(infile, line))
	{
		istringstream iss(line);
		float a, b, c;
		if (!(iss >> a >> b >> c)) { break; } // error
		else {
			cout << a << ' ' << b << ' ' << c;
			vertices.push_back(new Vertex(static_cast<int> (a), b, c));
		} 
	}
	return vertices;
}

void VertexHandler::connectVertices(std::vector<Vertex*>)
{

}

