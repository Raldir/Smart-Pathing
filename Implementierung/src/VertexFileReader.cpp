#include "VertexFileReader.h"
#include <sstream>
#include <fstream>

using namespace std;

vector<Vertex*> readVertexFile(string s){
	ifstream infile(s);
	vector<Vertex*> vertices;
	string line;
	while (getline(infile, line))
	{
		istringstream iss(line);
		int a;
		float b, c;
		if (!(iss >> a >> b >> c)) { break; } // error
		else {
			//cout << a << ' ' << b << ' ' << c;
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
			(*it)->getVertices().first->printEdges();
	}
	
}

std::map<int, Vertex*> VertexMap(std::vector<Vertex*> vertices) {
	std::map<int, Vertex*> vertexMap;
	std::vector<Vertex*>::iterator it = vertices.begin(); it != vertices.end(); it++){
		vertexMap[(*it)->]
	}
}

