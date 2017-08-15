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
		float a, b, c;
		if (!(iss >> a >> b >> c)) { break; } // error
		else {
			cout << a << ' ' << b << ' ' << c;
			vertices.push_back(new Vertex(static_cast<int> (a), b, c));
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

