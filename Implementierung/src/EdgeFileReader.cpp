#include "EdgeFileReader.h"
#include <sstream>
#include <fstream>

using namespace std;

std::vector<Edge*> calculateEdges(vector<Vertex*> vertices, string path)
{
	vector<std::pair<int, int>> connections = readEdgeFile(path);
	vector<Edge*> edges;
	int id = 0;
	for (vector<pair<int, int>>::iterator it = connections.begin(); it != connections.end(); it++) {
		int start = it->first;
		int end = it->second;
		//cout << start << " " << end;
		pair<float, float> coordinates1;
		pair<float, float> coordinates2;
		Vertex* startv = NULL;
		Vertex* endv = NULL;
		for (vector<Vertex*>::iterator it2 = vertices.begin(); it2 != vertices.end(); it2++) {
			if ((*it2)->getID() == start){
				startv = (*it2);
				coordinates1 = std	::pair <float, float>((*it2)->getX(), (*it2)->getY());
			}
			else if( (*it2)->getID() == end) {
				endv = (*it2);
				coordinates2 = std::pair <float, float>((*it2)->getX(), (*it2)->getY());
			}
		}
		float length = sqrt(pow((coordinates1.first - coordinates2.first), 2.0f) 
			+ pow((coordinates1.second - coordinates2.second), 2.0f));
		edges.push_back(new Edge(length, id, std::pair<Vertex*, Vertex*>(startv, endv)));
		//cout << length << " " << id << " " << startv->getID() << " " << endv->getID() <<"\n";
		id++;
	}
	return edges;
}


vector<std::pair<int, int>> readEdgeFile(string s) {
	ifstream infile(s);
	vector<std::pair<int, int>> connectionID;
	string line;

	while (getline(infile, line))
	{
		istringstream iss(line);
		int a, b;
		if (!(iss >> a >> b)) { break; } // error
		else {
			//cout << a << ' ' << b;
			connectionID.push_back(std::pair <int, int>(a, b));
		}
	}
	return connectionID;
}

map<int, Edge*> edgeMap(std::vector<Edge*> edges) {
	map<int, Edge*> edgeMap;
	for (vector<Edge*>::iterator it = edges.begin(); it != edges.end(); it++) {
		edgeMap[(*it)->getID] = (*it);
	}
	return edgeMap;
}
