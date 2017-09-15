/*############################################

Author: Rami Aly and Christoph Hueter
Date : 20.09.17
Libraries and Licences:
Boost Library 1.60, MPI Boost Library 1.60, Open Streetmaps and MPI in use.
All used Maps are licensed under the Open Street Maps License.

#############################################*/

#include "EdgeFileReader.h"
#include <sstream>
#include <fstream>
#include <math.h>

using namespace std;

std::vector<Edge*> calculateEdges(vector<Vertex*> vertices, string path)
{
	vector<std::pair<int, int>> connections = readEdgeFile(path);
	vector<Edge*> edges;
	int id = 0;
	//Create Edges with the vertices descibed in the connections
	for (vector<pair<int, int>>::iterator it = connections.begin(); it != connections.end(); it++) {
		int start = it->first;
		int end = it->second;
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
		//Calculates Length through simple geometry
		float length = sqrt(pow((coordinates1.first - coordinates2.first), 2.0f) 
			+ pow((coordinates1.second - coordinates2.second), 2.0f));
		edges.push_back(new Edge(length, id, std::pair<Vertex*, Vertex*>(startv, endv)));
		//increment id for next edge
		id++;
		edges.push_back(new Edge(length, id, std::pair<Vertex*, Vertex*>(endv, startv)));
		id++;
	}
	return edges;
}


vector<std::pair<int, int>> readEdgeFile(string s) {
	ifstream infile(s);
	vector<std::pair<int, int>> connectionID;
	string line;

	//Iterate through each line of the file
	while (getline(infile, line))
	{
		istringstream iss(line);
		int a, b;
		if (!(iss >> a >> b)) { break; } //error
		else {
			connectionID.push_back(std::pair <int, int>(a, b));
		}
	}
	return connectionID;
}


map<int, Edge*> edgeMap(std::vector<Edge*> edges) {
	map<int, Edge*> edgeMap;
	for (vector<Edge*>::iterator it = edges.begin(); it != edges.end(); it++) {
		edgeMap[(*it)->getID()] = (*it);
	}
	return edgeMap;
}
