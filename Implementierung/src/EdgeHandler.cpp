#include "EdgeHandler.h"
#include <sstream>
#include <fstream>

using namespace std;

EdgeHandler::EdgeHandler(string path, vector<Vertex*> vertices)
{
	_path = path;
	calculateEdges(vertices);
}

EdgeHandler::~EdgeHandler()
{
}

std::vector<Edge*> EdgeHandler::calculateEdges(vector<Vertex*> vertices)
{
	vector<std::pair<int, int>> connections = readFile(_path);
	for (vector<pair<int, int>>::iterator it = connections.begin; it != connections.end(); it++) {
		int start = it->first;
		int end = it->second;
		pair<float, float> coordinates1;
		pair<float, float> coordinates2;
		for (vector<Vertex*>::iterator it2 = vertices.begin; it2 != vertices.end(); it2++) {
			if ((*it2)->getID == start){
			coordinates1 = std::pair <float, float>((*it2)->getX, (*it2)->getY);
			}
			else if( (*it2)->getID == end) {
			coordinates2 = std::pair <float, float>((*it2)->getX, (*it2)->getY);
			}
		}
	}

	return std::vector<Edge*>();
}

void EdgeHandler::changePath(string path)
{
	_path = path;
}

vector<std::pair<int, int>> EdgeHandler::readFile(string s) {
	ifstream infile(s);
	vector<std::pair<int, int>> connectionID;
	string line;

	while (getline(infile, line))
	{
		istringstream iss(line);
		float a, b, c;
		if (!(iss >> a >> b)) { break; } // error
		else {
			cout << a << ' ' << b;
			connectionID.push_back(std::pair <int, int>(static_cast<int> (a), static_cast<int> (b)));
		}
	}
	return connectionID;
}
