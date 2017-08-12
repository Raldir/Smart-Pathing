#include "ConnectionHandler.h"
#include "VertexHandler.h"
#include <sstream>
#include <fstream>

using namespace std;


ConnectionHandler::ConnectionHandler()
{
}


ConnectionHandler::~ConnectionHandler()
{
}

vector<std::pair<int, int>> ConnectionHandler::readFile(string s){
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
			connectionID.push_back(std::pair <int, int> (static_cast<int> (a), static_cast<int> (b)));
		}
	}
	return connectionID;
}
