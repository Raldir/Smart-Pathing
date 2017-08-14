#pragma once

#include <vector>

class ConnectionHandler
{
public:
	ConnectionHandler();
	~ConnectionHandler();

	std::vector<std::pair<int, int>> readFile(std::string path);
};

