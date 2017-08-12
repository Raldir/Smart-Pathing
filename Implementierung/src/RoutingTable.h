#pragma once

#include <vector>

class Graph;
class Spawner;

class RoutingTable {

public:

	RoutingTable(int n) : dimension(n) {};

private:
	int dimension;

	std::vector<Spawner*> costMatrix;
};