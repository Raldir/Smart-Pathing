#include <iostream>
#include "RoutingTable.h"

void RoutingTable::insertRoute(int originID, int destID, std::queue<int> route)
{
	//Pushes originID into first map and int queue pair into the second
	routingMatrix[originID][destID] = route;
	
	//Pushes queue on symmetrical pair
	routingMatrix[destID][originID] = route;

	std::cout << "Added queue from " << originID << " to " << destID << " and reverse." << std::endl;
}

void RoutingTable::removeRoute(int originID, int destID)
{
	//Erases queue origin to destination and reverse
	routingMatrix[originID].erase(destID);
	routingMatrix[destID].erase(originID);

	std::cout << "Deleted queue from " << originID << " to " << destID << " and reverse." << std::endl;
}

void RoutingTable::replaceRoute(int originID, int destID, std::queue<int> route)
{
	removeRoute(originID, destID);

	insertRoute(originID, destID, route);

}

//Gibt Route zwischen origin und destination aus
std::queue<int> RoutingTable::getRoute(int originID, int destID) {

	std::queue<int> queue;

	RoutingMatrix::iterator iterOrigin = routingMatrix.find(originID);

	if (iterOrigin != routingMatrix.end()) {
		//Map where destinationID and queues are stored
		std::map<int, std::queue<int>> destinationMap = iterOrigin->second;

		queue = destinationMap.find(destID)->second;
	}
	else {
		std::cout << "No queue found in Routing Table from" << originID << " to " << destID << std::endl;
		return std::queue<int>();
	}

	return queue;
}
