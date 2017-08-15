#include <iostream>
#include "RoutingTable.h"


void RoutingTable::insertRoute(int originID, int destID, std::queue<int> route)
{

}

void RoutingTable::removeRoute(int originID, int destID)
{

}

void RoutingTable::replaceRoute(int originID, int destID, std::queue<int> route)
{
	removeRoute(originID, destID);

	insertRoute(originID, destID, route);

}

//Gibt Route zwischen origin und destination aus
std::queue<int> RoutingTable::getRoute(int originID, int destID) {

	std::queue<int> queue;

	auto iterOrigin = routingMatrix.find(originID);

	if (iterOrigin != routingMatrix.end()) {
		//Map where destinationID and queues are stored
		std::map<int, std::queue<int>> destinationMap = iterOrigin->second;

		queue = destinationMap.find(destID)->second;
	}

	return queue;
}
