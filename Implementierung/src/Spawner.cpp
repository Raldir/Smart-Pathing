#include "Spawner.h"
#include <stdlib.h>


Spawner::Spawner(int id, float x, float y) : Vertex(id, x, y) {
	_spawnRate = rand() % carDensity + 1;
}

//void Spawner::spawnCar(Car * car, RoutingTable routingTable) {
//
//}
