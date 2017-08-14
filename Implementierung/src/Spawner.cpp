#include "Spawner.h"


Spawner::Spawner(int id, float x, float y) : Vertex(id, x, y) {

}

void Spawner::SpawnCar(Car * car) {

}

int Spawner::getID()
{
	return _ID;
}
