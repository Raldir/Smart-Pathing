#pragma once

#include "ObserverPattern.h"


class Spawner : ObserverVertex {

public:

	Spawner(int id, float x, float y);

	void SpawnCar(Car* car);

private:
};
