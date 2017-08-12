#pragma once

#include "ObserverPattern.h"


class Spawner : ObserverVertex {

public:

	Spawner(int id) : _ID(id) {};

private:

	int _ID;

};
