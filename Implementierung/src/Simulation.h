#pragma once
class Simulation
{
public:
	Simulation();
	void init();
	void nextTick();
	~Simulation();


private:
	void initSpawner();

};

