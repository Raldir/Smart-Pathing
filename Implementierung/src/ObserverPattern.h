#pragma once

#include <iostream>
#include <vector>

class Observer {

public:
	virtual void update(Edge* edge);
};

class Subject {

private:
	std::vector<Edge*>;

public:
	virtual void registerObserver(Edge* edge) = 0;
	virtual void removeObserver(Edge* edge) = 0;
	virtual void notify(Car* car) = 0;
};
