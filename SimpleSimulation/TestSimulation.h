#pragma once
#include "simulation.h"
class TestSimulation
{
public:
	TestSimulation(void);
	~TestSimulation(void);
	
	void run();

private:
	b2World* world;
	b2Body* groundBody;
	b2Body* body;


};

