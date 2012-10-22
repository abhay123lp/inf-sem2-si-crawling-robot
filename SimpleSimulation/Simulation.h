#pragma once
#include <Box2D/Box2D.h>

class Simulation
{
public:
	Simulation();
	virtual ~Simulation();
	virtual void step() = 0;

protected:
	b2World* m_world;
	b2Body* m_groundBody;
};