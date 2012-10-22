#include "Simulation.h"

#include <iostream>

using namespace std;

Simulation::Simulation()
{
	//cout << "Simulation constructor\n";
	b2Vec2 gravity(0.0f, -10.0f);

	m_world = new b2World (gravity);

	b2BodyDef groundBodyDef;
	groundBodyDef.position.Set(0.0f, -10.0f);

	m_groundBody = m_world->CreateBody(&groundBodyDef);

	b2PolygonShape groundBox;
	groundBox.SetAsBox(50.0f, 10.0f);
	m_groundBody->CreateFixture(&groundBox, 0.0f);

}

Simulation::~Simulation()
{
	//cout << "Simulation destructor\n";

	delete m_world;
	m_world = NULL;
}

//komentarz bez znaczenia