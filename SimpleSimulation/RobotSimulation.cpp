#include "RobotSimulation.h"

#include <freeglut\freeglut.h>
#include <iostream>
#include <cmath>


//#define DBPRINT
#ifdef DBPRINT
	#define debug_print(fmt, ...) fprintf(stderr, fmt, __VA_ARGS__)
#else
	#define debug_print(fmt, ...) 
#endif

#define DEG_TO_RAD (2*3.14/360)

using namespace std;

RobotSimulation::RobotSimulation(MotorNeuralNetwork mnn) : 
	_mnn(mnn)
{
	// Define the gravity vector.
	b2Vec2 gravity(0.0f, -10.0f);

	// Construct a world object, which will hold and simulate the rigid bodies.
	m_world = new b2World(gravity);

	// Define the ground body.
	b2BodyDef groundBodyDef;
	groundBodyDef.position.Set(0.0f, -10.0f);

	m_groundBody = m_world->CreateBody(&groundBodyDef);

	if (m_groundBody == NULL)
	{
		cerr << "groundBody == NULL\n";
	}

	// Define the ground box shape.
	b2PolygonShape groundBox;

	// The extents are the half-widths of the box.
	groundBox.SetAsBox(50.0f, 10.0f);

	// Add the ground fixture to the ground body.
	m_groundBody->CreateFixture(&groundBox, 0.0f);

	{
	 //main robo part
		b2BodyDef bodyDef;
		bodyDef.type = b2_dynamicBody;
		bodyDef.position.Set(0.0f, 2.0f);

		_roboMain = m_world->CreateBody(&bodyDef);

		b2PolygonShape boxShape;
		boxShape.SetAsBox(2.0f, 1.0f);

		b2FixtureDef boxFixtureDef;
		boxFixtureDef.shape = &boxShape;
		boxFixtureDef.density = 1.0f;
		//boxFixtureDef.filter.groupIndex = 1; //no collision


    _roboMain->CreateFixture(&boxFixtureDef);
	}
    
		
	{  //robo arm1
		b2BodyDef bd;
		bd.type = b2_dynamicBody;
		bd.position.Set(2.6f, 3.38f);
		bd.angle = 45.0f * DEG_TO_RAD;

		_roboArm1 = m_world->CreateBody(&bd);

		b2PolygonShape boxShape;
		boxShape.SetAsBox(1.5f, 0.4f);

		b2FixtureDef boxFixtureDef;
		boxFixtureDef.shape = &boxShape;
		boxFixtureDef.density = 1;
		//boxFixtureDef.filter.groupIndex = 1;
		_roboArm1->CreateFixture(&boxFixtureDef);
	}

	{  //robo arm2
		b2BodyDef bd;
		bd.type = b2_dynamicBody;
		bd.position.Set(4.5f, 3.0f);
		bd.angle = -45.0 * DEG_TO_RAD;

		_roboArm2 = m_world->CreateBody(&bd);

		b2PolygonShape boxShape;
		boxShape.SetAsBox(1.5, 0.2);

		b2FixtureDef boxFixtureDef;
		boxFixtureDef.shape = &boxShape;
		boxFixtureDef.density = 1;
		//boxFixtureDef.filter.groupIndex = 1;
		_roboArm2->CreateFixture(&boxFixtureDef);
	}

	{	// joint A: roboMain + roboArm1
		b2RevoluteJointDef revJointDef;
		revJointDef.bodyA = _roboMain;
		revJointDef.bodyB = _roboArm1;
		revJointDef.collideConnected = false;

		revJointDef.localAnchorA.Set(2, 1);
		revJointDef.localAnchorB.Set(-1.5, -0.4);

		revJointDef.referenceAngle = 45 * DEG_TO_RAD;
		revJointDef.enableLimit = true;
		revJointDef.lowerAngle = 0;
		revJointDef.upperAngle = 90 * DEG_TO_RAD;

		revJointDef.enableMotor = true;
		revJointDef.maxMotorTorque = 100;
		revJointDef.motorSpeed = 0;

		_jointA = (b2RevoluteJoint*)m_world->CreateJoint( &revJointDef );
	}

	{	// joint B: roboArm1 + roboArm2
		b2RevoluteJointDef revJointDef;
		revJointDef.bodyA = _roboArm1;
		revJointDef.bodyB = _roboArm2;
		revJointDef.collideConnected = false;

		revJointDef.localAnchorA.Set(1.5, -0.2);
		revJointDef.localAnchorB.Set(-1.5, -0.2);

		revJointDef.referenceAngle = - 45 * DEG_TO_RAD;
		revJointDef.enableLimit = true;
		revJointDef.lowerAngle = -90 * DEG_TO_RAD;
		revJointDef.upperAngle = 0;

			
		revJointDef.enableMotor = true;
		revJointDef.maxMotorTorque = 100;
		revJointDef.motorSpeed =  0.0f;

		_jointB = (b2RevoluteJoint*)m_world->CreateJoint( &revJointDef );
	}

}


void RobotSimulation::updateMotors()
{
	vector<float> input;

	input.push_back(sin(_jointA->GetJointAngle()));
	input.push_back(sin(_jointB->GetJointAngle()));

	input.push_back(sin(_roboMain->GetAngle()));
	

	input.push_back(_roboMain->GetPosition().x - _roboArm1->GetPosition().x);
	input.push_back(_roboMain->GetPosition().x - _roboArm2->GetPosition().x);




	if (input.size() != 6)
	{
		debug_print("size error\n");
	}

	vector<float> motorSpeed = _mnn.calculateOutput(input);
	debug_print("Motor speed = (%f, %f)\n", motorSpeed[0]*2*3.14, motorSpeed[1]*2*3.14);
	

	_jointA->SetMotorSpeed(motorSpeed[0]);
	_jointB->SetMotorSpeed(motorSpeed[1]);

}

void RobotSimulation::step()
{
	
	float32 timeStep = 0.1f;
	int32 velocityIterations = 6;
	int32 positionIterations = 2;


	m_world->Step(timeStep, velocityIterations, positionIterations);

}

void RobotSimulation::run(int nSteps /*=1000*/) 
{
	for (int i = 0; i < nSteps; i++)
	{
		updateMotors();
		step();
	}

}

float RobotSimulation::getDistance()
{
	//debug_print("returning = %f\n", _roboMain->GetPosition().x + _roboArm2->GetPosition().x);
	return _roboMain->GetPosition().x; // + _roboArm2->GetPosition().x;
}

void RobotSimulation::DrawSolidPolygon(const b2Vec2* vertices, int32 vertexCount, const b2Color& color, float scale /*=1.0f*/)
{
	glEnable(GL_BLEND);
	glBlendFunc (GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
	glColor4f(0.5f * color.r, 0.5f * color.g, 0.5f * color.b, 0.5f);
	glBegin(GL_TRIANGLE_FAN);
	for (int32 i = 0; i < vertexCount; ++i)
	{
		glVertex2f(vertices[i].x * scale, vertices[i].y * scale);
	}
	glEnd();
	glDisable(GL_BLEND);

	glColor4f(color.r, color.g, color.b, 1.0f);
	glBegin(GL_LINE_LOOP);
	for (int32 i = 0; i < vertexCount; ++i)
	{
		glVertex2f(vertices[i].x, vertices[i].y);
	}
	glEnd();
}


void RobotSimulation::DrawPolygon(b2Fixture* fixture, const b2Transform& xf, const b2Color& color)
{

	b2PolygonShape* poly = (b2PolygonShape*)fixture->GetShape();
	int32 vertexCount = poly->m_vertexCount;
	b2Assert(vertexCount <= b2_maxPolygonVertices);
	b2Vec2 vertices[b2_maxPolygonVertices];

	for (int32 i = 0; i < vertexCount; ++i)
	{
		vertices[i] = b2Mul(xf, poly->m_vertices[i]);
	}

	DrawSolidPolygon(vertices, vertexCount, color, 0.1);
}

void RobotSimulation::draw()
{
	DrawPolygon(&(_roboMain->GetFixtureList()[0]), _roboMain->GetTransform(), b2Color(1, 0, 0)); 
	DrawPolygon(&(_roboArm1->GetFixtureList()[0]), _roboArm1->GetTransform(), b2Color(0, 1, 0)); 
	DrawPolygon(&(_roboArm2->GetFixtureList()[0]), _roboArm2->GetTransform(), b2Color(0, 0, 1)); 
}