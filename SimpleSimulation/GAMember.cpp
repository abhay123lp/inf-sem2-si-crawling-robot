#include "GAMember.h"
#include "RobotSimulation.h"
#include "MotorNeuralNetwork.h"
#include "RandomGenerator.h"

#include <time.h>
#include <iostream>

//#define DBPRINT
#ifdef DBPRINT
	#define debug_print(fmt, ...) fprintf(stderr, fmt, __VA_ARGS__)
#else
	#define debug_print(fmt, ...) 
#endif

using namespace std;

GAMember::GAMember(GAMember & memberA, GAMember & memberB) : 
	_fitness(0), _fitnessCalculated(false), _probability(0)
{
	debug_print("GAMember constructor");

	for (int i=0; i<memberA._values.size(); i++)
	{
		_values.push_back(memberA._values[i] + RandomGenerator::GenerateUniform() * (memberB._values[i] - memberA._values[i]));
	}
}

GAMember::GAMember(vector<float> values) : 
	_values(values), _fitness(0), _fitnessCalculated(false),  _probability(0)
{
		debug_print("GAMember values constructor\n");
}

GAMember::~GAMember(void)
{
}

float GAMember::getFitness()
{
	if (_fitnessCalculated)
	{
		debug_print("fitness already calculated, returning %f\n", _fitness);
		return _fitness;
	}
	else
	{
		debug_print("Fitness NOT calculated, running simulation\n");
		MotorNeuralNetwork mnn(layerCount, neuronsInLayer, _values);
		RobotSimulation rs(mnn);

		rs.run();
		debug_print("After simulation, about to get distance\n");
		_fitness = max(0.0f, rs.getDistance());

		debug_print("Setting _fitnessCalculated = true\n");
		_fitnessCalculated = true;

		return _fitness;
	}
}

void GAMember::mutate()
{
	for (int i=0; i<_values.size(); i++)
	{
		_values[i] += RandomGenerator::GenerateNormal(0.0, 1.0);
	}
}

