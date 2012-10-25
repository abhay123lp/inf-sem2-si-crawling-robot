#include "RandomGenerator.h"
#include <time.h>
#include <stdlib.h>
#include <cmath>

bool RandomGenerator::initialized = false;

RandomGenerator::~RandomGenerator(void)
{
}

float RandomGenerator::GenerateUniform (float a, float b)
{
	if (!initialized)
	{
		srand(time(NULL));
		initialized = true;
	}

	float uniform = rand() / (float)RAND_MAX;

	uniform *= (b-a);
	uniform += a;

	return uniform;
}

float RandomGenerator::GenerateNormal(float mean, float signum)
{
	if (!initialized)
	{
		srand(time(NULL));
		initialized = true;
	}

	float uniform = GenerateUniform(), uniform2 = GenerateUniform();

	float normal = std::sqrt(-2.0 * log(uniform)) * std::cos(2*3.14*uniform2);

	normal *= signum;
	normal += mean;

	return normal;
}