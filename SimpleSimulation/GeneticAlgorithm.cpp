#include "GeneticAlgorithm.h"
#include "RobotSimulation.h"
#include <iostream>
#include <random>
#include <time.h>

//#define DBPRINT
#ifdef DBPRINT
	#define debug_print(fmt, ...) fprintf(stderr, fmt, __VA_ARGS__)
#else
	#define debug_print(fmt, ...) 
#endif

using namespace std;

int layerCount = 3;
int neuronsInLayer[] = {5, 3, 2};

vector<float> GeneticAlgorithm::generateUniforms(int count, float min, float max)
{
	vector<float> result;
	for (int i=0; i<count; i++)
	{
		float uniform = ((float)rand())/((float)(RAND_MAX + 1));
		float randomValue = (max-min)*(uniform)+min;
		debug_print("random value = %.3f\n", randomValue);
		result.push_back(randomValue);
	}

	return result;
}

void GeneticAlgorithm::findTopMember()
{
	debug_print("findTopMember\n");
	_topMember = _members[0];
	for (int i=1; i<_members.size(); i++)
	{
		if(_topMember.getFitness() < _members[i].getFitness())
		{
			debug_print("changing top member\n");
			_topMember = _members[i];
		}
	}
}

void GeneticAlgorithm::evolve()
{
	float fitnessSum = 0.0f;
	//sum all fitnesses
	for (int i = 0; i < _members.size(); i++)
	{
		debug_print("summing fitness..\n");
		fitnessSum += _members[i].getFitness();
		
	}

	debug_print("got fitness sum: %f\n", fitnessSum);

	//set proper probabilities
	for (int i = 0; i < _members.size(); i++)
	{
		if (fitnessSum < 0.0005) //fitnessSum == 0
		{
			_members[i].setProbability(1.0f / ((float)_members.size()));
		}
		else
		{
			_members[i].setProbability(_members[i].getFitness() / fitnessSum);
		}
	}

	vector<GAMember> nextGeneration;

	for (int i = 0; i < _members.size() / 3; i++)
	{
		//lets draw parents indexes
		debug_print("about to draw parent indexes\n");

		int parentAIndex = drawMember();
		int parentBIndex = drawMember();

		debug_print("parentIndexA = %d, parentBIndex = %d", parentAIndex, parentBIndex);

		while (parentAIndex == parentBIndex)
		{
			parentBIndex = drawMember();
		}

		cout<<"A.fitness = " << _members[parentAIndex].getFitness() <<"B.fitness= " << _members[parentBIndex].getFitness() << endl;

		GAMember offspring (_members[parentAIndex], _members[parentBIndex]);
		offspring.mutate();

		nextGeneration.push_back(offspring);
	}

	//lets add some random ones!
	while (nextGeneration.size() != _members.size())
	{
		int valuesToGenerate = MotorNeuralNetwork::calculateNeuronCount(layerCount, neuronsInLayer);

		GAMember randomOffspring(generateUniforms(valuesToGenerate, -0.5, 0.5));

		nextGeneration.push_back(randomOffspring);
	}

	//update our population
	_members = nextGeneration;

	//set _topMember to proper value
	findTopMember();
}


int GeneticAlgorithm::drawMember()
{
	float U = generateUniforms(1)[0];
	int i = 0;

	debug_print("U = %f\n", U);

	while(U > _members[i].getProbability())
	{
		debug_print("_members[i].getProbability() = %f\n", _members[i].getProbability());
		U -= _members[i].getProbability();
		i++;
	}

	return i;
}
