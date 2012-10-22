#pragma once
#include "NeuralNetwork.h"
#include "MotorNeuralNetwork.h"
#include "GAMember.h"

#include <random>
#include <iostream>
#include <time.h>
#include <vector>

using namespace std;

extern int layerCount;
extern int neuronsInLayer[];

//#define DBPRINT
#ifdef DBPRINT
	#define debug_print(fmt, ...) fprintf(stderr, fmt, __VA_ARGS__)
#else
	#define debug_print(fmt, ...) 
#endif

class GeneticAlgorithm
{
public:
	GeneticAlgorithm(int membersInPopulation = 20, float mutationProbability = 1.0f) :
		_membersInPopulation(membersInPopulation), _mutationProbability(mutationProbability), _topMember(vector<float>())
	{ 
		int valuesToGenerate = MotorNeuralNetwork::calculateNeuronCount(layerCount, neuronsInLayer);
		debug_print("valuesToGenerate = %d\n", valuesToGenerate);

		for (int i=0; i<_membersInPopulation; i++)
		{
			vector<float> member = generateUniforms(valuesToGenerate, -0.5, 0.5);

			_members.push_back(member); //indirect GAMember constructor from vector<float>
		}

		//set _topMember to proper value
		_topMember = _members[0];
		findTopMember();
	}

	virtual ~GeneticAlgorithm(void) {}

	void evolve();
	GAMember getTopMember()
	{
		return _topMember;
	}



protected:
	int _membersInPopulation;
	float _mutationProbability;
	vector<GAMember> _members;
	GAMember _topMember;
	void findTopMember();

	
	int drawMember();

	vector<float> generateUniforms(int count, float min=0, float max=1);

};

