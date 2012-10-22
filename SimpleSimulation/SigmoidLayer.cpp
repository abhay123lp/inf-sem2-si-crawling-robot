#include "SigmoidLayer.h"
#include <cmath>

//#define DBPRINT
#ifdef DBPRINT
	#define debug_print(fmt, ...) fprintf(stderr, fmt, __VA_ARGS__)
#else
	#define debug_print(fmt, ...) 
#endif
SigmoidLayer::~SigmoidLayer(void)
{
}

SigmoidLayer* SigmoidLayer::clone()
{
	SigmoidLayer* result = new SigmoidLayer(_neuronsCount, _inputsCount-1, _beta);
	result->setWeights(_weights);

	return result;
}

vector<float> SigmoidLayer::calculateOutput(vector<float> input)
{
	if (input.size() != _inputsCount - 1)
	{
		cerr << "input count doesnt match! _inputsCount = " << _inputsCount << " input.size() = " << input.size() << endl;
		return vector<float>();
	}

	vector<float> result;

	debug_print("_inputsCount = %d, _neuronCount = %d\n", _inputsCount, _neuronsCount);
	for (int i = 0; i<_neuronsCount; i++)
	{
		//offset default input
		float sum = -1 * _weights[i][0];
		//foreach input
		for (int j = 0; j<_inputsCount-1; j++)
		{
			sum += input[j] * _weights[i][j+1];
		}

		result.push_back(transferFunction(sum));
	}


	return result;
}

float SigmoidLayer::transferFunction(float x)
{
	return (2.0f / (1 + exp(-_beta * x)) - 1.0f);
}