#include "MotorNeuralNetwork.h"
#include <cmath>
#include <stdio.h>

using namespace std;

//#define DBPRINT
#ifdef DBPRINT
	#define debug_print(fmt, ...) fprintf(stderr, fmt, __VA_ARGS__)
#else
	#define debug_print(fmt, ...) 
#endif

MotorNeuralNetwork::MotorNeuralNetwork(int layerCount, int* neuronCounts, vector<float> weights)
{
	for (int i=1; i<layerCount; i++)
	{
		int inputsCount = neuronCounts[i-1 >= 0 ? i-1 : 0];
		int neuronCount = neuronCounts[i];
		debug_print("inputsCount = %d, neuronCount = %d\n", inputsCount, neuronCount);
		vector<vector<float> > weightsForLayer;
		for (int j=0; j<neuronCount; j++)
		{
			debug_print("before erase weights.size() = %d\n", weights.size());
			vector<float> weightsForNeuron(weights.begin(), weights.begin() + inputsCount + 1);
			weights.erase(weights.begin(), weights.begin() + inputsCount + 1);
			debug_print("after erase weights.size() = %d\n", weights.size());

			weightsForLayer.push_back(weightsForNeuron);
		}

		Layer* sigmoidLayer = new SigmoidLayer(neuronCount, inputsCount, 1);
		sigmoidLayer->setWeights(weightsForLayer);
		

		pushLayer(sigmoidLayer);
	}

}

MotorNeuralNetwork::~MotorNeuralNetwork(void)
{
}


vector<float> MotorNeuralNetwork::calculateOutput(vector<float> input)
{
	vector<float> currentInput = input;
	vector<float> layerResult;

	for (int i=0; i<_layers.size(); i++)
	{

		layerResult = _layers[i]->calculateOutput(currentInput);
		currentInput = layerResult;
	}

	return layerResult;

}

void MotorNeuralNetwork::pushLayer(Layer* layer)
{
	_layers.push_back(layer);
}