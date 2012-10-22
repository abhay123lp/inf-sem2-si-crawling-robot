#pragma once
#include "NeuralNetwork.h"
#include "SigmoidLayer.h"

class MotorNeuralNetwork :
	public NeuralNetwork
{
public:
	//weights obtained from genetic algorithm
	MotorNeuralNetwork(int layerCount, int* neuronCounts, vector<float> weights);
	~MotorNeuralNetwork(void);
	virtual std::vector<float> calculateOutput(std::vector<float> input);

	static int calculateNeuronCount(int layerCount, int* neuronsInLayer)
	{
		int result = 0;

		for (int i=1; i<layerCount; i++)
		{
			result+= (neuronsInLayer[i-1]+1)*neuronsInLayer[i];
		}

		return result;
	}

	const static int layerCount;
	const static int neuronsInLayer[];

private:
	void pushLayer(Layer* layer);

};

