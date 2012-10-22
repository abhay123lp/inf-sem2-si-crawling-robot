#pragma once
#include "Layer.h"
#include "SigmoidLayer.h"
#include <vector>
#include <iostream>

using namespace std;

class NeuralNetwork
{
public:
	NeuralNetwork()
	  {  } 

	NeuralNetwork(NeuralNetwork & nn)
	{
		for (int i = 0; i<nn._layers.size(); i++)
		{
			_layers.push_back(nn._layers[i]->clone());

		}
		
	}

	virtual ~NeuralNetwork(void);

	virtual vector<float> calculateOutput(vector<float> input) = 0;


	

protected:
	vector<Layer*> _layers;
};

