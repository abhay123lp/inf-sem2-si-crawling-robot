#pragma once
#include <vector>
#include <iostream>

using namespace std;

class Layer
{
public:
	Layer(int neuronsCount, int inputsCount) : //add one for offset input
	  _neuronsCount(neuronsCount), _inputsCount(inputsCount+1)
	  {  }

	virtual ~Layer(void);

	virtual Layer* clone() = 0;

	virtual vector<float> calculateOutput(vector<float> input) = 0; 

	void setWeights (vector<vector<float>> weights)
	{
		_weights = weights;
	}

protected:
	virtual float transferFunction(float x) = 0;
	int _neuronsCount;
	int _inputsCount;
	vector<vector<float>> _weights;
};

