#pragma once
#include "layer.h"

using namespace std;

class SigmoidLayer :
	public Layer
{
public:
	SigmoidLayer(int neuronsCount, int inputsCount, float beta = 5) :
	  Layer(neuronsCount, inputsCount), _beta(beta) {}

	~SigmoidLayer(void);

	virtual SigmoidLayer* clone();

	virtual vector<float> calculateOutput(vector<float> input);

protected:
	virtual float transferFunction(float x);
	float _beta;

};

