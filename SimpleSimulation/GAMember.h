#pragma once
#include <vector>
#include <iostream>

//#define DBPRINT
#ifdef DBPRINT
	#define debug_print(fmt, ...) fprintf(stderr, fmt, __VA_ARGS__)
#else
	#define debug_print(fmt, ...) 
#endif

using namespace std;

extern int layerCount;
extern int neuronsInLayer[];

class GAMember
{
public:
	GAMember(GAMember & memberA, GAMember & memberB);
	GAMember(vector<float> values);
	~GAMember(void);

	void mutate();

	float getFitness();
	vector<float> getWeights() { return _values; }
	void setProbability(float probability)
	{
		_probability = probability;
	}
	float getProbability() 
	{
		return _probability;
	}

	friend ostream & operator << (ostream & out, const GAMember & m)
	{
		for (int i = 0; i < m._values.size(); i++)
		{
			out << m._values[i] <<" ";
		}

		return out;
	}

private:
	float _fitness;
	bool _fitnessCalculated;
	
	float _probability;
	vector<float> _values;
};

