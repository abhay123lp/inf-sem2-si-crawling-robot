#pragma once


class RandomGenerator
{
public:
	static float GenerateUniform (float a=0.0f, float b=1.0f);
	static float GenerateNormal (float mean=0.0f, float signum = 1.0f);

	~RandomGenerator(void);
private:
	RandomGenerator() {}
	static bool initialized;
};

