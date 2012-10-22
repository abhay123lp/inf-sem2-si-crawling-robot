#include "Simulation.h"
#include "NeuralNetwork.h"
#include "MotorNeuralNetwork.h"
#include "RobotSimulation.h"
#include "GeneticAlgorithm.h"
#include "TestSimulation.h"

#include <iostream>
#include <vector>
#include <Box2D/Box2D.h>
#include <freeglut\freeglut.h>
#include <time.h>

using namespace std;

namespace
{
	int32 width = 640;
	int32 height = 480;
	int32 framePeriod = 16;
	int32 mainWindow;
	float settingsHz = 60.0;
	float32 viewZoom = 1.0f;
	int tx, ty, tw, th;
	bool rMouseDown;
	b2Vec2 lastp;
}

static RobotSimulation * robotSimulation;




vector<float> generate(int count, float min = 0.0f, float max=1.0f)
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

static void Timer(int)
{
	//cout << "Timer\n";
	robotSimulation->updateMotors();
	robotSimulation->step();

	glutSetWindow(mainWindow);
	glutPostRedisplay();
	glutTimerFunc(framePeriod, Timer, 0);
}

static void SimulationLoop()
{
	//cout << "SimulationLoop\n";
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();

	//draw the stuff!
	robotSimulation->draw();

	glutSwapBuffers();
}


int main(int argc, char** argv)
{
	srand(222);

	vector<GAMember> topMembers;
	GeneticAlgorithm* ga = new GeneticAlgorithm(50, 1);

	for (int i = 0; i < 10; i++)
	{
		topMembers.push_back(ga->getTopMember());
		cout << ga->getTopMember().getFitness() << endl;
		ga->evolve();
	}

	GAMember topMember = topMembers[0];
	for (int i=0; i<topMembers.size(); i++)
	{
		if (topMembers[i].getFitness() > topMember.getFitness())
		{
			topMember = topMembers[i];
		}
	}
	robotSimulation = new RobotSimulation(MotorNeuralNetwork(layerCount, neuronsInLayer, topMember.getWeights()));

	//robotSimulation = new RobotSimulation(MotorNeuralNetwork(layerCount, 
	//	neuronsInLayer, generate(MotorNeuralNetwork::calculateNeuronCount(layerCount, neuronsInLayer), -1.0f, 1.0f)));

	/*************************************************************************
	/we have the special one. let's see how it moves
	**************************************************************************/
	cout << "Starting glutInit\n";
	glutInit(&argc, argv);
	glutInitDisplayMode(GLUT_RGBA | GLUT_DOUBLE);
	glutInitWindowSize(width, height);
	char title[32];
	sprintf(title, "Box2D Version %d.%d.%d", b2_version.major, b2_version.minor, b2_version.revision);
	mainWindow = glutCreateWindow(title);

	glutDisplayFunc(SimulationLoop);
	// Use a timer to control the frame rate.
	glutTimerFunc(framePeriod, Timer, 0);

	glutMainLoop();

	system("pause");

	delete robotSimulation;
	return 0;
}