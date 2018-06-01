#include "aria.h"
#include <iostream>
#include <fstream>
using namespace std;

int main(int argc, char **argv) {

	Aria::init();
	ArRobot robot;
	ArPose pose;

	ArArgumentParser argParser(&argc, argv);
	argParser.loadDefaultArguments();

	ArRobotConnector robotConnector(&argParser, &robot);
	if (robotConnector.connectRobot())
		std::cout << "Robot connected!" << std::endl;
	robot.runAsync(false);
	robot.lock();
	robot.enableMotors();
	robot.unlock();

	ArSensorReading *sonarSensor[8];

	// initializing speed and desired distance
	const int baseVel = 100;
	const int targetDist = 450;

	//setting PID parameters
	double P = 0.6;
	double I = 0.001;
	double D = 0.05;

	double e;
	double ei = 0;
	double eprev = 0;
	double ed;

	double currentDist;
	double PID;
	double leftVel;
	double rightVel;
	double sum;

	int time = 0;

	ofstream wfile;
	wfile.open("PID.txt");
	wfile << "";
	wfile.close();

	// give robot time to connect to sensors
	ArUtil::sleep(1000);

	while (true)
	{

		ArUtil::sleep(10);

		// storing sonar readings in array
		// just need the right side sonars for this assignment

		double sonarRange[8];


		for (int t = 6; t <= 7; t++) {

				sonarSensor[t] = robot.getSonarReading(t);
				sonarRange[t] = sonarSensor[t]->getRange();
				ArUtil::sleep(10);

		}


		// set current distance as the average of the sonar's readings
		// constants are added as they represent the distance from sonars to centre of robot

		//currentDist = (sonarRange[6] + sonarRange[7] + 115 + 130) / 2;


		//use minsonars as distance
		if (sonarRange[6] < sonarRange[7])
			currentDist = sonarRange[6];

		else
			currentDist = sonarRange[7];

		e = targetDist - currentDist;
		ed = e - eprev;
		eprev = e;

		if ((e < 50) && (e > -50))
			ei = e + ei;

		PID = P*e + I*ei + D*ed;
		if (PID > 50) PID = 50;
		if (PID < -50) PID = -50;
		leftVel = baseVel - PID;
		rightVel = baseVel + PID;
		robot.setVel2(leftVel, rightVel);

		printf("\n leftVel = %d ", leftVel);
		printf("\n output = %.5f ", rightVel);
		printf("\n e = %.5f ", e);
		printf("\n ei = %.5f ", ei);
		printf("\n ed = %.5f ", ed);
		//printf("\n frsonar = %.5f", sonarRange[6]);
		//printf("\n rsonar = %.5f", sonarRange[7]);

		ofstream wfile;
		wfile.open("PID.txt", std::ios::app);
		wfile << time << "	" << currentDist << endl;
		wfile.close();

		time = time + 1;

	}

	return 0;
}


