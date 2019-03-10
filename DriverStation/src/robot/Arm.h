#pragma once

#include "RobotIO.h"
#include "../utils/PID.h"
#include "Controls.h"
#include "Constants.h"
#include <iostream>
#include <cmath>
#include <ctime>

class Arm {
public:
	Arm();
	void periodic(const RobotIn& rIn, RobotOut& rOut);

	// converts from [-1.0, 1.0] to [0,255] for the arduino
	static char SOut(double out, double scale=1.0) { return (char)((out*scale + 1.0)/2.0*255.0); }
	// reverses from [255,0] to [0,255] for the arduino
	static char Rev(char out) { return 255 - out; }
	// positive modulus
	static double PMod(double n, double i) { return n - i*floor(n / i); }

private:
	PID waistPID;
	PID shoulderPID;
	PID elbowPID;

	bool initAngles = false;
	bool lastCtrlManual = false;
	bool manualCtrl = true;

	clock_t lastTime;


	double xTarget = 1.0;
	double yTarget = 0.0;
	double zTarget = 0.0;

	double waistTarget;
	double shoulderTarget;
	double elbowTarget;

	const double upperArmLen = 5.0;
	const double foreArmLen = 5.0;

	bool lastCtrlVacuum = false;

	void forwardKinematics();
	void inverseKinematics();
};
