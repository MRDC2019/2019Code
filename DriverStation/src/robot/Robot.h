#pragma once

#include "Drive.h"
#include "Arm.h"
#include "RobotIO.h"
#include "Constants.h"

class Robot {
public:
	Robot();
	void periodic(const RobotIn& rIn, RobotOut& rOut);

private:
	Drive drive;
	Arm arm;
};
