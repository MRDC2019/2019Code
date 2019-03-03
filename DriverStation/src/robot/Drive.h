#pragma once

#include "RobotIO.h"
#include "Controls.h"
#include "Constants.h"
#include <iostream>
#include <cmath>
#include <ctime>

class Drive {
public:
	Drive();
	void periodic(const RobotIn& rIn, RobotOut& rOut);

	// converts from [-1.0, 1.0] to [0,180] for the arduino
	static char SOut(float out) { return (char)((out + 1.0f)*90.0f); }
	// reverses from [180,0] to [0,180] for the arduino
	static char Rev(char out) { return 180 - out; }
};
