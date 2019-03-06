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

	// converts from [-1.0, 1.0] to [0,255] for the arduino
	static char SOut(float out) { return (char)((out + 1.0f)/2.0f*255.0f); }
	// reverses from [255,0] to [0,255] for the arduino
	static char Rev(char out) { return 255 - out; }
};
