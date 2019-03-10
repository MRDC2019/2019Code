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
	static char SOut(double out, double scale=1.0) { return (char)((out*scale + 1.0)/2.0*255.0); }
	// reverses from [255,0] to [0,255] for the arduino
	static char Rev(char out) { return 255 - out; }
};
