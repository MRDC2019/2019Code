#include "Drive.h"

Drive::Drive(){
}

void Drive::periodic(const RobotIn& rIn, RobotOut& rOut){
	rOut.driveFL = SOut(CTRL_TANK_LEFT);
	rOut.driveBL = SOut(CTRL_TANK_LEFT);
	rOut.driveFR = Rev(SOut(CTRL_TANK_RIGHT));
	rOut.driveBR = Rev(SOut(CTRL_TANK_RIGHT));
}
