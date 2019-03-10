#include "Robot.h"

Robot::Robot() :
	drive(),
	arm()
{
	
}

void Robot::periodic(const RobotIn& rIn, RobotOut& rOut){
	drive.periodic(rIn, rOut);
	arm.periodic(rIn, rOut);
}
