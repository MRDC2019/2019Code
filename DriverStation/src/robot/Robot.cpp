#include "Robot.h"

Robot::Robot() :
	drive()
{
	
}

void Robot::periodic(const RobotIn& rIn, RobotOut& rOut){
	drive.periodic(rIn, rOut);
}
