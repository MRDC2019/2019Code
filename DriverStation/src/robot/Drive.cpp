# include "Drive.h"

Drive::Drive(){
}

void Drive::periodic(const RobotIn& rIn, RobotOut& rOut){
	double driveSpeed = 0.3;
	if(CTRL_FULL_PWR){
		driveSpeed = 1.0;
	}

	rOut.driveFL = Rev(SOut(CTRL_TANK_LEFT, driveSpeed));
	rOut.driveBL = Rev(SOut(CTRL_TANK_LEFT, driveSpeed));
	rOut.driveFR = Rev(SOut(CTRL_TANK_RIGHT, driveSpeed));
	rOut.driveBR = Rev(SOut(CTRL_TANK_RIGHT, driveSpeed));
	printf("out=(% .2f,% .2f) spd=%.2f ", rOut.driveFL/255.0*2.0-1.0, rOut.driveFR/255.0*2.0-1.0, driveSpeed);

	if(JOY_DISCONNECTED(JOY0)){
		rOut.driveFL = 127;
		rOut.driveBL = 127;
		rOut.driveFR = 127;
		rOut.driveBR = 127;
	}
}
 