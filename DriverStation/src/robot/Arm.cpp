#include "Arm.h"

Arm::Arm():
	waistPID(PID::distance, 1.75, 0.0, 0.3),
	shoulderPID(PID::distance, 0.2, 0.0, 0.0),
	elbowPID(PID::distance, 0.2, 0.0, 0.0)
{
}

//shoulder up (pos)=decrease
//elbow up(pos)=increase
void Arm::periodic(const RobotIn& rIn, RobotOut& rOut){
	double waistCurrent = rIn.waist / 16384.0 * 2.0 * M_PI;
	double shoulderCurrent = rIn.shoulder / 16384.0 * 2.0 * M_PI;
	double elbowCurrent = PMod(rIn.elbow / 16384.0 * 2.0 * M_PI + shoulderCurrent, 2.0 * M_PI);
	//printf("waist = %5f    shoulder = %5f    elbow = %5f\n", waistCurrent, shoulderCurrent, elbowCurrent);

	if(CTRL_MANUAL && !lastCtrlManual) {
		manualCtrl = !manualCtrl;
		printf("Manual Ctrl = %d\n", manualCtrl);
		initAngles = false;
	}
	lastCtrlManual = CTRL_MANUAL;

	if(manualCtrl){
		rOut.waist = Rev(SOut(CTRL_ARM_Y, 0.3));
		rOut.shoulder = Rev(SOut(CTRL_ARM_X, 0.4));
		rOut.elbow = Rev(SOut(CTRL_ARM_Z));
	}
	else{
		double armSpeed = 1.0;

		clock_t time = clock();
		double dt = double(time - lastTime) / CLOCKS_PER_SEC;
		lastTime = time;
		if(dt < 0.1){
			xTarget += CTRL_ARM_X * armSpeed * dt;
			yTarget += CTRL_ARM_Y * armSpeed * dt;
			zTarget += CTRL_ARM_Z * armSpeed * dt;
		}

		if(!initAngles){
			initAngles = true;

			waistPID.reset();
			shoulderPID.reset();
			elbowPID.reset();

			waistTarget = waistCurrent;
			shoulderTarget = shoulderCurrent;
			elbowTarget = elbowCurrent;
			forwardKinematics();

			if(xTarget < 0.1 && (waistCurrent < M_PI/2.0 || waistCurrent > M_PI*3.0/2.0))
				xTarget = 0.1;
		}

		inverseKinematics();

		waistPID.setTarget(PMod(waistTarget, 2.0 * M_PI));
		shoulderPID.setTarget(PMod(shoulderTarget, 2.0 * M_PI));
		elbowPID.setTarget(PMod(elbowTarget, 2.0 * M_PI));

		double waistOut = waistPID.compute(waistCurrent);
		double shoulderOut = -shoulderPID.compute(shoulderCurrent);
		double elbowOut = elbowPID.compute(elbowCurrent);

		//if(waistOut > 0.1 && waistOut < 0.45)
		//	waistOut = 0.45;
		//if(waistOut < -0.1 && waistOut > -0.36)
		//	waistOut = -0.36;

		rOut.waist = SOut(waistOut);
		rOut.shoulder = SOut(shoulderOut);
		rOut.elbow = SOut(elbowOut);
		//printf("out=(%+.2f,%+.2f,%+.2f) posTar=(%.2f,%.2f,%.2f) tar=(%.2f,%.2f,%.2f) cur=(%.3f,%.3f,%.3f) %d\n", 
		//	   rOut.waist/255.0*2.0-1.0, rOut.shoulder/255.0*2.0-1.0, rOut.elbow/255.0*2.0-1.0,
		//	   xTarget, yTarget, zTarget,
		//	   PMod(waistTarget,2*M_PI), PMod(shoulderTarget, 2*M_PI), PMod(elbowTarget, 2*M_PI),
		//	   waistCurrent, shoulderCurrent, elbowCurrent, rOut.vacuum);

		rOut.waist = 127;
		rOut.shoulder = 127;
		rOut.elbow = 127;
		//printf("elbow = %3d\n", rOut.elbow);
	}

	if(CTRL_VACUUM2_FWD){
		rOut.wrist = 00;
	} else if(CTRL_VACUUM2_BWD){
		rOut.wrist = 255;
	} else{
		rOut.wrist = 127;
	}

	if(CTRL_VACUUM && !lastCtrlVacuum) {
		if(rOut.vacuum == 127)
			rOut.vacuum = 38;
		else
			rOut.vacuum = 127;
	}
	lastCtrlVacuum = CTRL_VACUUM;


	printf("arm=(% .2f,% .2f,% .2f) enc=(%.3f,%.3f,%.3f) vac=(% .2f,% .2f)\n", rOut.waist/255.0*2.0-1.0, rOut.shoulder/255.0*2.0-1.0, rOut.elbow/255.0*2.0-1.0,
		   waistCurrent, shoulderCurrent, elbowCurrent, rOut.vacuum/255.0*2.0-1.0, rOut.wrist/255.0*2.0-1.0);

	if(JOY_DISCONNECTED(JOY1)){
		rOut.waist = 127;
		rOut.wrist = 127;
		rOut.shoulder = 127;
		rOut.elbow = 127;
		rOut.vacuum = 127;
	}
}

void Arm::forwardKinematics(){
	double extension = upperArmLen*cos(shoulderTarget)+foreArmLen*cos(elbowTarget-shoulderTarget);
	xTarget = cos(waistTarget)*extension;
	yTarget = sin(waistTarget)*extension;
	zTarget = upperArmLen*sin(shoulderTarget)-foreArmLen*sin(elbowTarget-shoulderTarget);
	printf("x = %3f   y = %3f   z = %3f\n", xTarget, yTarget, zTarget);
}

void Arm::inverseKinematics(){
	waistTarget = atan2(yTarget, xTarget);
	double D = (xTarget*xTarget + yTarget*yTarget + (zTarget)*(zTarget)-upperArmLen*upperArmLen - foreArmLen*foreArmLen)/(2*upperArmLen*foreArmLen);
	if(D > 1.0)
		D = 1.0;
	if(D < -1.0)
		D = -1.0;
	elbowTarget = atan2(sqrt(1-D*D), D);
	shoulderTarget = atan2(zTarget, sqrt(xTarget*xTarget + yTarget*yTarget)) + atan2(foreArmLen*sin(elbowTarget), upperArmLen + foreArmLen*cos(elbowTarget));
}
