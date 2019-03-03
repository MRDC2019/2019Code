#pragma once
#include <cstdint>

struct RobotIn{
	uint16_t waist;
	uint16_t shoulder;
	uint16_t elbow;

	RobotIn() :
		waist(0),
		shoulder(0),
		elbow(0)
	{}
};

struct RobotOut{
	uint8_t driveFL;
	uint8_t driveBL;
	uint8_t driveFR;
	uint8_t driveBR;

	uint8_t waist;
	uint8_t shoulder;
	uint8_t elbow;
	uint8_t wrist;
	bool vacuum;

	RobotOut() :
		driveFL(90),
		driveBL(90),
		driveFR(90),
		driveBR(90),
		waist(90),
		shoulder(90),
		elbow(90),
		wrist(90),
		vacuum(false)
	{}
};

