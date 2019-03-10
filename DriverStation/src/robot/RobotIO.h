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
	uint8_t vacuum;

	RobotOut() :
		driveFL(127),
		driveBL(127),
		driveFR(127),
		driveBR(127),
		waist(127),
		shoulder(127),
		elbow(127),
		wrist(127),
		vacuum(127)
	{}
};

