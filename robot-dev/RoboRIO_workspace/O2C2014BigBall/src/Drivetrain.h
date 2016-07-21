#ifndef Drivetrain_H
#define Drivetrain_H

#include "Definitions.h"
#include "WPILib.h"
#include "math.h"

class DRIVETRAIN
{
public:
	DRIVETRAIN();
	void Init();
	void KiwiDrive(INPUT* input);
	//void AtonKiwiDrive(INPUT* input, double atonX, double atonY, double atonR);
	void Break();

private:
	int OrientMode;
	Victor MotorL;
	Victor MotorR;
	Victor MotorB;
	double relativeL;
	double relativeR;
	double relativeB;
	double mag;
	double angle;
	double rot;
	double drvpow;
	int ActiveState;
	double ActiveHeading;
	double ActiveRot(INPUT* input);
	bool FromRobotMode;
	void Mode(INPUT* input);
	int driveinitcnt;

};

#endif
