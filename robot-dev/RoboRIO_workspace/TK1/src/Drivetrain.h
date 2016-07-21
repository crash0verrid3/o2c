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
	void MecanumDrive(INPUT* input);
	void StopAll();

private:
	int OrientMode;
	int NextOrientMode;
	int SprintMode;
	int NextSprintMode;
	float OrientAngle;
	Victor *pMotorFL;
	Victor *pMotorBL;
	Victor *pMotorFR;
	Victor *pMotorBR;
	RobotDrive *pRobotDrive;

	void SprintCalc(INPUT* input);
	void OrientCalc(INPUT* input);


	int driveinitcnt;

};

#endif
