#ifndef MecanumDrivetrain_H
#define MecanumDrivetrain_H

#include "Definitions.h"
#include "WPILib.h"
#include "math.h"

class MECANUMDRIVE
{
public:
	MECANUMDRIVE();

	void Init(RobotStateBuffer *pRobotState, INPUT *pInput);
	void Drive();
	void StopMotors();

private:
	Talon *pMotorFL;
	Talon *pMotorBL;
	Talon *pMotorFR;
	Talon *pMotorBR;
	RobotDrive *pRobotDrive;

	void SprintCalc();
	void OrientCalc();
	void SpinControl();

	int driveinitcnt;
	DriveOrientation OrientMode;
	float OrientAngle;
	SpeedControl	SprintMode;
	double	sprint_factor;
	bool	spincontrol;
	double driftangle, prev_driftangle;
	double rotation, prev_rotation;
	int spin_enable_count;

};

#endif

