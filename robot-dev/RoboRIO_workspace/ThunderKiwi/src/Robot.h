#ifndef Robot_H
#define Robot_H
//mid-level actions

#include "Definitions.h"
#include "Drivetrain.h"
#include <math.h>
#include <Navigator.h>
#include "WPILib.h"
#include "Comms.h"
#include "Timer.h"

class ROBOT
{


public:
	ROBOT();	// ROBOT class is public (so addressable by Field
			// Public Functions/Procedures (actions).  These are all called by Field
			// in response to FMS signals.
	void Init(RobotStateBuffer *pMyRobotState);
	void AutoInit(INPUT* input);
	void Autonomous(INPUT* input);
	void TeleInit(INPUT* input);
	void Teleop(INPUT* input);
	void TestInit(INPUT* input);
	void Test(INPUT* input);
	void DisabledInit(INPUT* input);
	void Disabled(INPUT* input);

private:
			// Privat Subsystems and/or Components
			// Note:	These subsystem are internal to the Robot class and are
			//			referenced only by the Public and Private Functions/Procedures.
	DRIVETRAIN Drivetrain;			// Da wheels
	COMMS Comms;					// Used to obtain inputs from the driver station
	NAVIGATOR Navigator;			// This is the Robot's Inertial Navigation subsystem

	void SendData();				// Not currently used

	int robotinitcnt;
	bool hasInited;


};

#endif
