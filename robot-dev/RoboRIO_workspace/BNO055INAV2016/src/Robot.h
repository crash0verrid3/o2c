#ifndef Robot_H
#define Robot_H

#include <Definitions.h>
#include <Navigator.h>
#include "Comms.h"

class ROBOT
{

public:
	ROBOT();// ROBOT class is public (so addressable by Field
			// Public Functions/Procedures (actions).  These are all called by Field
			// in response to FMS signals.

	void Init(FieldStateBuffer *pFieldState);
	void Enable();
	void AutoInit();
	void Autonomous();
	void TeleInit();
	void Teleop();
	void TestInit();
	void Test();
	void DisabledInit();
	void Disabled();
	void CheckSensors();
	COMMS Comms;					// Used to obtain inputs from the driver station


private:
			// Privat Subsystems and/or Components
			// Note:	These subsystem are internal to the Robot class and are
			//			referenced only by the Public and Private Functions/Procedures.
	NAVIGATOR Navigator;			// This is the Robot's Inertial Navigation subsystem

	RobotStateBuffer	*pMyTargetState;
//	INPUT				*pMyInput;


};

#endif
