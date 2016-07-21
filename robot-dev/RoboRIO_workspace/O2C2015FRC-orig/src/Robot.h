#ifndef Robot_H
#define Robot_H
//mid-level actions

#include <Definitions.h>
#include <math.h>
//#include <Pincer.h>
#include <ToteLift.h>
#include <Autonomous.h>
#include <MecanumDrivetrain.h>
#include <Navigator.h>
#include <ToteWheels.h>
#include "WPILib.h"
#include "Comms.h"
#include "Timer.h"
#include "PowerDistributionPanel.h"

class ROBOT
{

public:
	ROBOT();	// ROBOT class is public (so addressable by Field
			// Public Functions/Procedures (actions).  These are all called by Field
			// in response to FMS signals.

	void Init(RobotStateBuffer *pRobotState, INPUT *pInput);
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
	MECANUMDRIVE Drivetrain;			// Da wheels
	NAVIGATOR Navigator;			// This is the Robot's Inertial Navigation subsystem
	AUTONOMOUS AutoClass;			// This is the set of autonomous control methods
//	DigitalInput PhotoSensor;		// Used to detect the hot goal for one ball autonomous mode
//	AnalogInput UltraSonic;			// Used only to detect a second ball for two ball autonomous mode
	TOTELIFT ToteLift;				// Used to pick up, carry, herd or stack totes
	TOTEWHEELS ToteWheels;			// Used to ingest totes into the robot for pickup by totelift
//	PINCER Pincer;					// Used to pick up, carry, or stack bins and totes
//	Timer AutoTimer;				// Used for loop timing
	PowerDistributionPanel *pPDP;	// Get true Battery Voltage and other power readings via CAN bus


//	void CheckPhotoSensor();		// Update the SmartDashboard with the value of the photosensor
//	void UltraSonicSensor();		// Update the SmartDashboard with the value of the Ultrasonic sensor
//	void ForkliftEncoder();			// Update the SmartDashboard with the value of the Forklift encoder

	int robotinitcnt;

};

#endif
