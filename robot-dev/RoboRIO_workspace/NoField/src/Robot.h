#ifndef Robot_H
#define Robot_H
//mid-level actions

#include "Definitions.h"
#include "Drivetrain.h"
#include "Claw.h"
//#include "O2CCATAPULT.h"
#include "Quadapult.h"
//#include "Navigator.gyro.h"
#include <math.h>
#include <Navigator.Dev.h>
#include "WPILib.h"
#include "Comms.h"
#include "Timer.h"

class Robot: public IterativeRobot
{
private:
	LiveWindow *lw;
	void RobotInit();
	int robotinitcnt;
	void AutonomousInit();
	int robotautocnt, robotautoinc;
	void AutonomousPeriodic();
	void TeleopInit();
	int robottelecnt, robotteleinc;
	void TeleopPeriodic();
	void TestInit();
	int robottestcnt, robottestinc;
	void TestPeriodic();
	void DisabledInit();
	int robotdiscnt, robotdisinc;
	void DisabledPeriodic();

//	void Init();
//	void AutoInit(INPUT* input);
//	void Autonomous(INPUT* input);
//	void TeleInit();
//	void Teleop(INPUT* input);
//	void Test(INPUT* input);
//	void Disabled(INPUT* input);

	DRIVETRAIN Drivetrain;			// Da wheels
	QUADAPULT Catapult;				// Da shooter
	DigitalInput PhotoSensor;		// Used to detect the hot goal for one ball autonomous mode
	AnalogInput UltraSonic;			// Used only to detect a second ball for two ball autonomous mode
	O2CClaw Forklift;				// Used to pick up, carry, load, or herd the ball
	COMMS Comms;					// Used to obtain inputs from the driver station
	Timer AutoTimer;				// Used for loop timing
	NAVIGATOR Navigator;			// This is the Robot's Inertial Navigation subsystem

	void TestClaw();				// Test the operation of the Forklift
	void TestDrive(INPUT* input);	// Test the operation of the Drivetrain
	void TestFire(INPUT* input);	// Test the operation of the catapult
	void CheckPhotoSensor();		// Update the SmartDashboard with the value of the photosensor
	void UltraSonicSensor();		// Update the SmartDashboard with the value of the Ultrasonic sensor
	void ForkliftEncoder();			// Update the SmartDashboard with the value of the Forklift encoder
	void SendData();				// Not currently used

	bool hasInited;
	int AutoState;
	int TwoBallMode;
	double TempTimerVal;
	double forktarget;
	double forkposition;
	double prevposition;
	double forkpower;
	double ultrasonicrange;
	bool hotgoal;

};

#endif
