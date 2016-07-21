/*
 * Robot.h
 *
 *  Created on: Jan 25, 2016
 *      Author: Rod
 */

#ifndef SRC_ROBOT_H_
#define SRC_ROBOT_H_

#include "Definitions.h"
#include "Comms.h"
#include "DriveTrain.h"
#include "ScooperShooter.h"
#include "Climber.h"
#include "Navigator.h"
#include "Sensors.h"
#include "AutoPilot.h"

class ROBOT: public IterativeRobot
	{

public:
	ROBOT();

private:

	virtual void RobotInit();
	virtual void AutonomousInit();
	virtual void AutonomousPeriodic();
	virtual void TeleopInit();
	virtual void TeleopPeriodic();
	virtual void TestInit();
	virtual void TestPeriodic();
	virtual void DisabledInit();
	virtual void DisabledPeriodic();
	void CheckSensors();

	COMMS 				Comms;				// Used to obtain inputs from the driver station
	DRIVETRAIN 			Drivetrain;			// Da wheels
	SCOOPER				SuperDuperScooperShooter;
	CLIMBER				Climber;
	NAVIGATOR 			Navigator;			// This is the Robot's Inertial Navigation subsystem
	AUTOPILOT 			AutoPilot;			// This is the set of autonomous control methods
	SENSORS				Sensors;

//	LiveWindow *lw;

};


#endif /* SRC_ROBOT_H_ */
