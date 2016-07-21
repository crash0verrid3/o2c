/*
 * Robot.h
 *
 *  Created on: Jan 25, 2016
 *      Author: Rod
 */

#ifndef SRC_ROBOT_H_
#define SRC_ROBOT_H_

#include "Definitions.h"
#include "Navigator.h"
#include "Comms.h"

class ROBOT: public IterativeRobot
	{

public:
	ROBOT();
	RobotDrive myRobot; // robot drive system
	Joystick stick; // only joystick

private:

	void RobotInit();

	void AutonomousInit();

	void AutonomousPeriodic();

	void TeleopInit();

	void TeleopPeriodic();

	void TestInit();

	void TestPeriodic();

	void DisabledInit();

	void DisabledPeriodic();

	void CheckSensors();

	COMMS 				Comms;				// Used to obtain inputs from the driver station
	NAVIGATOR 			Navigator;			// This is the Robot's Inertial Navigation subsystem
	RobotStateBuffer	*pMyTargetState;	// Set the Robot's desired state

};




#endif /* SRC_ROBOT_H_ */
