#include "Field.h"


FIELD :: FIELD() //Constructificate
	{
	//init variables here
    runs = 0;
    autoruns = 0;
	teleruns = 0;
	testruns = 0;
	disabledruns = 0;
	enabledruns = 0;
	//GetWatchdog().Feed();
	FieldTimer.Start();
	looptime = 0;
	lastloop = 0;
	looprate = 0;
	pMyRobotState = new RobotStateBuffer;
	pMyInput = new INPUT;
	}

void FIELD :: StartCompetition()
	{
	//Init the robot
	pMyRobotState->Robot_Heading = 0;
	Robot.Init(pMyRobotState, pMyInput);
	FieldTimer.Reset();
	while (1)
		{
		//GetWatchdog().Feed(); //encourage our canine overlord not to tear us limb from limb

		if (IsEnabled())
			{
			if (enabledruns == 0)
				{
				//enabled initialization
				}

			if (IsAutonomous())
				{
				if (autoruns == 0)
					{
					//auto init
					Robot.AutoInit();
					}
				//auto
				Robot.Autonomous();
				//input.autoruns = autoruns;
				SmartDashboard::PutNumber("AutonomousRuns", ++autoruns);
				}

			else if (IsOperatorControl())
				{
				if (teleruns == 0)
					{
					//tele init
					Robot.TeleInit();
					}
				//tele
				Robot.Teleop();
				SmartDashboard::PutNumber("TeleopRuns", ++teleruns);
				}

			else if (IsTest())
				{
				if (testruns == 0)
					{
					//test init
					Robot.TestInit();
					}
				//test
				Robot.Test();
				SmartDashboard::PutNumber("TestRuns", ++testruns);
				}

			else
				{
				//enabled, but not in any mode
				//this should never happen
				//actually, this is Practice Mode (which we don't normally use)
				}
			SmartDashboard::PutNumber("EnabledRuns", ++enabledruns);
			} //end enabled

		else if (IsDisabled())
			{
			if (disabledruns == 0)
				{
				//disabled init
				Robot.DisabledInit();
				}
			//disabled
			Robot.Disabled();
			SmartDashboard::PutNumber("DisabledRuns", ++disabledruns);
			}

		else
			{
			//robot isn't enabled OR disabled
			//somebody at FIRST done broked it
			}
//		Robot.Comms.Send();
		SmartDashboard::PutNumber("TotalLoops", runs);
		SmartDashboard::PutNumber("GyroHeading", pMyRobotState->Robot_Heading);
		SmartDashboard::PutNumber("Xcoord", pMyRobotState->Robot_Position_X);
		SmartDashboard::PutNumber("Ycoord", pMyRobotState->Robot_Position_Y);
		SmartDashboard::PutNumber("Direction", pMyRobotState->Robot_Direction);
		SmartDashboard::PutNumber("Speed", pMyRobotState->Robot_Speed);

		Robot.CheckSensors();
		//LOOPPERIOD should be no more than 20 msec (if drivetrain is based on Jaguars).
		//Set to no more than 10 msec if you use Victors in the drivetrain.
		//At this point, we use to just sit in a Wait state for LOOPPERIOD.  But no more.
		//Sitting in a wait state is a waste of valuable time.  Time that could be used
		//to update your position in the inertial navigation subsystem.
//		while ((! IsAutonomous()) && ((looptime = FieldTimer.Get()) - lastloop < LOOPPERIOD))
		while (((looptime = FieldTimer.Get()) - lastloop < LOOPPERIOD) && (! IsAutonomous()))
			{
			//Do "stuff" here that does not require a lot of time and does not require
			//input from the driver station and does not attempt to modify motor
			//or other actuator controls.
			//This could be capturing/processing video data
			//or computing a targeting solution
			//or making interesting light patterns
			//use your imagination.
			//Just do something besides waiting.
//			Robot.Navigator.OversampleGyro();
			Wait(0.001);
			}
		SmartDashboard::PutNumber("MainLoopTime", (looptime - lastloop) * 1000);	// display looptime in msec
		lastloop = looptime;
		runs ++;
		} //end while
	} //end StartCompetition

START_ROBOT_CLASS(FIELD); //entry point
