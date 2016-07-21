#include "Field.h"

INPUT input;  // Input Buffer
OUTPUT output; // Output Buffer


FIELD :: FIELD() //Constructificate
	{
	//init variables here
    runs = 0;
    autoruns = 0;
	teleruns = 0;
	testruns = 0;
	disabledruns = 0;
	enabledruns = 0;
	FieldTimer.Start();
	looptime = 0;
	lastloop = 0;
	looprate = 0;
	}

void FIELD :: StartCompetition()
	{
	//******************************************************************************
	//NOTE: DO NOT ATTEMPT to use the SmartDashboard until AFTER COMMS has initialized
	//as part of the Robot Class Init().  COMMS is owned by Robot, but the SmartDashboard
	//is a global resource that is not Class Restricted.
	//******************************************************************************

	//Init the robot
	Robot.Init();  //  After this it is OK to use SmartDashBoard.
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
					Robot.AutoInit(&input);
					}
				//auto
				Robot.Autonomous(&input);
				autoruns ++;
				//input.autoruns = autoruns;
				SmartDashboard::PutNumber("AutonomousRuns", autoruns);
				}

			else if (IsOperatorControl())
				{
				if (teleruns == 0)
					{
					//tele init
					Robot.TeleInit(&input);
					}
				//tele
				Robot.Teleop(&input);
				teleruns ++;
				SmartDashboard::PutNumber("TeleopRuns", teleruns);
				}

			else if (IsTest())
				{
				if (testruns == 0)
					{
					//test init
					Robot.TestInit(&input);
					}
				//test
				Robot.Test(&input);
				testruns ++;
				SmartDashboard::PutNumber("TestRuns", testruns);
				}

			else
				{
				//enabled, but not in any mode
				//this should never happen
				//actually, this is Practice Mode (which we don't normally use)
				}

			enabledruns ++;
			SmartDashboard::PutNumber("EnabledRuns", enabledruns);

			} //end enabled

		else if (IsDisabled())
			{
			if (disabledruns == 0)
				{
				//disabled init
				Robot.DisabledInit(&input);
				}
			//disabled
			Robot.Disabled(&input);
			disabledruns ++;
			SmartDashboard::PutNumber("DisabledRuns", disabledruns);
			}

		else
			{
			//robot isn't enabled OR disabled
			//somebody at FIRST done broked it
			}

		SmartDashboard::PutNumber("TotalLoops", runs);

		//LOOPPERIOD should be no more than 20 msec (if drivetrain is based on Jaguars).
		//Set to no more than 10 msec if you use Victors in the drivetrain.
		//At this point, we use to just sit in a Wait state for LOOPPERIOD.  But no more.
		//Sitting in a wait state is a waste of valuable time.  Time that could be used
		//to update your position in the inertial navigation subsystem.
		while ((looptime = FieldTimer.Get()) - lastloop < LOOPPERIOD)
			{
			//Do "stuff" here that does not require a lot of time and does not require
			//input from the driver station and does not attempt to modify motor
			//or other actuator controls.
			//This could be capturing/processing video data
			//or computing a targeting solution
			//or making interesting light patterns
			//use your imagination.
			//Just do something besides waiting.
			Wait(0.001);
			}
		lastloop = looptime;
		runs ++;
		} //end while
	} //end StartCompetition

START_ROBOT_CLASS(FIELD); //entry point
