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
			if (pMyRobotState->Field_Enabledruns++ == 0)	// Robot.EnabledInit();
				{
				Robot.Enable();
				}

			if (IsAutonomous())
				{
				if (pMyRobotState->Field_Autoruns++ == 0) Robot.AutoInit();
				Robot.Autonomous();
				}

			else if (IsOperatorControl())
				{
				if (pMyRobotState->Field_Teleruns++ == 0) Robot.TeleInit();
				Robot.Teleop();
				}

			else if (IsTest())
				{
				if (pMyRobotState->Field_Testruns++ == 0) Robot.TestInit();
				Robot.Test();
				}

//			else	//this is Practice Mode (which we don't normally use)
//				{
//				}
			} //end enabled

		else if (IsDisabled())
			{
			if (pMyRobotState->Field_Disabledruns++ == 0) Robot.DisabledInit();
			Robot.Disabled();
			}

		else
			{
			//robot isn't enabled OR disabled
			//somebody at FIRST done broked it
			}

		pMyRobotState->Field_Totalruns++;

		Robot.CheckSensors();
		Robot.Comms.Send();

		//LOOPPERIOD should be no more than 20 msec (if drivetrain is based on Jaguars).
		//Set to no more than 10 msec if you use Victors in the drivetrain.
		//At this point, we use to just sit in a Wait state for LOOPPERIOD.  But no more.
		//Sitting in a wait state is a waste of valuable time.  Time that could be used
		//to update your position in the inertial navigation subsystem.
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

		pMyRobotState->Field_LoopTime = (looptime - lastloop) * 1000;

		lastloop = looptime;
		} //end while
	} //end StartCompetition

START_ROBOT_CLASS(FIELD); //entry point
