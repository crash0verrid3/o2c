#include <Main.h>


MAIN :: MAIN() //Constructificate
	{
	//init variables here
//	pMyFieldState = new FieldStateBuffer;
	pMyInput = 0;
	pMyRobotState = 0;
	FieldTimer.Start();
	looptime = 0;
	lastloop = 0;
	}

void MAIN :: StartCompetition()
	{
	//Init the robot
//	memset(pMyFieldState, 0, sizeof(*pMyFieldState));		// same as setting all elements to 0 as follows
//	pMyFieldState->looprate = 0;
//	pMyFieldState->totalruns = 0;
//	pMyFieldState->autoruns = 0;
//	pMyFieldState->teleruns = 0;
//	pMyFieldState->testruns = 0;
//	pMyFieldState->disabledruns = 0;
//	pMyFieldState->enabledruns = 0;
//	pMyRobotState->Robot_Heading = 0;
//	Robot.Init(pMyFieldState);
	FieldTimer.Reset();
	while (1)
		{
		if (IsEnabled())
			{
//			if (pMyFieldState->enabledruns++ == 0)	// Robot.EnabledInit();
				{
//				Robot.Enable();
				}

			if (IsAutonomous())
				{
//				if (pMyFieldState->autoruns++ == 0) Robot.AutoInit();
//				Robot.Autonomous();
				}

			else if (IsOperatorControl())
				{
//				if (pMyFieldState->teleruns++ == 0) Robot.TeleInit();
//				Robot.Teleop();
				}

			else if (IsTest())
				{
//				if (pMyFieldState->testruns++ == 0) Robot.TestInit();
//				Robot.Test();
				}

//			else	//this is Practice Mode (which we don't normally use)
//				{
//				}
			} //end enabled

		else if (IsDisabled())
			{
//			if (pMyFieldState->disabledruns++ == 0) Robot.DisabledInit();
//			Robot.Disabled();
			}

		else
			{
			//robot isn't enabled OR disabled
			//somebody at FIRST done broked it
			}

		pMyFieldState->totalruns++;

//		Robot.CheckSensors();
//		Robot.Comms.Send();

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

		pMyFieldState->looprate = int(1 / fmax(0.001, looptime - lastloop));  // avoid divide by zero crash
		lastloop = looptime;
		} //end while
	} //end StartCompetition

START_ROBOT_CLASS(MAIN); //entry point
