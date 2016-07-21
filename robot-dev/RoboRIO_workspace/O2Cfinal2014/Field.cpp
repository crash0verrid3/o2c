#include "Field.h"

INPUT input;
OUTPUT output;


FIELD :: FIELD() //Constructificate
{
	//init variables here
    runs = 0;
    autoruns = 0;
	teleruns = 0;
	testruns = 0;
	disabledruns = 0;
	enabledruns = 0;
	GetWatchdog().Feed();
	LoopTimer.Reset();
	LoopTimer.Start();
}
	
void FIELD :: StartCompetition()
{
	while (1)
	{
		GetWatchdog().Feed(); //encourage our canine overlord not to tear us limb from limb
		
        if (runs == 0)
		{
			//Init the robot
			Master.Init();
		}
		
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
					Master.AutoInit(&input);
				}
				//auto
				Master.Autonomous(&input);
				autoruns ++;
				//input.autoruns = autoruns;
				SmartDashboard::PutNumber("AutonomousRuns", autoruns);
			}
			
			else if (IsOperatorControl())
			{
				if (teleruns == 0)
				{
					//tele init
					Master.TeleInit();
				}
				//tele
				Master.Teleop(&input);
				teleruns ++;
				SmartDashboard::PutNumber("TeleopRuns", teleruns);
			}
			
			else if (IsTest())
			{
				if (testruns == 0)
				{
					//test init
					Master.TestInit();
				}
				//test
				Master.Test(&input);
				testruns ++;
				SmartDashboard::PutNumber("TestRuns", testruns);
			}
			
			else
			{
				//enabled, but not in any mode
				//this should never happen
			}
			
			enabledruns ++;
			SmartDashboard::PutNumber("EnabledRuns", enabledruns);
			
		} //end enabled
		
		else if (IsDisabled())
		{
			if (disabledruns == 0)
			{
				//disabled init
				Master.DisabledInit();
			}
			//disabled
			Master.Disabled();
			disabledruns ++;
			SmartDashboard::PutNumber("DisabledRuns", disabledruns);
		}
		
		else
		{
			//robot isn't enabled OR disabled
			//somebody at FIRST done broked it
		}
		
		SmartDashboard::PutNumber("TotalLoops", runs);
		
		
		/*
		//For loop timing
		looprate = SmartDashboard::GetNumber("LoopsPerSecond");
		if (LoopTimer.Get() - lastloop <= 1 / looprate)
		{
			SmartDashboard::PutBoolean("LoopRateStable?", true);
			while (LoopTimer.Get() - lastloop <= 1 / looprate)
			{
				Wait (0.01);
			}
		}
		else
		{
			SmartDashboard::PutBoolean("LoopRateStable?", false);
		}
		lastloop = LoopTimer.Get();
		*/
        runs ++;
		Wait (0.02);
	} //end while
} //end StartCompetition

START_ROBOT_CLASS(FIELD); //entry point

