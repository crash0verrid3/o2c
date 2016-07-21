#include "Robot.h"

InterativeRobot :: Robot()
{

}

	void RobotInit()
	{
		lw = LiveWindow::GetInstance();
	}

	void AutonomousInit()
	{

	}

	void AutonomousPeriodic()
	{

	}

	void TeleopInit()
	{

	}

	void TeleopPeriodic()
	{

	}

	void TestInit()
	{

	}


	void TestPeriodic()
	{
		lw->Run();
	}

	void DisabledInit()
	{

	}

	void DisabledPeriodic()
	{

	}


};

START_ROBOT_CLASS(Robot);
