#include <ROBOT.h>

ROBOT :: ROBOT() : myRobot(0, 1), stick(0)
	{
	pMyTargetState = 0;
	myRobot.SetExpiration(0.1);
	}

void ROBOT :: RobotInit()
	{
	pMyFieldState = new FieldStateBuffer;
	pMyTargetState = new RobotStateBuffer;
	pMyRobotState = new RobotStateBuffer;
	pMyInput = new INPUT;
	memset(pMyFieldState, 0, sizeof(*pMyFieldState));		// set all elements to 0
	memset(pMyTargetState, 0, sizeof(*pMyTargetState));
	memset(pMyRobotState, 0, sizeof(*pMyRobotState));
	memset(pMyInput, 0, sizeof(*pMyInput));
	pMyRobotState->Robot_Init = 1;

	//	Robot.Init(pMyFieldState);
	Comms.Init(pMyRobotState, pMyInput, pMyTargetState, pMyFieldState);
	Navigator.Init(pMyRobotState, pMyInput);
	pMyRobotState->Robot_Init = 2;
	}

void ROBOT :: AutonomousInit()
	{
	Navigator.Zero();
	}

void ROBOT :: TestInit()
	{
	Navigator.Zero();
	}

void ROBOT :: DisabledInit()
	{
	myRobot.Drive(0.0, 0.0); 	// stop robot
	}

void ROBOT :: TeleopInit()
	{
	Navigator.Zero();
	}

void ROBOT :: AutonomousPeriodic()
	{
	pMyFieldState->totalruns++;
	pMyFieldState->enabledruns++;
	pMyFieldState->autoruns++;
	myRobot.Drive(-0.5, 0.0); 	// drive forwards half speed
	Comms.Send();
	}

void ROBOT :: TeleopPeriodic()
	{
	Comms.Get();				// get pInput data
	pMyFieldState->totalruns++;
	pMyFieldState->enabledruns++;
	pMyFieldState->teleruns++;
	myRobot.ArcadeDrive(stick); // drive with arcade style (use right stick)
	Comms.Send();
	}

void ROBOT :: TestPeriodic()
	{
	Comms.Get();				// get pInput data
	pMyFieldState->totalruns++;
	pMyFieldState->enabledruns++;
	pMyFieldState->testruns++;
	Comms.Send();
	}

void ROBOT :: DisabledPeriodic()
	{
	Comms.Get();				// get pInput data
	Navigator.SetStartingPosition();
	Navigator.ReadIMUCalStat();
	pMyFieldState->totalruns++;
	pMyFieldState->disabledruns++;
	myRobot.ArcadeDrive(stick); // drive with arcade style (use right stick)
	Comms.Send();
	}


START_ROBOT_CLASS(ROBOT)
