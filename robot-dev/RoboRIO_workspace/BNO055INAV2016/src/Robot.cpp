#include <Robot.h>

ROBOT :: ROBOT()	// : PhotoSensor(PHOTOCHANNEL), UltraSonic(ULTRASONICCHANNEL)
	{
	//init variables
	pMyTargetState = new RobotStateBuffer;
	pMyRobotState = new RobotStateBuffer;
	pMyInput = new INPUT;
	}


void ROBOT :: Init(FieldStateBuffer *pFieldState)
	{
	pMyFieldState = pFieldState;
	memset(pMyTargetState, 0, sizeof(*pMyTargetState));
	memset(pMyRobotState, 0, sizeof(*pMyRobotState));
	memset(pMyInput, 0, sizeof(*pMyInput));
	pMyRobotState->Robot_Init = 1;
	Comms.Init(pMyRobotState, pMyInput, pMyTargetState, pMyFieldState);
	Navigator.Init(pMyRobotState, pMyInput);
	pMyRobotState->Robot_Init = 2;
	}

void ROBOT :: Enable()
	{
	}

//AUTO
void ROBOT :: AutoInit()
	{
	Navigator.Zero();
	}

void ROBOT :: Autonomous()
	{
	}

//TELEOP
void ROBOT :: TeleInit()
	{
	Navigator.Zero();
	}

void ROBOT :: Teleop()
	{
	Comms.Get();				// get pInput data
	}

//TEST
void ROBOT :: TestInit()
	{
	//test init goes here
	Navigator.Zero();
	}

void ROBOT :: Test()
	{
	//test goes here
	}

//DISABLED
void ROBOT :: DisabledInit()
	{
	//disabled init goes here
	//Navigator.Zero();
	}

void ROBOT :: Disabled()
	{
	Comms.Get();				// get pInput data
	Navigator.SetStartingPosition();
	Navigator.ReadIMUCalStat();
	}


void ROBOT :: CheckSensors()
{
}

