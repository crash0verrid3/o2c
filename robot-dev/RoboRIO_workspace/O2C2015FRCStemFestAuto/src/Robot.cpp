#include "Robot.h"

ROBOT :: ROBOT()	// : PhotoSensor(PHOTOCHANNEL), UltraSonic(ULTRASONICCHANNEL)
	{
	//init variables
	pPDP = new PowerDistributionPanel();
	pMyAutoTarget = new RobotStateBuffer;
	}


void ROBOT :: Init(RobotStateBuffer *pRobotState, INPUT *pInput)
	{
	pMyRobotState = pRobotState;
	pMyInput = pInput;
	Comms.Init(pRobotState, pInput, pMyAutoTarget);
	SmartDashboard::PutNumber("RobotInit", 1);
	Navigator.Init(pRobotState, pInput);
	Drivetrain.Init(pRobotState, pInput);
	ToteLift.Init(pRobotState,pInput);
	ToteWheels.Init(pRobotState, pInput);
//	Pincer.Init(pRobotState, pInput);
	AutoClass.Init(pRobotState, pInput, pMyAutoTarget);
	//UltraSonic.SetOversampleBits(ULTRASONICOVERSAMPLEBITS);
	//UltraSonic.SetAverageBits(ULTRASONICAVERAGEBITS);
	SmartDashboard::PutNumber("RobotInit", 2);
	}

void ROBOT :: Enable()
	{
	}

//AUTO
void ROBOT :: AutoInit()
	{
	//auto init goes here
//	ToteLift.InitToteLiftState();
	AutoClass.Start();
	Navigator.Zero();
//	AutoTimer.Start();
//	AutoTimer.Reset();
	}

void ROBOT :: Autonomous()
	{
	AutoClass.RunAuto();
	Drivetrain.Drive();	// execute driving instructions
	ToteLift.Run();
	ToteWheels.Run();
	}

//TELEOP
void ROBOT :: TeleInit()
	{
	//teleop init goes here
	AutoClass.Stop();
	Navigator.Zero();
//	ToteLift.InitToteLiftState();
	}

void ROBOT :: Teleop()
	{
	Comms.Get();				// get pInput data
	Drivetrain.Drive();	// execute driving instructions
	ToteLift.Run();		// keep ToteLift operations before ToteWheels.  ToteLift may to override totewheels to prevent damage.
	ToteWheels.Run();
//	Pincer.Run();
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
	//CheckPhotoSensor();
	//UltraSonicSensor();
	//TestClaw();
	//TestDrive();
	//TestFire();
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
	AutoClass.SelectScenario();
	Navigator.SetStartingPosition();
	//	ToteWheels.IndexArms();
	//disabled goes here
	//CheckPhotoSensor();
	//UltraSonicSensor();
	//ForkliftEncoder();
	}

//void ROBOT :: CheckPhotoSensor()
//	{
//	}

//void ROBOT :: UltraSonicSensor()
//	{
//	}

//void ROBOT :: ForkliftEncoder()
//	{
//	}


void ROBOT :: CheckSensors()
{
	pMyRobotState->Battery_Voltage = pPDP->GetVoltage();
	pMyRobotState->Power_Consumption = pPDP->GetTotalPower();
}

