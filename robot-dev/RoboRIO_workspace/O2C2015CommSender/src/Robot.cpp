#include "Robot.h"

ROBOT :: ROBOT()	// : PhotoSensor(PHOTOCHANNEL), UltraSonic(ULTRASONICCHANNEL)
	{
	//init variables
	robotinitcnt = 0;
	pPDP = new PowerDistributionPanel();
	}


void ROBOT :: Init(RobotStateBuffer *pRobotState, INPUT *pInput)
	{
	robotinitcnt++;
	pMyRobotState = pRobotState;
	pMyInput = pInput;
	Comms.Init(pRobotState, pInput);
	Navigator.Init(pRobotState, pInput);
	Drivetrain.Init(pRobotState, pInput);
	ToteLift.Init(pRobotState,pInput);
	ToteWheels.Init(pRobotState, pInput);
//	Pincer.Init(pRobotState, pInput);
	AutoClass.Init(pRobotState, pInput);
	//UltraSonic.SetOversampleBits(ULTRASONICOVERSAMPLEBITS);
	//UltraSonic.SetAverageBits(ULTRASONICAVERAGEBITS);
	SmartDashboard::PutNumber("RobotInit", ++robotinitcnt);
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
	pMyInput->ToteJoyBtn = true;
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
//	SmartDashboard::PutNumber("UltraSonic", (UltraSonic.GetAverageVoltage() / ULTRASONICVOLTSPERINCH) - ULTRASONICSETBACK);
//	SmartDashboard::PutBoolean("TestFire",false);
//	SmartDashboard::PutNumber("FirePower", CATAPULT_FIRE_POWER);
	Navigator.Zero();
	}

void ROBOT :: Test()
	{
	//test goes here
//	SmartDashboard::PutNumber("ShooterSwitch", Catapult.ShooterResetSwitch.Get());
//	SmartDashboard::PutBoolean("PhotoSensor", PhotoSensor.Get());
//	SmartDashboard::PutNumber("UltraSonic", (UltraSonic.GetAverageVoltage() / ULTRASONICVOLTSPERINCH) - ULTRASONICSETBACK);
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
	double voltage = pPDP->GetVoltage();
	pMyRobotState->Battery_Voltage = voltage;
	double power = pPDP->GetTotalPower();
	SmartDashboard::PutNumber("Battery Voltage",voltage);
	SmartDashboard::PutNumber("Power Consumption",power);
}

