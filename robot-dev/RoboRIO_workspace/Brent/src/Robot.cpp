#include "Robot.h"

ROBOT :: ROBOT() : PhotoSensor(PHOTOCHANNEL), UltraSonic(ULTRASONICCHANNEL)
	{
	//init variables
	robotinitcnt = 0;
	TwoBallMode = 0;
	forktarget = 0;
	forkposition = 0;
	prevposition = 0;
	forkpower = 0;
	ultrasonicrange = 0;
	hotgoal = false;
	AutoState = -1;
	TempTimerVal = 0;
	hasInited = false;
	}


void ROBOT :: Init()
	{
	robotinitcnt++;
	Comms.Init();
	Navigator.Init();
	Drivetrain.Init();
	//Catapult.Init();
	//Forklift.Init();
	//UltraSonic.SetOversampleBits(ULTRASONICOVERSAMPLEBITS);
	//UltraSonic.SetAverageBits(ULTRASONICAVERAGEBITS);
	robotinitcnt++;
	SmartDashboard::PutNumber("RobotInit", robotinitcnt);
	SmartDashboard::PutBoolean("ManualCat",false);
	}

//AUTO
void ROBOT :: AutoInit(INPUT* input)
	{
	//auto init goes here
	Navigator.Zero();

	SmartDashboard::PutNumber("AutoState", 0);
	TempTimerVal = 0;
	AutoTimer.Start();
	AutoTimer.Reset();
	hasInited = false;
	AutoState = -1;
	//TwoBallMode = 0; //1 for one, 2 for two, 0 for auto
	TwoBallMode = int(SmartDashboard::GetNumber("TwoBallMode"));	// default to 0 if not input
	if ((TwoBallMode < 1) || (TwoBallMode > 2))
		{
		TwoBallMode = 1;
		if (ultrasonicrange <= 60)
			{
			TwoBallMode = 2;
			}
		}
	SmartDashboard::PutNumber("TwoBallMode", TwoBallMode);
	}

void ROBOT :: Autonomous(INPUT* input)
	{
	SmartDashboard::PutNumber("AutoState", AutoState);
//	Navigator.ReadAccel();
	switch (AutoState)
		{
		case -1: //delay
			input->DriveX = 0;
			input->DriveY = 0;
			input->DriveR = 0;
			input->FieldMode = false;
			if (AutoTimer.Get() >= 1)
				{
				AutoState = 0;
				}
			break;

		case 0: //wait for hot goal OR if in 2ball mode, skip
			input->ActiveRotation = true;
			input->ActiveOff = false;
			input->DriveX = 0;
			input->DriveY = 0;
			input->DriveR = 0;
			input->fire_catapult = false;
			if (hotgoal == true or AutoTimer.Get() >= 6 or TwoBallMode == 2)
				{
				AutoState = 20;
				TempTimerVal = AutoTimer.Get();
				}
			break;

		case 20: //drive forwards
			input->ActiveRotation = false;
			input->DriveX = 0;
			input->DriveY = 2;
			input->DriveR = 0;
			if (AutoTimer.Get() - TempTimerVal >= 1.25)
				{
				AutoState = 1;
				TempTimerVal = AutoTimer.Get();
				}
			break;

		case 1: //fire
			input->fire_catapult = true;
			if (AutoTimer.Get() - TempTimerVal >= 0.5)
				{
				AutoState = 2;
				TempTimerVal = AutoTimer.Get();
				}
			break;

		case 2: //stop firing. if in 2ball mode, continue
			input->fire_catapult = false;
			if (TwoBallMode == 2)
				{
				AutoState = 3;
				TempTimerVal = AutoTimer.Get();
				}
			else
				{
				AutoState = 10;
				TempTimerVal = AutoTimer.Get();
				}
			break;

		case 3: //lower claw for .8 seconds
			input->gotodown = true;
			if (AutoTimer.Get() >= TempTimerVal + 0.8)
				{
				input->gotodown = false;
				AutoState = 4;
				TempTimerVal = AutoTimer.Get();
				}
			break;

		case 4: //drive backwards until ball is sensed or up to 1.5 seconds
			input->DriveY = -1.5;
			if ((AutoTimer.Get() - TempTimerVal >= 1.5) or (ultrasonicrange <= 12))
				{
				AutoState = 5;
				TempTimerVal = AutoTimer.Get();
				}
			break;

		case 5: //stop and pick up ball
			input->DriveY = 0;
			input->gotocatapult = true;
			if (AutoTimer.Get() - TempTimerVal >= 2)
				{
				AutoState = 6;
				TempTimerVal = AutoTimer.Get();
				}
			break;

		case 6: //drive forwards
			input->DriveY = 2;
			if (AutoTimer.Get() - TempTimerVal >= 1)
				{
				AutoState = 7;
				TempTimerVal = AutoTimer.Get();
				}
			break;

		case 7: //fire
			input->gotoscore = true;
			input->fire_catapult = true;
			AutoState = 8;
			break;

		case 8: //stop firing/moving
			input->DriveY = 2;
			if (AutoTimer.Get() - TempTimerVal >= 1)
				{
				AutoState = 11;
				input->DriveY = 0;
				input->gotoscore = false;
				}
			input->fire_catapult = false;
			break;

		case 9: //oneball: wait to move
			if (AutoTimer.Get() - TempTimerVal >= 1)
				{
				input->DriveY = 2;
				AutoState = 10;
				TempTimerVal = AutoTimer.Get();
				}
			break;

		case 10: //oneball: move
			input->DriveY = 2;
			if (AutoTimer.Get() - TempTimerVal >= 1)
				{
				input->DriveY = 0;
				AutoState = 11;
				}
			break;

		case 11: //all done

			break;
		}
	//Forklift.Run(input);			// execute lifting instructions
	Drivetrain.KiwiDrive(input);	// execute driving instructions
	//Catapult.Run(input);			// execute firing instructions
	}

//TELEOP
void ROBOT :: TeleInit()
	{
	//teleop init goes here
	Navigator.Zero();
	}

void ROBOT :: Teleop(INPUT* input)
	{
	Comms.Get(input);				// get input data
	//Forklift.Run(input);			// execute lifting instructions
	Drivetrain.KiwiDrive(input);	// execute driving instructions
	//Catapult.Run(input);			// execute firing instructions
//	Navigator.ReadAccel();
	}

//TEST
void ROBOT :: TestInit()
	{
	//test init goes here
	SmartDashboard::PutNumber("UltraSonic", (UltraSonic.GetAverageVoltage() / ULTRASONICVOLTSPERINCH) - ULTRASONICSETBACK);
	SmartDashboard::PutBoolean("TestFire",false);
	SmartDashboard::PutNumber("ShooterState", CATAPULT_INITIAL);
	SmartDashboard::PutNumber("ShooterTimer", 0);
	SmartDashboard::PutNumber("TensionDetect", 0);
	SmartDashboard::PutNumber("TensionPower", CATAPULT_TENSION_POWER);
	SmartDashboard::PutNumber("AsssistPower", CATAPULT_TENSION_ASSIST_POWER);
	SmartDashboard::PutNumber("AsssistTime", CATAPULT_TENSION_ASSIST_TIME);
	SmartDashboard::PutNumber("FirePower", CATAPULT_FIRE_POWER);

	//TwoBallMode = 0; //1 for one, 2 for two, 0 for auto
	TwoBallMode = int(SmartDashboard::GetNumber("TwoBallMode"));	// default to 0 if not input
	if ((TwoBallMode < 1) || (TwoBallMode > 2))
		{
		TwoBallMode = 1;
		if ((UltraSonic.GetAverageVoltage() / ULTRASONICVOLTSPERINCH) - ULTRASONICSETBACK <= 60)
			{
			TwoBallMode = 2;
			}
		}
	SmartDashboard::PutNumber("TwoBallMode", TwoBallMode);
	Navigator.Zero();
	}

void ROBOT :: Test(INPUT* input)
	{
	//test goes here
//	SmartDashboard::PutNumber("ShooterSwitch", Catapult.ShooterResetSwitch.Get());
//	SmartDashboard::PutBoolean("PhotoSensor", PhotoSensor.Get());
//	SmartDashboard::PutNumber("UltraSonic", (UltraSonic.GetAverageVoltage() / ULTRASONICVOLTSPERINCH) - ULTRASONICSETBACK);
	//CheckPhotoSensor();
	//UltraSonicSensor();
	//TestClaw();
	TestDrive(input);
	//TestFire(input);
//	Navigator.ReadAccel();
	}

//DISABLED
void ROBOT :: DisabledInit()
	{
	//disabled init goes here
	//Navigator.Zero();
	}

void ROBOT :: Disabled(INPUT* input)
	{
	//disabled goes here
	SmartDashboard::PutNumber("TensionDetect", Catapult.Tension_Detect.Get());
	//CheckPhotoSensor();
	//UltraSonicSensor();
	//ForkliftEncoder();
//	Navigator.ReadAccel();
	}

void ROBOT :: CheckPhotoSensor()
{
	hotgoal = PhotoSensor.Get();
	SmartDashboard::PutBoolean("PhotoSensor", hotgoal);
}

void ROBOT :: UltraSonicSensor()
{
	ultrasonicrange = (UltraSonic.GetAverageVoltage() / ULTRASONICVOLTSPERINCH) - ULTRASONICSETBACK;
	SmartDashboard::PutNumber("UltraSonic", ultrasonicrange);
}

void ROBOT :: ForkliftEncoder()
{
	forkposition = Forklift.GetClawEncoder();
	SmartDashboard::PutNumber("ClawEncoder",forkposition);
}

void ROBOT :: TestClaw()
{
	/*
	 * take input from SD"ClawPower" -1 = full speed down, +1 = full speed up
	 *                 SD"ClawState" = encoder value to set the claw position
	 *                 The encoder runs approximately 200 ticks from full up to full down
	 *
	 * begin by setting global variable stopcnt = SD"MstrTestCnt" + SD"ClawState"; then reset SD"ClawState" = 0
	 * if SD"MstrTestCnt" > stopcnt set ClawMotor = 0
	 * else set ClawMotor = SD"ClawPower"
	 */
	ForkliftEncoder();
	forktarget = SmartDashboard::GetNumber("ClawState");
	forkpower = fmin(1,fmax(-1,((forkposition - forktarget)/((10*fabs(prevposition-forkposition))+50))+(0.3*sin((forkposition-85)*PI/180))+0.05));
	SmartDashboard::PutNumber("ClawPower",forkpower);
	Forklift.Test(forkpower);
	prevposition = forkposition;
}

void ROBOT :: TestDrive(INPUT* input)
{
	input->Sprint = true;
	input->DriveX = SmartDashboard::GetNumber("DriveX");
	input->DriveY = SmartDashboard::GetNumber("DriveY");
	input->DriveR = SmartDashboard::GetNumber("DriveR");
	Drivetrain.KiwiDrive(input);
}

void ROBOT :: TestFire(INPUT* input)
{
	input->fire_catapult = SmartDashboard::GetBoolean("TestFire");
	input->catmanual = SmartDashboard::GetBoolean("ManualCat");
	SmartDashboard::PutBoolean("TestFire",false);
	Catapult.Run(input);
}

void ROBOT :: SendData()
	{
	//send data to dashboard
	}
