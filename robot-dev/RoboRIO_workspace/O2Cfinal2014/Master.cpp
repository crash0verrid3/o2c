#include "Master.h"

MASTER :: MASTER()
{
	
}


void MASTER :: Init()
{
	Comms.Init();
	Robot.Init();
	//init variables
}

//AUTO
void MASTER :: AutoInit(INPUT* input)
{
	//auto init goes here
	
	/*
	input->atonForwardPower = 0.0;
	input->atonRunsStart = 0;
	input->atonFireDelay = 0;
	input->atonFire = FALSE;
	input->atonReadyToFire = FALSE;
	*/
	
	
	SmartDashboard::PutNumber("AutoState", 0);
	TempTimerVal = 0;
	//Robot.Drivetrain.RobotGyro.Reset();
	AutoTimer.Start();
	AutoTimer.Reset();
	hasInited = false;
	AutoState = -1;
	TwoBallMode = 1; //1 for one, 2 for two, 3 for auto
	TwoBallMode = SmartDashboard::GetNumber("TwoBallMode");
	SmartDashboard::PutNumber("UltraClaw", Robot.UltraClaw.GetVoltage() / 0.0098);
	if (TwoBallMode == 3)
	{
		TwoBallMode = 1;
		if (Robot.UltraClaw.GetVoltage() / 0.0098 <= 60)
		{
			TwoBallMode = 2;
		}
	}
	
}

void MASTER :: Autonomous(INPUT* input)
{
	
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
			input->catmanfire = false;
			if (Robot.PhotoSensor.Get() == true or AutoTimer.Get() >= 6 or TwoBallMode == 2)
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
			input->catmanfire = true;
			if (AutoTimer.Get() - TempTimerVal >= 0.5)
			{
				AutoState = 2;
				TempTimerVal = AutoTimer.Get();
			}
		break;
		
		case 2: //stop firing. if in 2ball mode, continue
			input->catmanfire = false;
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
			input->clawmanup = true;
			if (AutoTimer.Get() >= TempTimerVal + 0.8)
			{
				input->clawmanup = false;
				AutoState = 4;
				TempTimerVal = AutoTimer.Get();
			}
		break;
		
		case 4: //drive backwards until ball is sensed or up to 1.5 seconds
			input->DriveY = -1.5;
			if (AutoTimer.Get() - TempTimerVal >= 1.5 or Robot.UltraClaw.GetVoltage() / 0.0098 <= 12)
			{
				AutoState = 5;
				TempTimerVal = AutoTimer.Get();
			}
		break;
		
		case 5: //stop and pick up ball
			input->DriveY = 0;
			input->clawmandown = true;
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
			input->clawmandown = false;
			input->clawmanup = true;
			input->catmanfire = true;
			AutoState = 8;
		break;
		
		case 8: //stop firing/moving
			input->DriveY = 2;
			if (AutoTimer.Get() - TempTimerVal >= 1)
			{
				AutoState = 11;
				input->DriveY = 0;
				input->clawmanup = false;
			}
			input->catmanfire = false;
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
		
/*
	double TimerVal = AutoTimer.Get();
	
	if (TimerVal < AUTO_MOVETIME)
	{
		input->DriveX = 0;
		input->DriveY = 2;
		input->DriveR = 0;
		input->catstateoff = true;
	}
	else if (TimerVal < AUTO_STOPTIME)
	{
		input->DriveY = 0;

	}
	else if (TimerVal < AUTO_RESETTIME)
	{
		//input->clawmandown = true;
		input->catstateoff = true;
	}
	else if (TimerVal < AUTO_FIRETIME)// && hasInited == false)
	{
		//Robot.Init();
		input->DriveY = 0;
		input->catstateoff = true;
		input->catmanpower = 1;
		input->clawmandown = false;
		input->catmanfire = true;
		hasInited = true;
	}
	else if (TimerVal < AUTO_FIREDONE)
	{
		input->catstateoff = false;
		input->catmanfire = false;
		input->catstateon = true;
	}
*/
	Robot.Drive(input);
	Robot.Shooter(input);
	Robot.Claw(input);
	SmartDashboard::PutNumber("AutoState", AutoState);
	
}

//TELEOP
void MASTER :: TeleInit()
{
	//teleop init goes here
}

void MASTER :: Teleop(INPUT* input)
{
	//get input data
	Comms.Get(input);
	//do the teleop
	//input->isAton = FALSE;
	Robot.Drive(input);
	Robot.Shooter(input);
	Robot.Claw(input);
}

//TEST
void MASTER :: TestInit()
{
	//test init goes here
//	Robot.Drivetrain.RobotGyro.
	SmartDashboard::PutNumber("TwoBallMode", 1);
	SmartDashboard::PutNumber("Sensitivity", 0.013);
}

void MASTER :: Test(INPUT* input)
{
	//test goes here
	Robot.GyroCore(input);
	SmartDashboard::PutNumber("GyroAngle", Robot.Drivetrain.RobotGyro.GetAngle());
	SmartDashboard::PutNumber("CorrectedAngle", input->RoboGyroVal);
	SmartDashboard::PutNumber("GyroRate", Robot.Drivetrain.RobotGyro.GetRate());
	SmartDashboard::PutNumber("ShooterSwitch", Robot.Catapult.ShooterResetSwitch.Get());
	SmartDashboard::PutBoolean("PhotoSensor", Robot.PhotoSensor.Get());
	SmartDashboard::PutNumber("UltraClaw", Robot.UltraClaw.GetVoltage() / 0.0098);
	Robot.Drivetrain.RobotGyro.SetSensitivity(SmartDashboard::GetNumber("Sensitivity"));
}

//DISABLED
void MASTER :: DisabledInit()
{
	//disabled init goes here
}

void MASTER :: Disabled()
{
	//disabled goes here
}

void MASTER :: SendData()
{
	//send data to dashboard
}
