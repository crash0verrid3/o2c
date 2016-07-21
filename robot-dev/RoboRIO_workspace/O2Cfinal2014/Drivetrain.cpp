#include "Drivetrain.h"

DRIVETRAIN :: DRIVETRAIN() : MotorL(LDRIVEVICTORMODULE, LDRIVEVICTORCHANNEL), MotorR(RDRIVEVICTORMODULE, RDRIVEVICTORCHANNEL), MotorB(BDRIVEVICTORMODULE, BDRIVEVICTORCHANNEL), RobotGyro(ROBOTGYROMODULE, ROBOTGYROCHANNEL)
{
	
}

void DRIVETRAIN :: Init()
{
	OrientMode = ORIENT_ROBOT;
	SmartDashboard::PutNumber("DriveMode", OrientMode);
	RobotGyro.Reset();
	Break();
	mag = 0;
	angle = 0;
	rot = 0;
	ActiveState = 5;
	RobotGyro.SetSensitivity(0.007);
	FromRobotMode = true;
}

/*
void DRIVETRAIN :: AtonKiwiDrive(INPUT* input, double atonX, double atonY, double atonR)
{

	//figure out values of motors relative to each other		
	relativeB = (0.8 * atonX) - atonR;
	relativeL = (-0.5 * atonX) - (sqrt (3) / 2 * atonY) - atonR;
	relativeR = (-0.5 * atonX) + (sqrt (3) / 2 * atonY) - atonR;
	
	double maximum = max (max (fabs (relativeL), fabs (relativeR)), fabs (relativeB));
		
	if (fabs (maximum) >= .05)
	{
		maximum = max (fabs (maximum), 1) * (maximum / fabs ( maximum));	
		MotorL.Set(relativeL / maximum);
		MotorR.Set(relativeR / maximum);
		MotorB.Set(relativeB / maximum);
		SmartDashboard::PutNumber("MotorL", relativeL / maximum);
		SmartDashboard::PutNumber("MotorR", relativeR / maximum);
		SmartDashboard::PutNumber("MotorB", relativeB / maximum);
	}
	else
	{
		MotorL.Set(0);
		MotorR.Set(0);
		MotorB.Set(0);
	}
	
	return;
}
*/

void DRIVETRAIN :: KiwiDrive(INPUT* input)
{
	if (input->Sprint == false)
	{
		input->DriveX = input->DriveX / 2;
		input->DriveY = input->DriveY / 2;
		input->DriveR = input->DriveR / 3;
	}
	
	input->DriveR = input->DriveR + ActiveRot(input);
	
	if (false)
	{
	//figure out values of motors relative to each other		
	relativeB = (0.8 * input->DriveX);
	relativeL = (-0.5 * input->DriveX) - (sqrt (3) / 2 * input->DriveY);
	relativeR = (-0.5 * input->DriveX) + (sqrt (3) / 2 * input->DriveY);
	}
	else
	{
	Mode(input);
	mag = sqrt (pow (input->DriveX, 2) + pow (input->DriveY, 2)); //magnitude of motion
	angle = atan2 (input->DriveY, input->DriveX); //angle of motion, adjusted for gyro heading if in field mode
	rot = input->DriveR; //rotational input, adjusted if active rotation is enabled
	
	SmartDashboard::PutNumber("DriveMagnitude", mag);
	SmartDashboard::PutNumber("DriveAngle", angle);
	SmartDashboard::PutNumber("DriveRotation", rot);
	SmartDashboard::PutNumber("DriveMode", OrientMode);
	SmartDashboard::PutNumber("RobotGyro", RobotGyro.GetAngle());
	
	relativeB = cos (angle);
	relativeL = cos (angle + (2 * PI / 3));
	relativeR = cos (angle - (2 * PI / 3));
	
	double scalar = mag / (max (max (fabs (relativeL), fabs (relativeR)), fabs (relativeB)));
	relativeB = relativeB * scalar;
	relativeL = relativeL * scalar;
	relativeR = relativeR * scalar;
	SmartDashboard::PutNumber("Scalar", scalar);
	                                              
	}
	
	//add rotation and scale
	relativeB = relativeB -input->DriveR;
	relativeL = relativeL -input->DriveR;
	relativeR = relativeR -input->DriveR;
	SmartDashboard::PutNumber("RelativeB", relativeB);
	SmartDashboard::PutNumber("RelativeL", relativeL);
	SmartDashboard::PutNumber("RelativeR", relativeR);
	
	double maximum = max (max (fabs (relativeL), fabs (relativeR)), fabs (relativeB));
	
	if (fabs (maximum) >= .08)
	{
		maximum = max (fabs (maximum), 1) * (maximum / fabs ( maximum));	
		MotorL.Set(relativeL / maximum);
		MotorR.Set(relativeR / maximum);
		MotorB.Set(relativeB / maximum);
		SmartDashboard::PutNumber("MotorL", relativeL / maximum);
		SmartDashboard::PutNumber("MotorR", relativeR / maximum);
		SmartDashboard::PutNumber("MotorB", relativeB / maximum);
	}
	else
	{
		MotorL.Set(0);
		MotorR.Set(0);
		MotorB.Set(0);
		SmartDashboard::PutNumber("MotorL", 0);
		SmartDashboard::PutNumber("MotorR", 0);
		SmartDashboard::PutNumber("MotorB", 0);
	}
	SmartDashboard::PutNumber("GyroAngle", RobotGyro.GetAngle());
}

double DRIVETRAIN :: ActiveRot(INPUT* input)
{
	if (input->ActiveOff == true)
	{
		ActiveState = 4;
	}
	switch (ActiveState)
	{
		case 1: //no active rot
			if (fabs (input->DriveR) <= .1)
			{
				ActiveState = 2;
			}
			SmartDashboard::PutNumber("ActiveState", ActiveState);
			return 0;
		break;
		
		case 2: //just entering active mode
			ActiveHeading = RobotGyro.GetAngle();
			ActiveState = 3;
			if (fabs (input->DriveR) > 0.1)
			{
				ActiveState = 1;
			}
			SmartDashboard::PutNumber("ActiveState", ActiveState);
			return 0;
		break;
		
		case 3: 
			if (fabs (input->DriveR) > 0.1)
			{
				ActiveState = 1;
			}
			SmartDashboard::PutNumber("ActiveState", ActiveState);
			double pow = (ActiveHeading - RobotGyro.GetAngle()) / ACTIVESENSE;
			if (pow > 0.3)
			{
				pow = 0.3;
			}
			if (pow < -0.3)
			{
				pow = -0.3;
			}
			SmartDashboard::PutNumber("ActivePower", pow);
			return pow;
		break;
		
		case 4:
			if (input->ActiveOff == false)
			{
				ActiveState = 5;
			}
			return 0;
		break;
		
		case 5:
			if (input->ActiveRotation == true)
			{
				ActiveState = 2;
			}
			return 0;
		break;
	}
	SmartDashboard::PutNumber("ActiveState", ActiveState);
	return 0;
}

void DRIVETRAIN :: Mode(INPUT* input)
{
	//state machine handling field-oriented and robot-oriented driving
	SmartDashboard::PutNumber("ModeButton", input->FieldMode);
	SmartDashboard::PutNumber("DriveMode", OrientMode);
	switch (OrientMode)
	{
		case ORIENT_BTNHELD:
			if (input->FieldMode == false)
			{
				if (FromRobotMode == true)
				{
					OrientMode = ORIENT_FIELD;
				}
				else
				{
					OrientMode = ORIENT_ROBOT;
				}
			}
			
		break;
		
		case ORIENT_ROBOT:
			FromRobotMode = true;
			if (input->FieldMode == true)
			{
				OrientMode = ORIENT_BTNHELD;
			}
		break;
		
		case ORIENT_FIELD: //actually just reverse direction
			FromRobotMode = false;
			if (input->FieldMode == true)
			{
				OrientMode = ORIENT_BTNHELD;
			}
			input->DriveX = -1 * input->DriveX;
			input->DriveY = -1 * input->DriveY;
		break;
	}
	
}

void DRIVETRAIN :: Break()
{
	//stop the motors
	MotorL.Set(0);
	MotorR.Set(0);
	MotorB.Set(0);
}
