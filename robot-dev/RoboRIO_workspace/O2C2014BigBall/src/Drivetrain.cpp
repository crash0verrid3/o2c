#include "Drivetrain.h"

DRIVETRAIN :: DRIVETRAIN() : MotorL(LDRIVEVICTORCHANNEL), MotorR(RDRIVEVICTORCHANNEL), MotorB(BDRIVEVICTORCHANNEL)
{
	driveinitcnt = 0;
	OrientMode = ORIENT_ROBOT;
	FromRobotMode = true;
	mag = 1;
	rot = 0;
	angle = 0;
	relativeL = 0;
	relativeR = 0;
	relativeB = 0;
	ActiveState = 5;
	ActiveHeading = 0;
	drvpow = 0;
}

void DRIVETRAIN :: Init()
{
	driveinitcnt++;
	SmartDashboard::PutNumber("DriveInit", driveinitcnt);
	SmartDashboard::PutNumber("DriveMode", OrientMode);
	Break();
	driveinitcnt++;
	SmartDashboard::PutNumber("DriveInit", driveinitcnt);
}

void DRIVETRAIN :: KiwiDrive(INPUT* input)
{
	if (input->Sprint == false)
	{
		input->DriveX = input->DriveX / 2;
		input->DriveY = input->DriveY / 2;
		input->DriveR = input->DriveR / 3;
	}

	Mode(input);
	mag = sqrt(pow(input->DriveX, 2) + pow(input->DriveY, 2)); //magnitude of motion
	angle = atan2(input->DriveX, input->DriveY); //angle of motion, adjusted for gyro heading if in field mode
	rot = input->DriveR + ActiveRot(input); //rotational input, adjusted if active rotation is enabled

	SmartDashboard::PutNumber("DriveMagnitude", mag);
	SmartDashboard::PutNumber("DriveAngle", (int(angle * 180 / PI) + 360) % 360);
	SmartDashboard::PutNumber("DriveRotation", rot);

	relativeB = sin(angle + PI);
	relativeL = sin(angle + (PI/3));
	relativeR = sin(angle - (PI/3));

	double scalar = mag / (fmax (fmax (fabs (relativeL), fabs (relativeR)), fabs (relativeB)));
	relativeB = relativeB * scalar;
	relativeL = relativeL * scalar;
	relativeR = relativeR * scalar;

	//add rotation and scale
	relativeB = relativeB + rot;
	relativeL = relativeL + rot;
	relativeR = relativeR + rot;
	double maximum = fmax (fmax (fabs (relativeL), fabs (relativeR)), fabs (relativeB));
	if (maximum < .08)
	{
		relativeB = 0;
		relativeL = 0;
		relativeR = 0;
	}
	maximum = fmax (maximum, 1);
	MotorL.Set(relativeL / maximum);
	MotorR.Set(relativeR / maximum);
	MotorB.Set(relativeB / maximum);
	SmartDashboard::PutNumber("MotorL", relativeL / maximum);
	SmartDashboard::PutNumber("MotorR", relativeR / maximum);
	SmartDashboard::PutNumber("MotorB", relativeB / maximum);
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
			ActiveHeading = input->GyroHeading;
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
			drvpow = (ActiveHeading - input->GyroHeading) / ACTIVESENSE;
			if (drvpow > 0.3)
			{
				drvpow = 0.3;
			}
			if (drvpow < -0.3)
			{
				drvpow = -0.3;
			}
			SmartDashboard::PutNumber("ActivePower", drvpow);
			return drvpow;
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
