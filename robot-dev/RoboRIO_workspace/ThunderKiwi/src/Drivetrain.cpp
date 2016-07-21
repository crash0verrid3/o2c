#include "Drivetrain.h"


DRIVETRAIN :: DRIVETRAIN()
{
	driveinitcnt = 0;
	OrientMode = ORIENT_ROBOT;
	NextOrientMode = ORIENT_ROBOT;
	SprintMode = SPRINT_THIRD;
	NextSprintMode = SPRINT_THIRD;
	OrientAngle = 0.0;
	pMotorFL = new Victor(FLVICTORCHANNEL);
	pMotorBL = new Victor(BLVICTORCHANNEL);
	pMotorFR = new Victor(FRVICTORCHANNEL);
	pMotorBR = new Victor(BRVICTORCHANNEL);
	StopAll();  // Make sure all motors are stopped.
	pRobotDrive = new RobotDrive(pMotorFL, pMotorBL, pMotorFR, pMotorBR);
	/* ******************************************************************
	  IF YOU NEED TO INVERT MOTORS -  USE ONE OR MORE OF THESE LINES HERE

	  	RobotDriveP->SetInvertedMotor(MotorType motor, bool isInverted);

	 	WHERE motor is one of:
			RobotDrive::kFrontLeftMotor
  			RobotDrive::kFrontRightMotor
			RobotDrive::kRearLeftMotor
			RobotDrive::kRearRightMotor

		AND isInverted is "true"

		Example: RobotDriveP->SetInvertedMotor(RobotDrive::kFrontLeftMotor, true);
	 *************************************************************** */

}

void DRIVETRAIN :: Init(RobotStateBuffer *pRobotState)
{
	driveinitcnt++;
	SmartDashboard::PutNumber("DriveInit", driveinitcnt);
	pMyRobotState = pRobotState;
	SmartDashboard::PutNumber("OrientMode", OrientMode);
	SmartDashboard::PutNumber("NextOrientMode", NextOrientMode);
}


void DRIVETRAIN :: MecanumDrive(INPUT* input)
{

	SprintCalc(input);  // Check for Speed Adjustments.  A.K.A.  SPRINTMODE

	// Show inputs including adjustments
	SmartDashboard::PutNumber("DriveX", input->DriveX);
	SmartDashboard::PutNumber("DriveY", input->DriveY);
	SmartDashboard::PutNumber("DriveR", input->DriveR);
	SmartDashboard::PutNumber("SprintModeBtn", input->SprintModeBtn);
	SmartDashboard::PutNumber("SprintMode", SprintMode);
	SmartDashboard::PutNumber("NextSprintMode", NextSprintMode);

	//Gyro Orientation - When Implemented
	OrientCalc(input);
	SmartDashboard::PutNumber("OrientModeBtn", input->OrientModeBtn);
	SmartDashboard::PutNumber("OrientMode", OrientMode);
	SmartDashboard::PutNumber("NextOrientMode", NextOrientMode);
	SmartDashboard::PutNumber("OrientAgle", OrientAngle);


	//Call the MecanumDrive_Cartesian RobotDrive Function
	pRobotDrive->MecanumDrive_Cartesian(input->DriveX, input->DriveY, input->DriveR, OrientAngle);

}

void DRIVETRAIN :: SprintCalc(INPUT* input)
{

	// Complete the transition only when the button is released.
		if (input->SprintModeBtn == false)
		{
			SprintMode = NextSprintMode;  // No need to Check. Same when not in transition.
		}

		//state machine handling field-oriented and robot-oriented driving
		switch (SprintMode)
		{
			case SPRINT_THIRD: //Adjust for HALF POWER
				//Check for StateChange and indicate update for when Button is released.
				if (input->SprintModeBtn == true)
				{
					NextSprintMode = SPRINT_HALF;
				}
				//Reduce the Inputs by HALF
				input->DriveX = input->DriveX / 3.0;
				input->DriveY = input->DriveY / 3.0;
				input->DriveR = input->DriveR / 3.0;
			break;

			case SPRINT_HALF: //Adjust for HALF POWER
				//Check for StateChange and indicate update for when Button is released.
				if (input->SprintModeBtn == true)
				{
					NextSprintMode = SPRINT_TWOTHIRDS;
				}
				//Reduce the Inputs by HALF
				input->DriveX = input->DriveX / 2.0;
				input->DriveY = input->DriveY / 2.0;
				input->DriveR = input->DriveR / 2.0;
			break;

			case SPRINT_TWOTHIRDS: //Adjust for HALF POWER
				//Check for StateChange and indicate update for when Button is released.
				if (input->SprintModeBtn == true)
				{
					NextSprintMode = SPRINT_FULL;
				}
				//Reduce the Inputs by HALF
				input->DriveX = input->DriveX / 1.5;
				input->DriveY = input->DriveY / 1.5;
				input->DriveR = input->DriveR / 1.5;
			break;

			case SPRINT_FULL:
				//Check for StateChange and indicate update for when Button is released.
				if (input->SprintModeBtn == true)
				{
					NextSprintMode = SPRINT_HALF;
				}
				// In FULL Mode, no adjustment is needed.
			break;

		}


}





void DRIVETRAIN :: OrientCalc(INPUT* input)
{


	// Complete the transition only when the button is released.
	if (input->OrientModeBtn == false)
	{
		OrientMode = NextOrientMode;  // No need to Check. Same when not in transition.
	}

	//state machine handling field-oriented and robot-oriented driving
	switch (OrientMode)
	{
		case ORIENT_ROBOT:
			//Check for StateChange and indicate update for when Button is released.
			if (input->OrientModeBtn == true)
			{
				NextOrientMode = ORIENT_FIELD;
			}

			// Robot Orient Mode
			OrientAngle = 0.0;  // Robot Orient mode is 0.0
		break;

		case ORIENT_FIELD: //Adjust for Field Orientation
			//Check for StateChange and indicate update for when Button is released.
			if (input->OrientModeBtn == true)
			{
					NextOrientMode = ORIENT_ROBOT;
			}

			// Future Field Oriented Changes Go Here.
			// For Now, default to Robot Orientation of angle 0.
			OrientAngle = pMyRobotState->Navigation_Heading;
//			OrientAngle = 0.0;
		break;
	}

}

void DRIVETRAIN :: StopAll()
{
	//stop the motors
	pMotorFL->Set(0);
	pMotorBL->Set(0);
	pMotorFR->Set(0);
	pMotorBR->Set(0);
}


