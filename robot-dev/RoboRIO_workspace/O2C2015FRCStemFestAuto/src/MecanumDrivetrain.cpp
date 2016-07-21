 #include <MecanumDrivetrain.h>

MECANUMDRIVE :: MECANUMDRIVE()
	{
	pMyRobotState = 0;
	pMyInput = 0;
	pMotorFL = new Talon(FLVICTORCHANNEL);
	pMotorBL = new Talon(BLVICTORCHANNEL);
	pMotorFR = new Talon(FRVICTORCHANNEL);
	pMotorBR = new Talon(BRVICTORCHANNEL);
	StopMotors();  // Make sure all motors are stopped.

	OrientMode = Robot;
	SprintMode = Fast;
	OrientAngle = 0.0;
	spincontrol = false;
	rotation = prev_rotation = 0;
	driftangle = prev_driftangle = 0;
	sprint_factor = 0.5;
	spin_enable_count = 0;

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
//	pRobotDrive->SetInvertedMotor(RobotDrive::kFrontLeftMotor, true);
//	pRobotDrive->SetInvertedMotor(RobotDrive::kRearLeftMotor, true);
	pRobotDrive->SetInvertedMotor(RobotDrive::kFrontRightMotor, true);
	pRobotDrive->SetInvertedMotor(RobotDrive::kRearRightMotor, true);
	}

void MECANUMDRIVE :: Init(RobotStateBuffer *pRobotState, INPUT *pInput)
	{
	SmartDashboard::PutNumber("DriveInit", 1);
	pMyRobotState = pRobotState;
	pMyInput = pInput;
	StopMotors();
	pMyRobotState->Drivetrain_Speed_Control = SprintMode;
	pMyRobotState->Drivetrain_Spin_Control = spincontrol;
	pMyRobotState->Drivetrain_Heading = OrientAngle;
	SprintCalc();  // Check for Speed Adjustments.  A.K.A.  SPRINTMODE
	OrientCalc();	// If enabled, translate joystick movements to Field oriented drive instructions
	SpinControl();	// If enabled, keep robot from spinning unless we command it to spin
	spin_enable_count = 0;
	pMyRobotState->Drivetrain_SprintFactor = sprint_factor;
	pMyRobotState->Drivetrain_Directional_Orientation = OrientMode;
	pMyRobotState->Drivetrain_SpinCorrection = rotation;
	SmartDashboard::PutNumber("DriveInit", 2);
	}

void MECANUMDRIVE :: Drive()
	{
	SprintCalc();  // Check for Speed Adjustments.  A.K.A.  SPRINTMODE
	//Reduce the Inputs by pMyRobotState->Drivetrain_Speed_Control
	/*
	 * 	double driveX, driveY, driveR;
	 *
	 */
	driveX = pMyInput->DriveX * sprint_factor * XPOWERFACTOR;
	driveY = pMyInput->DriveY * sprint_factor;
	driveR = pMyInput->DriveR * sprint_factor;
	// Now scale X and Y so neither is greater than 100%
	double scalar = 1 / fmax(1, (fmax (fabs (driveX), fabs (driveY))));
	driveX *= scalar;
	driveY *= scalar;

	//Gyro Orientation - When Implemented
	OrientCalc();
	OrientAngle = pMyRobotState->Robot_Heading * pMyRobotState->Drivetrain_Directional_Orientation;
	pMyRobotState->Drivetrain_Heading = OrientAngle;

	SpinControl();	// If enabled, keep robot from spinning unless we command it to spin
	rotation += pMyInput->DriveR; //rotational input, adjusted if spin control is active
	pMyRobotState->Drivetrain_SpinCorrection = rotation;

	//Call the MecanumDrive_Cartesian RobotDrive Function
	pRobotDrive->MecanumDrive_Cartesian(driveX, driveY, rotation, OrientAngle);
	}


void MECANUMDRIVE :: SprintCalc()
	{
	//state machine handling drivetrain speed control
	if (pMyInput->SprintModeBtn == false)	// Complete the state change only when the button is released.
		{
		pMyRobotState->Drivetrain_Speed_Control = SprintMode;
		}
	if (pMyInput->SprintModeBtn == true)	//Check for StateChange and indicate update for when Button is released.
		{
		switch (pMyRobotState->Drivetrain_Speed_Control)
			{
			case Speedy:
				SprintMode = Slow;
				break;

			case Fast:
				SprintMode = Speedy;
				break;

			case Normal:
				SprintMode = Fast;
				break;

			case Slow:
				SprintMode = Normal;
				break;
			}
		}
	sprint_factor = (double) pMyRobotState->Drivetrain_Speed_Control / (double) Speedy;
	}


void MECANUMDRIVE :: OrientCalc()
	{
	//state machine handling field-oriented and robot-oriented driving

	if (pMyInput->OrientModeBtn == false)	// Complete the state change only when the button is released.
		{
		pMyRobotState->Drivetrain_Directional_Orientation = OrientMode;
		}
	else	//Check for StateChange and indicate update for when Button is released.
		{
		switch (pMyRobotState->Drivetrain_Directional_Orientation)
			{
			case Robot:			// Robot Orient Mode
				OrientMode = Field;
				break;

			case Field:			// Field Orientation
				OrientMode = Robot;
				break;
			}
		}
	}


void MECANUMDRIVE :: SpinControl()
	{
	//state machine handling drivetrain spin control
	// if no rotational input on the right joystick, then the robot should not spin while driving.
	// this helps compensate for wheel slip and imbalances in motor controller calibration.

	// There are two parts to spin control
	//	1.  A button on the gamepad enables/disables the spin control feature
	//	2.	Any input on the rotational joystick deactivates spin control
	//		Returning the rotational joystick to (near) zero, activates spin control and sets the target robot heading
	rotation = 0;
	spin_enable_count++;
	if (pMyInput->SpinControlBtn == false)	// Complete the state change only when the button is released.
		{
		if (spincontrol && ! pMyRobotState->Drivetrain_Spin_Control)			// we are about to enable Spin Control
			{
			pMyRobotState->Drivetrain_Heading = pMyRobotState->Robot_Heading;	// don't go crazy when Spin Control gets enabled
			}
		pMyRobotState->Drivetrain_Spin_Control = spincontrol;
		}
	else	//Check for StateChange and indicate update for when Button is released.
		{
		spincontrol = (! pMyRobotState->Drivetrain_Spin_Control);	// toggle spincontrol
		}

	if (pMyRobotState->Drivetrain_Spin_Control)	//If spin control is enabled
		/*
		 * Need a new approach...  It seems that a rotational power value based solely on the driftangle is not going to do the job.
		 * Small corrections for small driftangles do not have enough power to actually move the bot.  Large corrections for large
		 * driftangles are either insufficient to keep up with the driftangle being introduced by out-of-balance drive forces, or are
		 * so large that they cause the bot to oscillate (sometimes shaking quite violently).
		 *
		 * We need to have a starting point as previously computed and then adjust based on the effect from the previous cycle.
		 * We'll keep the previous driftangle and the previous rotational power correction.
		 *
		 * If the sign of the driftangle changes, we overshot on the last correction, so adjust by cutting off 20% of this calculation.
		 * If the driftangle is not diminishing significantly, add 1% to the calculation.
		 * If the driftangle is increasing, add even more.
		 */
		{
		if (fabs(pMyInput->DriveR) > 0.08)		// deactivate spin control and update target robot heading
			{									// add prediction of angular momentum to stopping heading
			pMyRobotState->Drivetrain_Heading = pMyRobotState->Robot_Heading; // + spin_prediction;
			spin_enable_count = 0;
			driftangle = 0;
			rotation = 0;
			}
		else if (spin_enable_count < 10)		// for the first 0.2 seconds after stopping a spin
			{									// apply breaking power and update the Drivetrain_Heading
			pMyRobotState->Drivetrain_Heading = pMyRobotState->Robot_Heading;
			rotation = -1 * pMyRobotState->Robot_Spin_Rate * LOOPPERIOD / (spin_enable_count + 1);
			}
		else	// calculate the difference between the target and actual heading and adjust the rotational power
			{
			driftangle = (double) (int(1000 * (pMyRobotState->Drivetrain_Heading - pMyRobotState->Robot_Heading)) % 360000) / 1000.000;
			driftangle += (driftangle > 180 ? -360.000 : driftangle < -180 ? 360.000 : 0.000);
			rotation = fabs(driftangle) < 0.25 ? 0 : copysign(fmin(sprint_factor, (0.02 * fabs(driftangle)) + (log1p(int(fabs(driftangle)) % 10)/(fabs(ACTIVESENSE)+1.0))), driftangle);

			if (driftangle != copysign(driftangle,prev_driftangle))
				{	// we overshot the target with the previous calculation so reduce this calculation to avoid overshooting back
				rotation *= 0.6;
				}
			else if (fabs(driftangle) > fabs(0.9 * prev_driftangle))
				{	// driftangle is not diminishing quickly enough (and may actually be increasing)
				rotation = copysign(fmax(fabs(rotation), fabs(prev_rotation)),rotation) * (1.01 + ((driftangle - prev_driftangle)/copysign(0.01 + fabs(driftangle), driftangle)));
				}
			rotation = fmax(-1 * sprint_factor, fmin(sprint_factor, rotation));
			}
		}
	prev_rotation = rotation;
	prev_driftangle = driftangle;
	}


void MECANUMDRIVE :: StopMotors()
	{
	//stop the motors
	pMotorFL->Set(0);
	pMotorBL->Set(0);
	pMotorFR->Set(0);
	pMotorBR->Set(0);
//	pMyRobotState->Drivetrain_FL_Motor_Power = 0;
//	pMyRobotState->Drivetrain_FR_Motor_Power = 0;
//	pMyRobotState->Drivetrain_BL_Motor_Power = 0;
//	pMyRobotState->Drivetrain_BR_Motor_Power = 0;
	}
