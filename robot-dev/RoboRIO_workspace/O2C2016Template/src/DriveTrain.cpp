/*
 * DriveTrain.cpp
 *
 *  Created on: Jan 25, 2016
 *      Author: Rod
 */

#include <DriveTrain.h>

/*
 * This is our own DriveTrain code.  We augment the joystick inputs with some additional features.  Specifically:
 * 		SprintMode:		Toggling the SprintModeBtn on the game controller rotates through speed control factors
 * 						Speedy, Fast, Normal, and Slow
 *
 * 		OrientMode:		This feature is not available when using TankDriveControl since TankDrive cannot strafe.
 * 						When employing a drivetrain with strafing capabilities, this feature allows the driver
 * 						to select either Field-Oriented driving or Robot-Oriented driving.
 *
 * 		SpinControl:	When the drive control(s) are NOT commanding the robot to turn, spin control will automagically
 * 						adjust wheel power settings to maintain the robot's heading.  This helps to compensate for both
 * 						wheel slipage and also differences in motor controller callibration, gearbox friction, etc.
 *
 * Dependencies.
 * 		Proper operation of this Drivetrain module requires the following values set within the RobotState buffer.
 * 		pMyRobotState->Drivetrain_Spin_Control defaults to false; The SpinControlBtn must be mapped to a game controller button
 * 		in order to enable SpinControl.
 * 		pMyRobotState->Drivetrain_Speed_Control defaults to Fast; The SprintModeBtn must be mapped to a game controller button
 * 		in order to switch the SprintMode.
 * 		pMyRobotState->Drivetrain_Directional_Orientation defaults to Robot; The OrientModeBtn must be mapped to a game controller button
 * 		in order to switch the OrientMode.
 *
 *
 * RobotState Buffer Values Updated.
 * 		pMyRobotState->DriveTrain_Init						-- 1 = Initialization started; 2 = Initialization complete
 * 		pMyRobotState->Drivetrain_Spin_Control				-- described above
 * 		pMyRobotState->Drivetrain_Speed_Control				-- described above
 * 		pMyRobotState->Drivetrain_Directional_Orientation	-- described above
 * 		pMyTargetState->Robot_Heading						-- the direction the robot is suppose to be moving when SpinControl is active
 *
	  IF YOU NEED TO INVERT MOTORS -  USE ONE OR MORE OF THESE LINES HERE

	  	RobotDriveP->SetInvertedMotor(MotorType motor, bool isInverted);

	 	WHERE motor is one of:
			RobotDrive::kFrontLeftMotor
  			RobotDrive::kFrontRightMotor
			RobotDrive::kRearLeftMotor
			RobotDrive::kRearRightMotor

		AND isInverted is "true"

		Example: RobotDriveP->SetInvertedMotor(RobotDrive::kFrontLeftMotor, true);
 */

DRIVETRAIN :: DRIVETRAIN()
	{
	#if DRIVETRAINTYPE==TankDriveTrain
		pMotorL = new Talon(LEFTDRIVEMOTOR_PWM_PORT);
		pMotorR = new Talon(RIGHTDRIVEMOTOR_PWM_PORT);
//		pGhostL = new Talon(21);							// no such PWM channel, this is a bit bucket
//		pGhostR = new Talon(22);							// no such PWM channel, this is a bit bucket
//		pRobotDrive = new RobotDrive(pGhostL, pMotorL, pGhostR, pMotorR);	// (FrontLeft, RearLeft, FrontRight, RearRight)
//		pRobotDrive = new RobotDrive(pMotorL, pGhostL, pMotorR, pGhostR);	// (FrontLeft, RearLeft, FrontRight, RearRight)
		pRobotDrive = new RobotDrive(pMotorL, pMotorR);						// (Left, Right)
		/*
		 * These SetInvertedMotor methods are not working properly and under some conditions actually cause your program to crash.
		 * Ah ha!  Figured it out!  Right Motor power initializes as inverted by default.  Inverting it again, puts it back to normal.
		 * Do not use them.
		 * Instead, we apply power inversion factor in our own Drive method (see below)
		pRobotDrive->SetInvertedMotor(RobotDrive::kFrontRightMotor, true);
		pRobotDrive->SetInvertedMotor(RobotDrive::kRearRightMotor, true);
		 *
		 */
	#elif DRIVETRAINTYPE==MecanumDriveTrain
		pMotorFL = new Talon(LEFTFRONTDRIVEMOTORCHANNEL);
		pMotorBL = new Talon(LEFTBACKDRIVEMOTORCHANNEL);
		pMotorFR = new Talon(RIGHTFRONTDRIVEMOTORCHANNEL);
		pMotorBR = new Talon(RIGHTBACKDRIVEMOTORCHANNEL);
		pRobotDrive = new RobotDrive(pMotorFL, pMotorBL, pMotorFR, pMotorBR);
		pRobotDrive->SetInvertedMotor(RobotDrive::kFrontRightMotor, true);
		pRobotDrive->SetInvertedMotor(RobotDrive::kRearRightMotor, true);
	#endif
		//	pRobotDrive->SetInvertedMotor(RobotDrive::kFrontLeftMotor, true);
		//	pRobotDrive->SetInvertedMotor(RobotDrive::kRearLeftMotor, true);
	pRobotDrive->SetExpiration(0.5);
	pRobotDrive->SetSafetyEnabled(true);
	OrientMode = Robot;
//	SprintMode = Fast;
	OrientAngle = 0.0;
	spincontrol = false;
	rotation = prev_spin_correction = 0;
	driftangle = prev_driftangle = 0;
	#if DRIVECONTROL==TankDriveControl
		left_stick  = right_stick = 0;
	#elif DRIVECONTROL==ArcadeDriveControl
		x_stick  = y_stick = spin_stick = 0;
	#endif

	sprint_factor = 0.5;
	lateral_movement_compensation_factor = 1.0;
	SprintModeBtn_enabled = SpinControlBtn_enabled = prev_sprint_btn = prev_spin_btn = false;
	SprintMode_button_debouncer = SpinControl_button_debouncer = 0;
	}

void DRIVETRAIN :: Init(RobotStateBuffer *pRobotState, RobotStateBuffer *pTargetState, INPUT *pInput)
	{
	pMyRobotState = pRobotState;
	pMyTargetState = pTargetState;
	pMyInput = pInput;
	pMyRobotState->DriveTrain_Init++;
	StopMotors();  // Make sure all motors are stopped.
//	pMyTargetState->Drivetrain_Speed_Control = Fast;
	pMyTargetState->Drivetrain_Speed_Control = Speedy;
	pMyRobotState->Drivetrain_SprintFactor = pMyTargetState->Drivetrain_SprintFactor = SprintCalc();  // Check for Speed Adjustments.  A.K.A.  SPRINTMODE
	#if DRIVETRAINTYPE==MecanumDriveTrain
		lateral_movement_compensation_factor = XPOWERFACTOR;
		OrientCalc();	// If enabled, translate joystick movements to Field oriented drive instructions
	#endif
	pMyRobotState->Drivetrain_Spin_Control = false;
//	pMyTargetState->Drivetrain_Spin_Control = true;
	pMyTargetState->Drivetrain_Spin_Control = false;
	pMyRobotState->Drivetrain_SpinCorrection = SpinControl();	// If enabled, keep robot from spinning unless we command it to spin
	StopMotors();
	pMyRobotState->DriveTrain_Init++;
	}

void DRIVETRAIN :: Drive()
	{
	pMyRobotState->Drivetrain_SpinCorrection = SpinControl();		// If enabled, keep robot from spinning unless we command it to spin

	#if DRIVETRAINTYPE==MecanumDriveTrain
		OrientCalc();	//Gyro Orientation - When Implemented
		OrientAngle = pMyRobotState->Robot_Heading * pMyRobotState->Drivetrain_Directional_Orientation;
	#endif

	pMyRobotState->Drivetrain_SprintFactor = SprintCalc(); 	// Set Speed Adjustment factor.
	#if DRIVECONTROL==TankDriveControl
		left_stick  = pMyInput->DriveL * pMyRobotState->Drivetrain_SprintFactor;
		right_stick = pMyInput->DriveR * pMyRobotState->Drivetrain_SprintFactor;
	#elif DRIVECONTROL==ArcadeDriveControl
		x_stick = pMyInput->DriveX * pMyRobotState->Drivetrain_SprintFactor * lateral_movement_compensation_factor;
		y_stick = pMyInput->DriveY * pMyRobotState->Drivetrain_SprintFactor;
		spin_stick = pMyInput->DriveRotation * pMyRobotState->Drivetrain_SprintFactor;
		// Now scale X and Y so neither is greater than 100%
		double scalar = 1 / fmax(1, (fmax (fabs (x_stick), fabs (y_stick))));
		x_stick *= scalar;
		y_stick *= scalar;
	#endif

	#if DRIVECONTROL==TankDriveControl
		/*
		 * The tankdrive method does not have a rotation or OrientAngle parameter.
		 * Why?  Because the tankdrive cannot move in any direction (cannot strafe).
		 * We will need to compensate the DriveL and DriveR power values to stay on course.
		 * And FieldOriented driving is not an option with TankDrive, is it?
		 */
		left_stick += (0.5 * pMyRobotState->Drivetrain_SpinCorrection  * pMyRobotState->Drivetrain_SprintFactor);
		right_stick -= (0.5 * pMyRobotState->Drivetrain_SpinCorrection  * pMyRobotState->Drivetrain_SprintFactor);
		// Now compensate for an imbalance between left and right drive effeciencies
		left_stick = (left_stick + DRIVEPOWERBALANCEOFFSET) * DRIVEPOWERBALANCEFACTOR;
		// Now scale L and R so neither is greater than 100%
		double scalar = 1 / fmax(1, (fmax (fabs (left_stick), fabs (right_stick))));
		left_stick *= scalar;
		right_stick *= scalar;
	#elif DRIVECONTROL==ArcadeDriveControl
		spin_stick += pMyRobotState->Drivetrain_SpinCorrection; //rotational input, adjusted if spin control is active
	#endif

	/*
	 * There is an apparent gap here.  If we try to use a TankDriveTrain with an ArcadeDriveControl, we will have troubles.
	 * I'm leaving this trouble for someone else to worry about later (if the team ever actually decides to do try something
	 * like that).
	 */

	#if DRIVETRAINTYPE==TankDriveTrain
		pMyRobotState->Drivetrain_Left_Motor_Power = left_stick;
		pMyRobotState->Drivetrain_Right_Motor_Power = right_stick;
		if (INVERTLEFTDRIVEMOTORPOWER) left_stick *= -1;
		if (INVERTRIGHTDRIVEMOTORPOWER) right_stick *= -1;
		pRobotDrive->TankDrive(left_stick, right_stick, false);	// change false to true to decrease sensitivity at lower power settings
	#elif DRIVETRAINTYPE==MecanumDriveTrain
//		pMyRobotState->Drivetrain_FL_Motor_Power = (x_stick * trig_funcion(OrientAngle)) + (y_stick * trig_function(OrientAngle)) +|- pMyRobotState->Drivetrain_SpinCorrection;
//		pMyRobotState->Drivetrain_FR_Motor_Power = (x_stick * trig_funcion(OrientAngle)) + (y_stick * trig_function(OrientAngle)) +|- pMyRobotState->Drivetrain_SpinCorrection;
//		pMyRobotState->Drivetrain_BL_Motor_Power = (x_stick * trig_funcion(OrientAngle)) + (y_stick * trig_function(OrientAngle)) +|- pMyRobotState->Drivetrain_SpinCorrection;
//		pMyRobotState->Drivetrain_BR_Motor_Power = (x_stick * trig_funcion(OrientAngle)) + (y_stick * trig_function(OrientAngle)) +|- pMyRobotState->Drivetrain_SpinCorrection;
//		if (INVERTLEFTFRONTDRIVEMOTORPOWER) pMyInput->DriveLF *= -1;
//		if (INVERTRIGHTFRONTDRIVEMOTORPOWER) pMyInput->DriveRF *= -1;
//		if (INVERTLEFTREARDRIVEMOTORPOWER) pMyInput->DriveLR *= -1;
//		if (INVERTRIGHTREARDRIVEMOTORPOWER) pMyInput->DriveRR *= -1;
		//Call the MecanumDrive_Cartesian RobotDrive Function
		pRobotDrive->MecanumDrive_Cartesian(x_stick, y_stick, spin_stick, OrientAngle);
	#endif
	}


double DRIVETRAIN :: SprintCalc()
	{
	//state machine handling drivetrain speed control
	SprintModeBtn_enabled = (SprintMode_button_debouncer-- < 1);
	if (SprintModeBtn_enabled)			// if the intake button is enabled, check to see if it went from released to pressed
		{
		if ((pMyInput->SprintModeBtn) && (! prev_sprint_btn))
			{
			switch (pMyRobotState->Drivetrain_Speed_Control)
				{
				case Speedy:
					pMyTargetState->Drivetrain_Speed_Control = Slow;
					break;

				case Fast:
					pMyTargetState->Drivetrain_Speed_Control = Speedy;
					break;

				case Normal:
					pMyTargetState->Drivetrain_Speed_Control = Fast;
					break;

				case Slow:
					pMyTargetState->Drivetrain_Speed_Control = Normal;
					break;
				}
			}
		if (pMyInput->SprintModeBtn != prev_sprint_btn)			// whenever the button changes state, disable it for 5 cycles
			{
			SprintMode_button_debouncer = 5;
			}
		prev_sprint_btn = pMyInput->SprintModeBtn;
		}
	pMyRobotState->Drivetrain_Speed_Control = pMyTargetState->Drivetrain_Speed_Control;
	return (double) pMyRobotState->Drivetrain_Speed_Control / (double) Speedy;
	}


double DRIVETRAIN :: SpinControl()
	{
	/*
	 * SpinControl() manages the state of the SpinControl feature (disabled, enabled, active, inactive).
	 * If SpinControl is enabled and active, SpinControl() calls SpinCalc() to obtain a spin correction value.
	 * Positive values turn the robot clockwise.  Negative values turn the robot counterclockwise.
	 *
	 * Side Effect: When SpinControl is activated, the pMyTargetState->Robot_Heading value
	 * is updated automatically whenever the robot is spinning faster than the preset threshold.
	 * The pMyTargetState->Robot_Heading is the reference value used by the SpinControl() method to determine
	 * robot directional drift (and correction factor) in TeleOp mode.
	 *
	 * The SpinControl feature must be enabled by the driver in order to operate.
	 * Once enabled, it is either active or inactive, based on driver control dynamics.
	 * For ArcadeDriveControl, it is inactive when the spin input control is outside the dead zone.
	 * For TankDriveControl, it is inactive when the left and right drive power controls are in differnt positions.
	 * SpinControl is automatically re-activated, when the above conditions are no longer true AND the robot
	 * has a chance to stop spinning at a high rate (spin rate drops below a set threshold).
	 */
	double spin_power = 0;
	SpinControlBtn_enabled = (SpinControl_button_debouncer-- < 1);
	if (SpinControlBtn_enabled)			// if the SpinControl button is enabled, check to see if it went from released to pressed
		{
		if ((pMyInput->SpinControlBtn) && (! prev_spin_btn))	// whenever we first push the button
			{
			pMyTargetState->Drivetrain_Spin_Control = (! pMyRobotState->Drivetrain_Spin_Control);	// toggle spincontrol
			}
		if (pMyInput->SpinControlBtn != prev_spin_btn)			// whenever the button changes state, disable it for 5 cycles and set the target heading
			{
			SpinControl_button_debouncer = 5;
			pMyTargetState->Robot_Heading = pMyRobotState->Robot_Heading;
			}
		prev_spin_btn = pMyInput->SpinControlBtn;
		}
	pMyRobotState->Drivetrain_Spin_Control = pMyTargetState->Drivetrain_Spin_Control;
	if (pMyTargetState->Drivetrain_Spin_Control)	//If spin control is supposed to be enabled,
		{											//	update target heading during turns or calculate correction for straight line driving
		#if DRIVECONTROL==TankDriveControl
			spin_power = pMyInput->DriveL - pMyInput->DriveR;		// this may need some tweaking for desired effect
		#elif DRIVECONTROL==ArcadeDriveControl
			spin_power = pMyInput->DriveRotation;
		#endif
		if (fabs(spin_power) > 0.08)		// as we are directing the robot to turn, we need to continously update target robot heading
			{
			pMyTargetState->Robot_Heading = pMyRobotState->Robot_Heading + (pMyRobotState->Robot_Spin_Rate / 50); // 50 is kind of arbitrary (may need to adjust);
			pMyTargetState->Robot_Heading += (pMyTargetState->Robot_Heading < 0) ? TWO_PI : 0;
			pMyTargetState->Robot_Heading -= (pMyTargetState->Robot_Heading > TWO_PI) ? TWO_PI : 0;
			pMyRobotState->Drivetrain_DriftAngle = 0;
			}
		else return SpinCalc();
		}
	return 0;
	}

double DRIVETRAIN :: SpinCalc()
	{

	/*
	 * returns a value between -1 and +1 depending on DriftAngle (= pMyTargetState->Robot_Heading - pMyRobotState->Robot_Heading)
	 * Negative values correct counter-clockwise; positive values correct clockwise.
	 */
	double spin_correction = 0;
	pMyRobotState->Drivetrain_DriftAngle = pMyTargetState->Robot_Heading - pMyRobotState->Robot_Heading;		// between -TWO_PI and +TWO_PI
	if (pMyRobotState->Drivetrain_DriftAngle > PI) pMyRobotState->Drivetrain_DriftAngle -= TWO_PI;
	else if (pMyRobotState->Drivetrain_DriftAngle < -1 * PI) pMyRobotState->Drivetrain_DriftAngle += TWO_PI;	// always turn the shortest direction

	/*
		minimum value will be MINIMUM_POWER_TO_MOVE but with the appropriate directional sign
		maximum value should not cause us to overshoot
			if the robot can spin at 10 radians per second (~573 degrees per second)
			then, in one cycle, it could spin ~11.5 degrees
			But, it would take two cycles to stop it from spinning; so max spin rate would turn the robot ~23 degrees (.5 radians)
			So, anything less than a 0.5 DriftAngle will need to generate less than 100% spin_correction

		Compute the correction value based on the DriftAngle, then normalize it between MINIMUM_POWER_TO_MOVE...1
		Finally, set the sign for the correct spin direction
	*/

	// apply a spin_correction when DriftAngle exceeds a quarter of a degree
	if (fabs(pMyRobotState->Drivetrain_DriftAngle) < (0.25 * DEGREES_TO_RADIANS)) spin_correction = 0;
	else
		{
		spin_correction = (fmin(0.5,fabs(pMyRobotState->Drivetrain_DriftAngle * ACTIVESENSE)) * ( 1 - MINIMUM_POWER_TO_MOVE)) + MINIMUM_POWER_TO_MOVE;
		spin_correction = copysign(spin_correction, pMyRobotState->Drivetrain_DriftAngle);
		}

	if (pMyRobotState->Drivetrain_DriftAngle != copysign(pMyRobotState->Drivetrain_DriftAngle,prev_driftangle))
		{	// we overshot the target with the previous calculation so reduce this calculation to avoid overshooting back
		spin_correction *= 0.6;
		}
	else if (fabs(pMyRobotState->Drivetrain_DriftAngle) > fabs(0.9 * prev_driftangle))
		{	// driftangle is not diminishing quickly enough (and may actually be increasing)
		spin_correction = copysign(fmax(fabs(spin_correction), fabs(prev_spin_correction)),spin_correction)
						* (1.01 + ((pMyRobotState->Drivetrain_DriftAngle - prev_driftangle)
									/copysign(0.01 + fabs(pMyRobotState->Drivetrain_DriftAngle), pMyRobotState->Drivetrain_DriftAngle)));
		}
	prev_spin_correction = spin_correction;
	prev_driftangle = pMyRobotState->Drivetrain_DriftAngle;
	return spin_correction;
	}


	/*
						// need to add differential of drive wheel encoders to the following spin check ?
			else if (fabs(pMyRobotState->Robot_Spin_Rate) > SPIN_RATE_STABILITY_THRESHOLD)	// still spinning (due to angular momentum)
				{									// apply breaking power and update the pMyTargetState->Robot_Heading
				pMyTargetState->Robot_Heading = pMyRobotState->Robot_Heading;
				spin_correction = -1 * pMyRobotState->Robot_Spin_Rate / (fabs(ACTIVESENSE)+1.0);		// * LOOPPERIOD
				}
			else	// calculate the difference between the target and actual heading and adjust the rotational power
				{
				pMyRobotState->Drivetrain_DriftAngle = pMyTargetState->Robot_Heading - pMyRobotState->Robot_Heading;		// between -TWO_PI and +TWO_PI
				if (pMyRobotState->Drivetrain_DriftAngle > PI) pMyRobotState->Drivetrain_DriftAngle -= TWO_PI;
				else if (pMyRobotState->Drivetrain_DriftAngle < -1 * PI) pMyRobotState->Drivetrain_DriftAngle += TWO_PI;							// always turn the shortest direction

				spin_correction = fabs(pMyRobotState->Drivetrain_DriftAngle) < 0.25 ?
										0 :
										copysign(fmin(sprint_factor, (0.02 * fabs(pMyRobotState->Drivetrain_DriftAngle))
																		+ (log1p(int(fabs(pMyRobotState->Drivetrain_DriftAngle)) % 10)/(fabs(ACTIVESENSE)+1.0))),
													pMyRobotState->Drivetrain_DriftAngle);
				if (pMyRobotState->Drivetrain_DriftAngle != copysign(pMyRobotState->Drivetrain_DriftAngle,prev_driftangle))
					{	// we overshot the target with the previous calculation so reduce this calculation to avoid overshooting back
					spin_correction *= 0.6;
					}
				else if (fabs(pMyRobotState->Drivetrain_DriftAngle) > fabs(0.9 * prev_driftangle))
					{	// driftangle is not diminishing quickly enough (and may actually be increasing)
					spin_correction = copysign(fmax(fabs(spin_correction), fabs(prev_spin_correction)),spin_correction)
									* (1.01 + ((pMyRobotState->Drivetrain_DriftAngle - prev_driftangle)
												/copysign(0.01 + fabs(pMyRobotState->Drivetrain_DriftAngle), pMyRobotState->Drivetrain_DriftAngle)));
					}
				spin_correction = fmax(-1 * sprint_factor, fmin(sprint_factor, spin_correction));
				}
			*/


void DRIVETRAIN :: StopMotors()
	{
	//stop the motors
	#if DRIVETRAINTYPE==TankDriveTrain
		pMotorL->Set(0);
		pMotorR->Set(0);
		pMyRobotState->Drivetrain_Left_Motor_Power = 0;
		pMyRobotState->Drivetrain_Right_Motor_Power = 0;
	#elif DRIVETRAINTYPE==MecanumDriveTrain
		pMotorFL->Set(0);
		pMotorBL->Set(0);
		pMotorFR->Set(0);
		pMotorBR->Set(0);
		pMyRobotState->Drivetrain_FL_Motor_Power = 0;
		pMyRobotState->Drivetrain_FR_Motor_Power = 0;
		pMyRobotState->Drivetrain_BL_Motor_Power = 0;
		pMyRobotState->Drivetrain_BR_Motor_Power = 0;
	#endif
	}


#if DRIVETRAINTYPE==MecanumDriveTrain
void DRIVETRAIN :: OrientCalc()
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
#endif

