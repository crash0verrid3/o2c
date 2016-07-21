/*
 * ToteWheels.cpp
 *
 *  Created on: Jan 31, 2015
 *      Author: Rod
 */

#include <ToteWheels.h>


TOTEWHEELS :: TOTEWHEELS()
	{
	initcnt = 0;
	left_cycle_count = right_cycle_count = 0;
	left_accum = right_accum = 0;
	left_duty_cycle = right_duty_cycle = WHEELARMS_DUTY_CYCLE;
	left_wheel_power = right_wheel_power = 0;
	pWheelLeftArm = new DoubleSolenoid(WHEELS_LEFT_ARM_FWD_CHANNEL, WHEELS_LEFT_ARM_REV_CHANNEL);
	pWheelRightArm = new DoubleSolenoid(WHEELS_RIGHT_ARM_FWD_CHANNEL, WHEELS_RIGHT_ARM_REV_CHANNEL);
	pWheelLeftMotor = new Victor(TOTEWHEEL_LEFT_CHANNEL);
	pWheelRightMotor = new Victor(TOTEWHEEL_RIGHT_CHANNEL);
	}

void TOTEWHEELS :: Init(RobotStateBuffer *pRobotState, INPUT *pInput)
	{
	SmartDashboard::PutNumber("ToteWheelsInit", ++initcnt);
	pMyRobotState = pRobotState;
	pMyInput = pInput;
	pMyRobotState->ToteWheelArm_Left_Position = WHEELARMS_GRIP;
	pMyRobotState->ToteWheelArm_Right_Position = WHEELARMS_GRIP;
	pMyRobotState->ToteWheels_Left_Speed = 0;
	pMyRobotState->ToteWheels_Right_Speed = 0;
	pWheelLeftArm->Set(DoubleSolenoid::kOff);
	pWheelRightArm->Set(DoubleSolenoid::kOff);
	SmartDashboard::PutNumber("ToteWheelsInit", ++initcnt);
	pMyRobotState->ToteWheels_Init = initcnt;
	}

void TOTEWHEELS :: IndexArms()
	{
	// This method is called during each Disabled state cycle.
	// It will gently move the tote wheel arms to the full open position,
	// then gently move them back to the normal position.
	pWheelLeftArm->Set(DoubleSolenoid::kOff);
	pWheelRightArm->Set(DoubleSolenoid::kOff);
	if (initcnt < 3)
		{	// The arms need to be moved to the full open position
		if (left_cycle_count < WHEELARMS_MAX_CYCLES * 2)	// * 2 to ensure full retraction from whatever position they started
			{	// not done moving arms to the full open position
			left_accum += WHEELARMS_INIT_DUTY_CYCLE;
			if (left_accum >= 100)
				{	// duty cycle is a turn-on
				pWheelLeftArm->Set(DoubleSolenoid::kReverse);		// pull the tote wheel arm back
				pWheelRightArm->Set(DoubleSolenoid::kReverse);		// pull the tote wheel arm back
				left_accum -= 100;
				left_cycle_count++;
				pMyRobotState->ToteWheelArm_Left_Position--;		// set the arm position state
				pMyRobotState->ToteWheelArm_Right_Position--;		// set the arm position state
				}
			}
		else
			{	// the arms are now (hopefully) fully retracted (index to 0)
			initcnt++;
			pMyRobotState->ToteWheelArm_Left_Position = 0;		// set the arm position state
			pMyRobotState->ToteWheelArm_Right_Position = 0;		// set the arm position state
			}
		}
	else if (initcnt < 4)
		{	// The arms need to be moved to the WHEELARMS_NORMAL position
		if (pMyRobotState->ToteWheelArm_Left_Position < WHEELARMS_NORMAL)
			{	// not done moving arms to the WHEELARMS_NORMAL position
			left_accum += WHEELARMS_INIT_DUTY_CYCLE;
			if (left_accum >= 100)
				{	// duty cycle is a turn-on
				pWheelLeftArm->Set(DoubleSolenoid::kForward);		// push the tote wheel arm forward
				pWheelRightArm->Set(DoubleSolenoid::kForward);		// push the tote wheel arm forward
				left_accum -= 100;
				pMyRobotState->ToteWheelArm_Left_Position++;		// set the arm position state
				pMyRobotState->ToteWheelArm_Right_Position++;		// set the arm position state
				}
			}
		else
			{	// the arms are now (hopefully) in the WHEELARMS_NORMAL position
			initcnt++;
			left_accum = 0;
			left_cycle_count = 0;
			}
		}
	}



/*
 * control inputs from Joystick2 will run the totewheels manually
 * 		LeftArm sets the duty_cycle to be applied to the WheelLeftArm DoubleSolenoid control
 * 		LeftWheel sets the power to the WheelLeftMotor Victor
 * 		RightArm sets the duty_cycle to be applied to the WheelRightArm DoubleSolenoid control
 * 		RightWheel sets the power to the WheelRightMotor Victor
 *
 * 	RobotState is updated with each cycle
 */

void TOTEWHEELS :: Run()
	{
	pWheelLeftArm->Set(DoubleSolenoid::kOff);
	pWheelRightArm->Set(DoubleSolenoid::kOff);

	float leftabs = fabs(pMyInput->LeftArm);
	if (leftabs > 0.05)						// dead zone +/- 0.05
		{
		left_accum += (WHEELARMS_DUTY_CYCLE * leftabs);
		if (left_accum >= 100)
			{	// duty cycle is a turn-on
			if (pMyInput->LeftArm < 0)
				{
				pWheelLeftArm->Set(DoubleSolenoid::kReverse);		// pull the tote wheel arm back
				pMyRobotState->ToteWheelArm_Left_Position--;		// set the arm position state
				pMyRobotState->ToteWheelArm_Left_Position = WHEELARMS_GRIP;
				}
			else if (pMyInput->LeftArm > 0)
				{
				pWheelLeftArm->Set(DoubleSolenoid::kForward);		// push the tote wheel arm forward
				pMyRobotState->ToteWheelArm_Left_Position++;		// set the arm position state
				pMyRobotState->ToteWheelArm_Left_Position = WHEELARMS_WIDE_OPEN;
				}
			left_accum -= 100;
			}
		}

	float rightabs = fabs(pMyInput->RightArm);
	if (rightabs > 0.05)						// dead zone +/- 0.05
		{
		right_accum += (WHEELARMS_DUTY_CYCLE * rightabs);
		if (right_accum >= 100)
			{	// duty cycle is a turn-on
			if (pMyInput->RightArm < 0)						// dead zone +/- 0.05
				{
				pWheelRightArm->Set(DoubleSolenoid::kReverse);		// pull the tote wheel arm back
				pMyRobotState->ToteWheelArm_Right_Position--;		// set the arm position state
				pMyRobotState->ToteWheelArm_Right_Position = WHEELARMS_GRIP;
				}
			else if (pMyInput->RightArm > 0)						// dead zone +/- 0.05
				{
				pWheelRightArm->Set(DoubleSolenoid::kForward);		// push the tote wheel arm forward
				pMyRobotState->ToteWheelArm_Right_Position++;		// set the arm position state
				pMyRobotState->ToteWheelArm_Right_Position = WHEELARMS_WIDE_OPEN;
				}
			right_accum -= 100;
			}
		}

	float lpower = 0;
	if (fabs(pMyInput->LeftWheel) > 0.05)						// dead zone +/- 0.05
		{
		lpower = pMyInput->LeftWheel * (pMyInput->LeftWheel > 0 ? TOTEWHEEL_LEFT_IN_POWER : TOTEWHEEL_LEFT_OUT_POWER) / 100;
		}
	pWheelLeftMotor->Set(lpower);			// run the left tote wheel per the joystick position
	pMyRobotState->ToteWheels_Left_Speed = lpower;

	float rpower = 0;
	if (fabs(pMyInput->RightWheel) > 0.05)						// dead zone +/- 0.05
		{
		rpower = pMyInput->RightWheel * (pMyInput->RightWheel > 0 ? TOTEWHEEL_RIGHT_IN_POWER : TOTEWHEEL_RIGHT_OUT_POWER) / 100;
		}
	pWheelRightMotor->Set(rpower);			// run the left tote wheel per the joystick position
	pMyRobotState->ToteWheels_Right_Speed = rpower;
	}

