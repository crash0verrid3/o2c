/*
 * ToteWheels.cpp
 *
 *  Created on: Jan 31, 2015
 *      Author: Rod
 */

#include <ToteWheels.h>


TOTEWHEELS :: TOTEWHEELS()
	{
	pWheelLeftArm = new DoubleSolenoid(WHEELS_LEFT_ARM_FWD_CHANNEL, WHEELS_LEFT_ARM_REV_CHANNEL);
	pWheelRightArm = new DoubleSolenoid(WHEELS_RIGHT_ARM_FWD_CHANNEL, WHEELS_RIGHT_ARM_REV_CHANNEL);
	pWheelLeftMotor = new Victor(TOTEWHEEL_LEFT_CHANNEL);
	pWheelRightMotor = new Victor(TOTEWHEEL_RIGHT_CHANNEL);
	}

void TOTEWHEELS :: Init(RobotStateBuffer *pRobotState, INPUT *pInput)
	{
	SmartDashboard::PutNumber("WheelsInit", 1);
	pMyRobotState = pRobotState;
	pMyInput = pInput;
	pMyRobotState->ToteWheelArm_Left_Position = WHEELARMS_GRIP;
	pMyRobotState->ToteWheelArm_Right_Position = WHEELARMS_GRIP;
	pMyRobotState->ToteWheelArm_Left_Target = WHEELARMS_WIDE_OPEN;
	pMyRobotState->ToteWheelArm_Right_Target = WHEELARMS_WIDE_OPEN;
	pMyRobotState->ToteWheelArm_Left_Direction = WHEELARMS_INITIALIZING;
	pMyRobotState->ToteWheelArm_Right_Direction = WHEELARMS_INITIALIZING;
	pMyRobotState->ToteWheels_Left_Speed = 0;
	pMyRobotState->ToteWheels_Right_Speed = 0;
	pWheelLeftArm->Set(DoubleSolenoid::kOff);
	pWheelRightArm->Set(DoubleSolenoid::kOff);
	SmartDashboard::PutNumber("WheelsInit", 2);
	}
/*
#define WHEELARMS_WIDE_OPEN			0		// tote wheel arm positions in degrees
#define WHEELARMS_WIDE_TOTE			60
#define WHEELARMS_NORMAL			90
#define WHEELARMS_CLOSED			110
#define WHEELARMS_PINCHED			130
#define WHEELARMS_GRIP				150
#define WHEELARMS_CLOSING			1
#define WHEELARMS_HOLDING			0
#define WHEELARMS_OPENING			-1
#define WHEELARMS_INITIALIZING		-2
#define WHEELARMS_DEGREES_PER_CYCLE	2.0	// @ 20msec per cycle equals 100 degrees per second (tune with flow control fittings)
*/

void TOTEWHEELS :: Run()
	{
	// First order of business is to update our current positions
	pMyRobotState->ToteWheelArm_Left_Position =
			fmin(WHEELARMS_GRIP,
				 fmax(WHEELARMS_WIDE_OPEN,
					  pMyRobotState->ToteWheelArm_Left_Position + ((double) pMyRobotState->ToteWheelArm_Left_Direction * WHEELARMS_DEGREES_PER_CYCLE)));
	pMyRobotState->ToteWheelArm_Right_Position =
			fmin(WHEELARMS_GRIP,
				 fmax(WHEELARMS_WIDE_OPEN,
					  pMyRobotState->ToteWheelArm_Right_Position + ((double) pMyRobotState->ToteWheelArm_Right_Direction * WHEELARMS_DEGREES_PER_CYCLE)));

	// Next, we need to interpret the inputs to decide what the target position should be
	if ((pMyRobotState->ToteWheelArm_Left_Direction == WHEELARMS_INITIALIZING)
		|| (pMyRobotState->ToteWheelArm_Right_Direction == WHEELARMS_INITIALIZING))	// complete initialization sequence before changing target
		{
		if (pMyRobotState->ToteWheelArm_Left_Position == pMyRobotState->ToteWheelArm_Left_Target)
			pMyRobotState->ToteWheelArm_Left_Direction = WHEELARMS_HOLDING;
		if (pMyRobotState->ToteWheelArm_Right_Position == pMyRobotState->ToteWheelArm_Right_Target)
			pMyRobotState->ToteWheelArm_Right_Direction = WHEELARMS_HOLDING;
		}
	else		// wheelarm initialization sequence completed, feel free to move the arms
		{
		if (fabs(pMyInput->LeftArm) > AXIS_DEADZONE)
			pMyRobotState->ToteWheelArm_Left_Target =
					fmin(WHEELARMS_GRIP,
							fmax(WHEELARMS_WIDE_OPEN,
								 pMyRobotState->ToteWheelArm_Left_Target + (pMyInput->LeftArm * WHEELARMS_DEGREES_PER_CYCLE)));
		else
			pMyRobotState->ToteWheelArm_Left_Target = pMyRobotState->ToteWheelArm_Left_Position;

		if (fabs(pMyInput->RightArm) > AXIS_DEADZONE)
			pMyRobotState->ToteWheelArm_Right_Target =
					fmin(WHEELARMS_GRIP,
							fmax(WHEELARMS_WIDE_OPEN,
								 pMyRobotState->ToteWheelArm_Right_Target + (pMyInput->RightArm * WHEELARMS_DEGREES_PER_CYCLE)));
		else
			pMyRobotState->ToteWheelArm_Right_Target = pMyRobotState->ToteWheelArm_Right_Position;
		}

	// Next, compare the target arm positions to the current arm positions and switch the valves as needed
	if (pMyRobotState->ToteWheelArm_Left_Target < pMyRobotState->ToteWheelArm_Left_Position - (AXIS_DEADZONE * WHEELARMS_DEGREES_PER_CYCLE))	// need to spread the arms
		{
		pWheelLeftArm->Set(DoubleSolenoid::kReverse);
		pMyRobotState->ToteWheelArm_Left_Direction = WHEELARMS_OPENING;
		}
	else if (pMyRobotState->ToteWheelArm_Left_Target > pMyRobotState->ToteWheelArm_Left_Position + (AXIS_DEADZONE * WHEELARMS_DEGREES_PER_CYCLE))	// need to close the arms
		{
		pWheelLeftArm->Set(DoubleSolenoid::kForward);
		pMyRobotState->ToteWheelArm_Left_Direction = WHEELARMS_CLOSING;
		}
	else
		{
		pWheelLeftArm->Set(DoubleSolenoid::kOff);
		pMyRobotState->ToteWheelArm_Left_Direction = WHEELARMS_HOLDING;
		}
	if (pMyRobotState->ToteWheelArm_Right_Target < pMyRobotState->ToteWheelArm_Right_Position - (AXIS_DEADZONE * WHEELARMS_DEGREES_PER_CYCLE))	// need to spread the arms
		{
		pWheelRightArm->Set(DoubleSolenoid::kReverse);
		pMyRobotState->ToteWheelArm_Right_Direction = WHEELARMS_OPENING;
		}
	else if (pMyRobotState->ToteWheelArm_Right_Target > pMyRobotState->ToteWheelArm_Right_Position + (AXIS_DEADZONE * WHEELARMS_DEGREES_PER_CYCLE))	// need to close the arms
		{
		pWheelRightArm->Set(DoubleSolenoid::kForward);
		pMyRobotState->ToteWheelArm_Right_Direction = WHEELARMS_CLOSING;
		}
	else
		{
		pWheelRightArm->Set(DoubleSolenoid::kOff);
		pMyRobotState->ToteWheelArm_Right_Direction = WHEELARMS_HOLDING;
		}


	float lpower = 0;
	if (fabs(pMyInput->LeftWheel) > AXIS_DEADZONE)						// dead zone +/- 0.05
		{
		lpower = pMyInput->LeftWheel * (pMyInput->LeftWheel > 0 ? TOTEWHEEL_LEFT_IN_POWER : TOTEWHEEL_LEFT_OUT_POWER) / 100;
		}
	pWheelLeftMotor->Set(lpower);			// run the left tote wheel per the joystick position
	pMyRobotState->ToteWheels_Left_Speed = lpower;

	float rpower = 0;
	if (fabs(pMyInput->RightWheel) > AXIS_DEADZONE)						// dead zone +/- 0.05
		{
		rpower = pMyInput->RightWheel * (pMyInput->RightWheel > 0 ? TOTEWHEEL_RIGHT_IN_POWER : TOTEWHEEL_RIGHT_OUT_POWER) / 100;
		}
	pWheelRightMotor->Set(rpower);			// run the left tote wheel per the joystick position
	pMyRobotState->ToteWheels_Right_Speed = rpower;
	}

