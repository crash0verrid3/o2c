#include <ToteLift.h>

TOTELIFT :: TOTELIFT()
	{
	pToteLifter = new DoubleSolenoid(LIFTER_UP_CHANNEL, LIFTER_DN_CHANNEL);
	}

void TOTELIFT :: Init(RobotStateBuffer *pRobotState, INPUT *pInput)
	{
	SmartDashboard::PutNumber("LiftInit", 1);
	pMyRobotState = pRobotState;
	pMyInput = pInput;
	pMyRobotState->ToteLift_Direction = LIFT_INITIALIZE;
	pMyRobotState->ToteLift_Position = LIFTER_PLATFORM;		// assume lifter is not really all the way down at start
	pMyRobotState->ToteLift_Target = 0;
	pToteLifter->Set(DoubleSolenoid::kOff);
	SmartDashboard::PutNumber("LiftInit", 2);
	}

/*
#define LIFTER_FULL_UP	24				// 24 inches off the ground; pneumatic actuator is fully retracted
#define LIFTER_STACK	14				// 14 inches off the ground; clearance to stack on top of another tote
#define LIFTER_PLATFORM	4				// 24 inches off the ground; minimum height to place on the stacking platform
#define LIFTER_FULL_DN	0				// 0 inches off the ground; pneumatic actuator is fully extended
#define LIFTER_INCHES_PER_CYCLE	0.15	// @ 20msec per cycle equals 7.5 inches per second (tune with flow control fittings)
#define LIFTER_LEAKAGE	0.01			// assumes pneumatics will leak a bit (tune this number for actual leakage)
#define LIFT_UP			1
#define LIFT_HOLD		0
#define LIFT_DN 		-1
#define LIFT_INITIALIZE	-2
*/

void TOTELIFT :: Run()
	{
	// First order of business is to update our current position
	// This is totally a guess, based on the direction of movement and assuming the flow control valves are set
	// to provide about 7.5 inches per second of travel.
	pMyRobotState->ToteLift_Position =
			fmin(LIFTER_FULL_UP,
				 fmax(LIFTER_FULL_DN,
					  pMyRobotState->ToteLift_Position + ((double) pMyRobotState->ToteLift_Direction * LIFTER_INCHES_PER_CYCLE))); //- LIFTER_LEAKAGE));

	// Next, we need to interpret the inputs to decide what the target position should be
 	if (pMyRobotState->ToteLift_Direction == LIFT_INITIALIZE)	// complete initialization sequence before changing target
		{
 //		SmartDashboard::PutNumber("LiftInit", 3);
		if (pMyRobotState->ToteLift_Position == pMyRobotState->ToteLift_Target)
			pMyRobotState->ToteLift_Direction = LIFT_HOLD;
		}
	else		// lift initialization sequence completed, feel free to move the lift
		{
//		SmartDashboard::PutNumber("LiftInit", 4);
		if (fabs(pMyInput->ToteLift) > AXIS_DEADZONE)
			pMyRobotState->ToteLift_Target =
					fmin(LIFTER_FULL_UP,
							fmax(LIFTER_FULL_DN,
									pMyRobotState->ToteLift_Target + (pMyInput->ToteLift * LIFTER_INCHES_PER_CYCLE)));
		else
			pMyRobotState->ToteLift_Target = pMyRobotState->ToteLift_Position;

		if (pMyInput->ToteUPBtn == true)
			{
			if (pMyRobotState->ToteLift_Direction == LIFT_DN)
				pMyRobotState->ToteLift_Target = pMyRobotState->ToteLift_Position;
			else
				pMyRobotState->ToteLift_Target = LIFTER_FULL_UP;
			}
		if (pMyInput->ToteDNBtn == true)
			{
			if (pMyRobotState->ToteLift_Direction == LIFT_UP)
				pMyRobotState->ToteLift_Target = pMyRobotState->ToteLift_Position;
			else
				pMyRobotState->ToteLift_Target = LIFTER_FULL_DN;
			}
		}

	// Next, compare the target position to the current position and switch the valve as needed
	if (pMyRobotState->ToteLift_Target > pMyRobotState->ToteLift_Position)	// need to go up
		{
		pToteLifter->Set(DoubleSolenoid::kForward);
		pMyRobotState->ToteLift_Direction = LIFT_UP;
		}
	else if (pMyRobotState->ToteLift_Target < pMyRobotState->ToteLift_Position)	// need to go down
		/*
		 * Need a safety interlock here.
		 *
		 * The totwheels cannot be pinched when the totelift comes down.
		 * Otherwise the totewheel motors could be damaged by dropping a stack of totes on top of the totewheels.
		 *
		 * So, anytime the ToteLift is directed to go down, the totewheel
		 * arms should be directed to spread.
		 *
		 * We will need to ignore the ToteLift Down directives until the
		 * RobotStateBuffer indicates that the totewheels are sufficiently spread.
		 *
		 */
		{
		if ((pMyRobotState->ToteWheelArm_Left_Position < WHEELARMS_NORMAL) && (pMyRobotState->ToteWheelArm_Right_Position < WHEELARMS_NORMAL))
			{
			pToteLifter->Set(DoubleSolenoid::kReverse);
			pMyRobotState->ToteLift_Direction = LIFT_DN;
			}
		else																	// but don't drop totes on top of tote wheels
			{
			pToteLifter->Set(DoubleSolenoid::kOff);
			pMyRobotState->ToteLift_Direction = LIFT_HOLD;
			}
		}
	else		// need to stop here
		{
		pToteLifter->Set(DoubleSolenoid::kOff);
		pMyRobotState->ToteLift_Direction = LIFT_HOLD;
		}

	if (pMyInput->ToteUPBtn == true) {

				pToteLifter->Set(DoubleSolenoid::kForward);
				pMyRobotState->ToteLift_Direction = LIFT_UP;
	} else if (pMyInput->ToteDNBtn == true) {
					pToteLifter->Set(DoubleSolenoid::kReverse);
					pMyRobotState->ToteLift_Direction = LIFT_DN;

	} else {
				pToteLifter->Set(DoubleSolenoid::kOff);
				pMyRobotState->ToteLift_Direction = LIFT_HOLD;

	}



	}  // End Run
