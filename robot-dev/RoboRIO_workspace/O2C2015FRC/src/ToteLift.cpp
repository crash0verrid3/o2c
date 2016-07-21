#include <ToteLift.h>

TOTELIFT :: TOTELIFT()
{
	initcnt = 0;
	pToteLiftMotor = new Victor(LIFTVICTORCHANNEL);
	pLiftEncoder = new Encoder(LIFT_DIO_PORT1, LIFT_DIO_PORT2, LIFT_REVERSE_DIRECTION, Encoder::k4X);
	pLiftEncoder->SetReverseDirection(LIFT_REVERSE_DIRECTION);
	Lift_Position = 0;
	Init_Lift_Position = 0;
	LiftPower = 0;
	Init_Verify_Count = 0;
	ToteLiftStateInited = false;
	max_init_cycles = LIFT_MAX_INIT_CYCLES;
}

void TOTELIFT :: Init(RobotStateBuffer *pRobotState, INPUT *pInput)
{
	SmartDashboard::PutNumber("ToteLiftInit", ++initcnt);
	pMyRobotState = pRobotState;
	pMyInput = pInput;
	pMyRobotState->ToteLift_Direction = LIFT_HOLD;
	pMyRobotState->ToteLift_JoyMode = LIFT_JOY_OFF;
	pMyRobotState->ToteLift_Target = 0;
	pMyRobotState->ToteLift_Position = Init_Lift_Position = pLiftEncoder->Get() + LIFT_HOLD_POWER_WINDOW;
	SmartDashboard::PutNumber("ToteLiftStateInited", ToteLiftStateInited);
	SmartDashboard::PutNumber("ToteLiftInit", ++initcnt);
}

void TOTELIFT :: InitToteLiftState()	  // First Init with Arms Down.
	{
	Lift_Position = pLiftEncoder->Get();
	if (max_init_cycles-- <= 0)
		{												// Done trying to init, go with what we got
		pToteLiftMotor->Set(0);							// Stop the Lift Power
		pLiftEncoder->Reset();							// This seems to be as far down as it goes
		ToteLiftStateInited = true;
		pMyRobotState->ToteLift_Position = Lift_Position;
		pMyRobotState->ToteLift_Target = LIFT_LIMIT_DN;
		pMyRobotState->ToteLift_Direction = LIFT_HOLD;
		}
	else if (Lift_Position < Init_Lift_Position)		// Lift is still moving down
		{
		pToteLiftMotor->Set(LIFT_DN_POWER/100);  				// Move the Lift Down.
		Init_Verify_Count = 0;
		}
	else if (Init_Verify_Count++ > 5)					// Lift has hit bottom for 6 cycles
		{
		pToteLiftMotor->Set(0);							// Stop the Lift Power
		pLiftEncoder->Reset();							// We are at the bottom
		ToteLiftStateInited = true;
		pMyRobotState->ToteLift_Position = Lift_Position;
		pMyRobotState->ToteLift_Target = LIFT_LIMIT_DN;
		pMyRobotState->ToteLift_Direction = LIFT_HOLD;
		SmartDashboard::PutNumber("ToteLiftStateInited", ToteLiftStateInited);
		}
	Init_Lift_Position = Lift_Position;
#ifdef DEBUG_LIFT_INIT
	SmartDashboard::PutNumber("ToteLiftPosition", Lift_Position);
	SmartDashboard::PutNumber("ToteLiftCycles", max_init_cycles);
#endif
	}


void TOTELIFT :: Run()
	{

	/*
	 * TODO:
	 *
	 * Need a safety interlock here.
	 *
	 * The totwheels cannot be pinched when the totelift arms come down.
	 * Otherwise the totewheel motors could be damaged by the totelift arms.
	 *
	 * So, anytime the ToteLift is directed to go down, the totewheel
	 * arms should be directed to spread.
	 *
	 * Trouble is:  The totewheel arms operate on a duty cycle, so
	 * it might take a number of cycles to pull the totewheels back,
	 * while the totelift arms only require one cycle to move.
	 *
	 * We will need to ignore the ToteLift Down directives until the
	 * RobotStateBuffer indicates that the totewheels are sufficiently spread.
	 *
	 */
	if (ToteLiftStateInited == false)
		{
		InitToteLiftState(); // First Init with Arms Down.
		}
	/*
	 * TODO:
	 * It is possible that the ToteLift subsystem did not completely reset to the bottom.
	 * It might be good to reset the encoder anytime it reads less than 0.
	 */
	else
		{
		if (pMyInput->ToteJoyBtn == true)   //One Way to Joy Control.  No going back.
			{
			pMyRobotState->ToteLift_JoyMode = LIFT_JOY_ON;
			}
		SmartDashboard::PutNumber("ToteLiftJoyMode", pMyRobotState->ToteLift_JoyMode);

		if (pMyRobotState->ToteLift_JoyMode == LIFT_JOY_ON)
			{
			// Need to have a safety timeout.  No more than .2 seconds against an upper or lower limit.

			if (pMyInput->ToteLift > 0) LiftPower = LIFT_DN_POWER * pMyInput->ToteLift; // Joy>0 is DN.   Positive to Motor Goes DN.
			else LiftPower = -1 * LIFT_UP_POWER * pMyInput->ToteLift;  // Joy<0 is UP.  Negative to Motor Goes UP.  The extra -1 removes the negative the define of the up power that is not needed because the joystick goes negative.

			if (fabs(LiftPower) < 3) LiftPower = 0;  // Make sure we are at zero if we are close to zero.
			else pMyRobotState->ToteLift_Target = Lift_Position = pMyRobotState->ToteLift_Position = pLiftEncoder->Get();  //Adjust the hold target current.


			// Hold Code:
			// If the joystick is off (no power indicated from above code), hold current position
			if (LiftPower == 0)
				{
				Lift_Position = pMyRobotState->ToteLift_Position = pLiftEncoder->Get();
				// If we are above the target in hold window, cut power.
				if (Lift_Position - pMyRobotState->ToteLift_Target > LIFT_HOLD_WINDOW)
					{
					LiftPower = LIFT_NO_POWER;
					}
				// We are below the hold window. Adjust Power
				else if (pMyRobotState->ToteLift_Target - Lift_Position > LIFT_HOLD_WINDOW)
					{
					double powerDelta = LIFT_UP_POWER - LIFT_MINHOLD_POWER;
					int targetDelta =   pMyRobotState->ToteLift_Target - Lift_Position;
					if (targetDelta > LIFT_HOLD_POWER_WINDOW )  //We are below the hold power range.  Max Lift and no more.
						{
						LiftPower = LIFT_UP_POWER;
						}
					else  // We are in the hold window.  Scale the power.
						{
						LiftPower = LIFT_MINHOLD_POWER + ( (double) targetDelta / (double) LIFT_HOLD_POWER_WINDOW * powerDelta);
						}
					}
				else // We are in the Hold Window.  Set Minimum Hold Power.
					{
					LiftPower = LIFT_MINHOLD_POWER;
					}
				}
			pToteLiftMotor->Set(LiftPower/100);  // Set the power / 100.
			SmartDashboard::PutNumber("ToteLiftPower", LiftPower/100);
			pMyRobotState->ToteLift_Position = Lift_Position = pLiftEncoder->Get();
			SmartDashboard::PutNumber("ToteLiftTicks", Lift_Position);
			}

		else // State Machine Mode.
			{
			Lift_Position = pMyRobotState->ToteLift_Position = pLiftEncoder->Get();
			// Deal with the Up and Down Buttons
			if (pMyInput->ToteUPBtn == true && pMyRobotState->ToteLift_Direction == LIFT_HOLD)
				{
				pMyRobotState->ToteLift_Direction = LIFT_UP;
				switch (pMyRobotState->ToteLift_Target)
					{
					case LIFT_LIMIT_DN:
						 pMyRobotState->ToteLift_Target = LIFT_STEP_UP;
						break;

					case LIFT_STEP_UP:
						pMyRobotState->ToteLift_Target = LIFT_TOTE_UP;
						break;

					case LIFT_TOTE_UP:
						pMyRobotState->ToteLift_Target = LIFT_TOTESTEP_UP;
						break;

					case LIFT_TOTESTEP_UP:
						pMyRobotState->ToteLift_Target = LIFT_LIMIT_UP;
						break;

					case LIFT_LIMIT_UP:
						pMyRobotState->ToteLift_Target = LIFT_LIMIT_UP;
						pMyRobotState->ToteLift_Direction = LIFT_HOLD; //Stay Holding at limit
						break;
					}
				}
			else if (pMyInput->ToteDNBtn == true && pMyRobotState->ToteLift_Direction == LIFT_HOLD)
				{
				pMyRobotState->ToteLift_Direction = LIFT_DN;
				switch (pMyRobotState->ToteLift_Target)
					{
					case LIFT_LIMIT_DN:
						pMyRobotState->ToteLift_Target = LIFT_LIMIT_DN;
						pMyRobotState->ToteLift_Direction = LIFT_HOLD;  //Stay Holding at limit
						break;

					case LIFT_STEP_UP:
						pMyRobotState->ToteLift_Target = LIFT_LIMIT_DN;
						break;

					case LIFT_TOTE_UP:
						pMyRobotState->ToteLift_Target = LIFT_STEP_UP;
						break;

					case LIFT_TOTESTEP_UP:
						pMyRobotState->ToteLift_Target = LIFT_TOTE_UP;
						break;

					case LIFT_LIMIT_UP:
						pMyRobotState->ToteLift_Target = LIFT_TOTESTEP_UP;
						break;
					}
				}

			// Check to see if we should be in hold status
			if (abs(Lift_Position - pMyRobotState->ToteLift_Target) < LIFT_HOLD_WINDOW)
				{
				pMyRobotState->ToteLift_Direction = LIFT_HOLD;
				}

			// Adjust the power based on
			switch (pMyRobotState->ToteLift_Direction)
				{
				case LIFT_UP:
					if (pMyRobotState->ToteLift_Target - Lift_Position  < LIFT_CLOSE)
						{
						LiftPower = 0.75 * LIFT_UP_POWER;
						}
					else LiftPower = LIFT_UP_POWER;
					if (pMyRobotState->ToteLift_Target - Lift_Position  < LIFT_HOLD_WINDOW)
						{
						pMyRobotState->ToteLift_Direction = LIFT_HOLD;
						}
					break;
				case LIFT_DN:
					if (Lift_Position - pMyRobotState->ToteLift_Target < LIFT_CLOSE)
						{
						LiftPower = 0.75 * LIFT_DN_POWER;
						}
					else LiftPower = LIFT_DN_POWER;
					if (Lift_Position - pMyRobotState->ToteLift_Target  < LIFT_HOLD_WINDOW)
						{
						pMyRobotState->ToteLift_Direction = LIFT_HOLD;
						}
					break;
				case LIFT_HOLD:
					//If we are on either side of the hold window or below AND at the BOTTOM, cut power.
					if (pMyRobotState->ToteLift_Target == LIFT_LIMIT_DN  && Lift_Position - pMyRobotState->ToteLift_Target < LIFT_HOLD_WINDOW )
						{
						LiftPower = LIFT_NO_POWER;
						}
					// If we are above the target in hold window, cut power.
					else if (Lift_Position - pMyRobotState->ToteLift_Target > LIFT_HOLD_WINDOW)
						{
						LiftPower = LIFT_NO_POWER;
						}
					// We are below the hold window. Adjust Power
					else if (pMyRobotState->ToteLift_Target - Lift_Position > LIFT_HOLD_WINDOW)
						{
						double powerDelta = LIFT_UP_POWER - LIFT_MINHOLD_POWER;
						int targetDelta =   pMyRobotState->ToteLift_Target - Lift_Position;
						if (targetDelta > LIFT_HOLD_POWER_WINDOW )  //We are below the hold power range.  Max Lift and no more.
							{
							LiftPower = LIFT_UP_POWER;
							}
						else  // We are in the hold window.  Scale the power.
							{
							LiftPower = LIFT_MINHOLD_POWER + ( (double) targetDelta / (double) LIFT_HOLD_POWER_WINDOW * powerDelta);
							}
						}
					else // We are in the Hold Window.  Set Minimum Hold Power.
						{
						LiftPower = LIFT_MINHOLD_POWER;
						}
					break;
				}  // End Swtich
			pToteLiftMotor->Set(LiftPower/100);  //  Set Lift Power / 100
			}  // End Else Joy Mode (State Machine Mode)
		}
	SmartDashboard::PutNumber("ToteLiftPower", LiftPower/100);
	SmartDashboard::PutNumber("ToteLiftPosition", Lift_Position);
	SmartDashboard::PutNumber("ToteLiftTarget", pMyRobotState->ToteLift_Target);
	SmartDashboard::PutNumber("ToteLiftDirection", pMyRobotState->ToteLift_Direction);
	}  // End Run
