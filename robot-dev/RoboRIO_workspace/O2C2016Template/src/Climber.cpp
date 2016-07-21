/*
 * Climber.cpp
 *
 *  Created on: Mar 28, 2016
 *      Author: Rod
 */

#include <Climber.h>


CLIMBER :: CLIMBER()
	{
	pLeftWinch = new Talon(WINCHLEFTMOTORPWMPORT);
	pRightWinch = new Talon(WINCHRIGHTMOTORPWMPORT);
	pWinchLock = new Servo(WINCHLOCKPWMPORT);
	}

void CLIMBER :: Init(RobotStateBuffer *pRobotState, RobotStateBuffer *pTargetState, INPUT *pInput)
	{
	pMyRobotState = pRobotState;
	pMyTargetState = pTargetState;
	pMyInput = pInput;
	pMyRobotState->Winch_Init++;
	StopMotors();  // Make sure all motors are stopped.
	pMyTargetState->Winch_Position = pMyRobotState->Winch_Position = WINCHRETRACTED;
	pMyTargetState->Winch_Lock_Posn = WINCHSERVOLOCKPOSN;
	pMyTargetState->Winch_Power = 0;
	pMyRobotState->Winch_Init++;
	}


void CLIMBER :: RunWinch()
	{
	if (pMyInput->WinchUpBtn)
		{
		pMyTargetState->Winch_Position += WINCHPERCYCLE;
		pMyTargetState->Winch_Lock_Posn = WINCHSERVORELEASEPOSN;
		}
	else if (pMyInput->WinchDnBtn)
		{
		pMyTargetState->Winch_Position -= WINCHPERCYCLE;
		pMyTargetState->Winch_Lock_Posn = WINCHSERVOLOCKPOSN;
		}
	else
		{
		pMyTargetState->Winch_Lock_Posn = WINCHSERVOLOCKPOSN;
		}


	if (pMyRobotState->Winch_Position > pMyTargetState->Winch_Position)
		{
		pLeftWinch->Set(WINCHLEFTRETRACTPOWER);
		pRightWinch->Set(WINCHRIGHTRETRACTPOWER);
		pMyRobotState->Winch_Power = WINCHLEFTRETRACTPOWER;
		pMyRobotState->Winch_Position -= WINCHPERCYCLE;
		}
	else if ((pMyRobotState->Winch_Position < pMyTargetState->Winch_Position) && (pMyRobotState->Winch_Lock_Posn <= WINCHSERVORELEASEPOSN))
		{
		pLeftWinch->Set(WINCHLEFTEXTENDPOWER);
		pRightWinch->Set(WINCHRIGHTEXTENDPOWER);
		pMyRobotState->Winch_Power = WINCHLEFTEXTENDPOWER;
		pMyRobotState->Winch_Position += WINCHPERCYCLE;
		}
	else
		{
		pLeftWinch->Set(0);
		pRightWinch->Set(0);
		pMyRobotState->Winch_Power = 0;
		}

	if (pMyRobotState->Winch_Lock_Posn < pMyTargetState->Winch_Lock_Posn)
		{
		pMyRobotState->Winch_Lock_Posn += WINCHSERVOSPEED;
		pWinchLock->Set(pMyTargetState->Winch_Lock_Posn / WINCHSERVORANGE);
		}
	else if (pMyRobotState->Winch_Lock_Posn > pMyTargetState->Winch_Lock_Posn)
		{
		pMyRobotState->Winch_Lock_Posn -= WINCHSERVOSPEED;
		pWinchLock->Set(pMyTargetState->Winch_Lock_Posn / WINCHSERVORANGE);
		}
	}


void CLIMBER :: StopMotors()
	{
	//lock the winch
	pWinchLock->Set(WINCHSERVOLOCKPOSN);
	//stop the winch motors
	pLeftWinch->Set(0);
	pRightWinch->Set(0);
	pMyRobotState->Winch_Power = 0;
	}

