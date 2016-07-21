/*
 * Climber.h
 *
 *  Created on: Mar 28, 2016
 *      Author: Rod
 */

#ifndef CLIMBER_H_
#define CLIMBER_H_

#include "Definitions.h"

class CLIMBER
{
public:
	CLIMBER();

	void Init(RobotStateBuffer *pRobotState, RobotStateBuffer *pTargetState, INPUT *pInput);
	void RunWinch();
	void StopMotors();

private:

	Talon 		*pLeftWinch, *pRightWinch;
	Servo		*pWinchLock;

};


#endif /* CLIMBER_H_ */
