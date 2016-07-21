/*
 * ToteWheels.h
 *
 *  Created on: Jan 31, 2015
 *      Author: Rod
 */

#ifndef TOTEWHEELS_H
#define TOTEWHEELS_H

#include <Definitions.h>
#include <math.h>
#include "WPILib.h"

//#include <AnalogPotentiometer.h>

class TOTEWHEELS
{
public:
	TOTEWHEELS();
	void Init(RobotStateBuffer *pRobotState, INPUT *pInput);
	void Run();

private:
	Victor *pWheelLeftMotor;
	Victor *pWheelRightMotor;
	DoubleSolenoid	*pWheelLeftArm;
	DoubleSolenoid	*pWheelRightArm;

};

#endif /* TOTEWHEELS_H */
