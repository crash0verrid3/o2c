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
	void IndexArms();
	void Run();

private:
	Victor *pWheelLeftMotor;
	Victor *pWheelRightMotor;
	DoubleSolenoid	*pWheelLeftArm;
	DoubleSolenoid	*pWheelRightArm;

	int	initcnt;
	int left_cycle_count, right_cycle_count;
	int left_accum, right_accum;
	int left_duty_cycle, right_duty_cycle;
	float left_wheel_power, right_wheel_power;

};

#endif /* TOTEWHEELS_H */
