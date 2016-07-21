#ifndef TOTELIFT_H
#define TOTELIFT_H

#include <Definitions.h>
#include <math.h>
#include "WPILib.h"

class TOTELIFT
{
public:
	TOTELIFT();
	void Init(RobotStateBuffer *pRobotState, INPUT *pInput);
	void Run();

private:
	DoubleSolenoid	*pToteLifter, *pLifterEnable;
	bool			button_pressed;
	int				countdown;

/*
 *
#define LIFT_UP_BTN	5
#define LIFT_DN_BTN 6
 *
 */
};
#endif
