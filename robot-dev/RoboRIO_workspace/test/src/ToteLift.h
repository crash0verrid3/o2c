#ifndef TOTELIFT_H
#define TOTELIFT_H

#include <Definitions.h>
#include "WPILib.h"

class TOTELIFT
{
public:
	TOTELIFT();
	void Run();
	void Init(RobotStateBuffer *pRobotState, INPUT *pInput);

private:
	Victor *pToteLiftMotor;
	Encoder *pLiftEncoder;

	void InitToteLiftState();

	int Lift_Position, Init_Lift_Position, Init_Verify_Count;
	double LiftPower;
	int initcnt;
	bool ToteLiftStateInited;
	int max_init_cycles;

/*
 *
#define LIFT_UP_BTN	5
#define LIFT_DN_BTN 6
#define LIFT_DIO_PORT1	4
#define LIFT_DIO_PORT2	5
#define LIFT_UP_POWER 15
#define LIFT_DN_POWER -5
 *
 */
};
#endif
