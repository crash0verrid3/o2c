#ifndef Robot_H
#define Robot_H
//mid-level actions

#include "Definitions.h"
#include "Drivetrain.h"
#include "O2CCATAPULT.h"
#include "O2CClaw.h"
#include "Quadapult.h"
#include <math.h>
//todo: include drivetrain, forklift, shooter

class ROBOT
{

public:
	ROBOT();
	void Init();
	void Drive(INPUT* input);
	void Shooter(INPUT* input);
	void Claw(INPUT* input);
	void Aton(INPUT* input);
	void GyroCore(INPUT* input);
	DRIVETRAIN Drivetrain;
	QUADAPULT Catapult;
	DigitalInput PhotoSensor;
	AnalogChannel UltraClaw;
	
private:
	//todo: has a shooter
	double PrevAngle;
	O2CClaw Forklift;
};

#endif
