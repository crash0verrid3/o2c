#ifndef O2CCATAPULT_H
#define O2CCATAPULT_H
//This is for the catapult. It contains the catapult state machine and dictates how it should fire.
#include "Definitions.h"

#include "WPILib.h"

class O2CCATAPULT
{
public:
	O2CCATAPULT();
	
	void Run(INPUT* input);
	void Init();
	DigitalInput ShooterResetSwitch;  //Reset Switch
	
private:
	Timer ShootTimer;
	Encoder Sencoder; //Shooter encoder.
	Victor LaMotor;	//Launch motor.
	
	int CatapultState; //catapult state
	int runsStart;
	//int TimesShot;
	//double tempencoder;
};

#endif //Ends O2CCATAPULT_H
	
