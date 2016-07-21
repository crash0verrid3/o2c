#ifndef O2CClaw_H
#define O2CClaw_H
// Our claw class. This is responsible for running our claw
#include "Definitions.h"
#include <math.h>
#include "WPILib.h"

//#include <AnalogPotentiometer.h>

class O2CClaw
{
public:
	O2CClaw();
	void Init();
	void Run(INPUT* input);
	double GetClawEncoder();
	void Test(double power);

private:
	int PlusMin(double value);
	double forkencoder, forkencoder1, forkencoder2;
	double clwpwr, clwpwrlim, breakpwr;
	Victor ClawMotor;//The victor's name is ClawMotor.
	Encoder ClawEncoder;//The encoder's name is ClawEncoder.
	double TargetAngle;// the target angle is a double (it's not really the angle, it's the encoder value)
	int ClawMode;
	int State;//state is an integer
	double difference;//difference is a double
	double direction;//direction is a double
	int clawinitcnt;
};

#endif //End O2CSHOOTER_H
