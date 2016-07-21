#ifndef QUADAPULT_H
#define QUADAPULT_H

#include "Definitions.h"
#include "WPILib.h"

class QUADAPULT
{
public:
	QUADAPULT();
	void Run(INPUT* input);
	void Init();
	Counter Tension_Detect;

private:
	Timer ShootTimer;
	Victor Tensioner;
	Victor Launcher;

	int CatapultState;
	int catinitcnt;
	double tension_assist_power;
	double tension_assist_time;
	double tension_power;
	double fire_power;
	int tension_count;
	int prev_tension_count;
	int prev2_tension_count;

};
#endif
