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
	DigitalInput ShooterResetSwitch;
	
private:
	Timer ShootTimer;
	Victor Loader;
	Victor Launcher;
	
	int CatapultState;
	
};
#endif
