#ifndef COMMS_H
#define COMMS_H
//this is where all the comms to and from the robot happen

#include "WPILib.h"
//#include "SmartDashboard/SmartDashboard.h"
#include "Definitions.h"

class COMMS
{

public:
	COMMS();
	void Init();
	void Get(INPUT* input);
	void Send();

	INPUT input;
	OUTPUT output;

private:
	Joystick *Joy1;
	//Joystick Joy2;
	int commsinitcnt;
	int commsgetcnt;
};

#endif
