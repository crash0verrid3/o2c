#ifndef MASTER_H
#define MASTER_H
//high level control
//gets input from joysticks/dashboard
//then tells the robot to do stuff

//include defs, robot, and driverstation
#include "Definitions.h"
#include "Robot.h"
#include "Comms.h"
#include "Timer.h"


class MASTER
{

public:
	
	MASTER(); //constructor
	
	//Init, Autonomous, Teleop, Test, Disabled
	void Init();
	void AutoInit(INPUT* input);
	void Autonomous(INPUT* input);
	void TeleInit();
	void Teleop(INPUT* input);
	void TestInit();
	void Test(INPUT* input);
	void DisabledInit();
	void Disabled();
	void SendData();
	
private:
	ROBOT Robot;
	COMMS Comms;
	Timer AutoTimer;
	bool hasInited;
	int AutoState;
	int TwoBallMode;
	double TempTimerVal;
	//has a robot and driverstation
	//also, define vars here
};

#endif
