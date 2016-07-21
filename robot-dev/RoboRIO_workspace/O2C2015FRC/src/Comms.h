#ifndef COMMS_H
#define COMMS_H
//this is where all the comms to and from the robot happen

#include <Definitions.h>
#include "WPILib.h"

class COMMS
{

public:
	COMMS();
	void Init(RobotStateBuffer *pRobotState, INPUT *pInput);
	void Get();
	void Send();

	INPUT input;
	OUTPUT output;

private:
	bool			 Update(std::string Label, bool CurrentState, bool PrevState);
	int				 Update(std::string Label, int CurrentState, int PrevState);
	float			 Update(std::string Label, float CurrentState, float PrevState);
	double			 Update(std::string Label, double CurrentState, double PrevState);
	SpeedControl	 Update(std::string Label, SpeedControl CurrentState, SpeedControl PrevState);
	DriveOrientation Update(std::string Label, DriveOrientation CurrentState, DriveOrientation PrevState);
	Joystick *Joy1;
	Joystick *Joy2;
	RobotStateBuffer ReportState;
	int initcnt;
	int commsgetcnt;
};

#endif
