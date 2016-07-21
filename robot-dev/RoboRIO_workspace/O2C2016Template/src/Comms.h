#ifndef COMMS_H
#define COMMS_H
//this is where all the comms to and from the robot happen

#include <Definitions.h>

class COMMS
{

public:
	COMMS();
	void Init(RobotStateBuffer *pRobotState, RobotStateBuffer *pTargetState, INPUT *pInput);
	void Get();
	void Send();

//	OUTPUT output;

private:
	INPUT input;

	bool OneShot(bool *input, bool button);
	void ReadSmartDashboard();
	void Update(std::string Label, bool CurrentState, bool PrevState);
	void Update(std::string Label, int CurrentState, int PrevState);
	void Update(std::string Label, float CurrentState, float PrevState);
	void Update(std::string Label, float CurrentState, float PrevState, float DisplayFactor);
	void Update(std::string Label, double CurrentState, double PrevState);
	void Update(std::string Label, double CurrentState, double PrevState, float DisplayFactor);
	void Update(std::string Label, SpeedControl CurrentState, SpeedControl PrevState);
	void Update(std::string Label, DriveOrientation CurrentState, DriveOrientation PrevState);
	Joystick *Joy1;
	Joystick *Joy2;
	RobotStateBuffer ReportState, ReportTarget;
	INPUT			 ReportInput;
	int commsgetcnt;
};

#endif
