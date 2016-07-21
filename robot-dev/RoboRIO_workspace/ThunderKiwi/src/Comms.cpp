#include "Comms.h"
//declare the input and output buffers
//they are made global in Definitions.h

COMMS :: COMMS()
{
	commsinitcnt = 0;
	Joy1 = new Joystick(0);
	commsgetcnt = 0;
}

void COMMS :: Init(RobotStateBuffer *pRobotState)
{
	commsinitcnt++;
	SmartDashboard::init();
	pMyRobotState = pRobotState;
	Wait(.5);  // Give the Dashboard Time to settle down before any Gets (or Puts).

	SmartDashboard::PutNumber("CommsInit", ++commsinitcnt);
	SmartDashboard::PutNumber("CommsGetCount", 0);
	SmartDashboard::PutNumber("RobotInit", 0);
	SmartDashboard::PutNumber("DriveInit", 0);
	SmartDashboard::PutNumber("AutonomousRuns", 0);
	SmartDashboard::PutNumber("TeleopRuns", 0);
	SmartDashboard::PutNumber("TestRuns", 0);
	SmartDashboard::PutNumber("EnabledRuns", 0);
	SmartDashboard::PutNumber("DisabledRuns", 0);
	SmartDashboard::PutNumber("TotalLoops", 0);

	// Setup the JoySticks.
	Joy1->SetAxisChannel(Joystick::kXAxis,0);
	Joy1->SetAxisChannel(Joystick::kYAxis, 1);
	Joy1->SetAxisChannel(Joystick::kTwistAxis, 2);

	//Last Step in Init to update CommsInit
	SmartDashboard::PutNumber("CommsInit", ++commsinitcnt);
}

void COMMS :: Get(INPUT* input)
{
	//get inputs

	SmartDashboard::PutNumber("CommsGetCount", commsgetcnt++);

	//DriveTrain Inputs
	input->DriveX = Joy1->GetX();
	input->DriveY = Joy1->GetY();
	input->DriveR = Joy1->GetTwist();
	input->OrientModeBtn = Joy1->GetRawButton(BTN_DRIVEMODE);
	input->SprintModeBtn = Joy1->GetRawButton(BTN_DRIVESPRINT);
	//Other Inputs to Follow

}

void COMMS :: Send()
{
	//send outputs
}
