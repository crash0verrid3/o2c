#include "Comms.h"
//declare the input and output buffers
//they are made global in Definitions.h

COMMS :: COMMS()
{
	commsinitcnt = 0;
	Joy1 = new Joystick(0);
	commsgetcnt = 0;
}

void COMMS :: Init()
{
	commsinitcnt++;
	SmartDashboard::init();
	Joy1->SetAxisChannel(Joystick::kXAxis,0);
	Joy1->SetAxisChannel(Joystick::kYAxis, 1);
	Joy1->SetAxisChannel(Joystick::kTwistAxis, 2);
	//int x = Joy1->GetAxisChannel(Joystick::kXAxis);
	//SmartDashboard::PutNumber("XAxisChannel",x);
	//int y = Joy1->GetAxisChannel(Joystick::kYAxis);
	//SmartDashboard::PutNumber("YAxisChannel",y);
	//int z = Joy1->GetAxisChannel(Joystick::kZAxis);
	//SmartDashboard::PutNumber("ZAxisChannel",z);
	//int r = Joy1->GetAxisChannel(Joystick::kTwistAxis);
	//SmartDashboard::PutNumber("TwistAxisChannel",r);
	//int b = Joy1->GetButtonCount();
	//SmartDashboard::PutNumber("JSbuttonCnt",b);

	//Joy2 = new Joystick(2);
	//send initial data to dashboard
	//SmartDashboard::PutNumber("TwoBallMode", 0);
	//looprate = SmartDashboard::GetNumber("LoopsPerSecond");
	SmartDashboard::PutNumber("CommsInit", commsinitcnt);
	SmartDashboard::PutNumber("MstrInit", 1);
	SmartDashboard::PutNumber("MstrTestInit", 0);
	SmartDashboard::PutNumber("MstrTestCnt", 0);
	SmartDashboard::PutNumber("RoboInit", 0);
	SmartDashboard::PutNumber("DriveInit", 0);
	SmartDashboard::PutNumber("CatapultInit", 0);
	SmartDashboard::PutNumber("ClawInit", 0);
	SmartDashboard::PutNumber("GyroScopeInit", 0);
	SmartDashboard::PutNumber("AutonomousRuns", 0);
	SmartDashboard::PutNumber("TeleopRuns", 0);
	SmartDashboard::PutNumber("TestRuns", 0);
	SmartDashboard::PutNumber("EnabledRuns", 0);
	SmartDashboard::PutNumber("DisabledRuns", 0);
	SmartDashboard::PutNumber("TotalLoops", 0);
	SmartDashboard::PutNumber("RelativeL",0);
	SmartDashboard::PutNumber("MotorL",0);
	SmartDashboard::PutNumber("RelativeR",0);
	SmartDashboard::PutNumber("MotorR",0);
	SmartDashboard::PutNumber("RelativeB",0);
	SmartDashboard::PutNumber("MotorB",0);
	SmartDashboard::PutNumber("GyroLoopTime", 0);
	commsinitcnt++;
	SmartDashboard::PutNumber("CommsInit", commsinitcnt);
}

void COMMS :: Get(INPUT* input)
{
	//get inputs
	//input->DriveX = Joy1.GetRawAxis(AXIS_DRIVEX) / -1;
	//input->DriveY = Joy1.GetRawAxis(AXIS_DRIVEY) / 1;
	//input->DriveR = Joy1.GetRawAxis(AXIS_DRIVER) / 1;

	SmartDashboard::PutNumber("CommsGetCount", commsgetcnt++);

	input->DriveX = Joy1->GetX();
	input->DriveY = Joy1->GetY() * -1;
	input->DriveR = Joy1->GetTwist();

	input->ActiveRotation = Joy1->GetRawButton(BTN_ACTIVEROT);
	input->ActiveOff = Joy1->GetRawButton(BTN_ACTIVEOFF);
	input->FieldMode = Joy1->GetRawButton(BTN_DRIVEMODE);
	input->Sprint = Joy1->GetRawButton(BTN_DRIVESPRINT);

	input->fire_catapult = Joy1->GetRawButton(BTN_CATSHOOT);
	input->reset = Joy1->GetRawButton(BTN_CATRESET);
	input->catstateon = Joy1->GetRawButton(BTN_CATSTATEON);
	input->catstateoff = Joy1->GetRawButton(BTN_CATSTATEOFF);
//	input->catkill = Joy1->GetRawButton(BTN_CATKILL);
	input->catmanual = (Joy1->GetRawButton(BTN_CATMANUAL_OPERATION) || input->catmanual) \
			&& !(Joy1->GetRawButton(BTN_CATMANUAL_OPERATION) && input->catmanual);	// exclusive OR to toggle

	input->gotostart = Joy1->GetRawButton(BTN_CLAWSTART);
	input->gotoscore = Joy1->GetRawButton(BTN_CLAWSCORE);
	input->gotodown = Joy1->GetRawButton(BTN_CLAWDOWN);
	input->gotocatapult = Joy1->GetRawButton(BTN_CLAWCATAPULT);
	input->clawmandown = Joy1->GetRawButton(BTN_CLAWMANDOWN);
	input->clawmanup = Joy1->GetRawButton(BTN_CLAWMANUP);
	input->clawmanpower = Joy1->GetRawAxis(AXIS_CLAWMANPOWER);

	SmartDashboard::PutBoolean("GoToStart", input->gotostart);
	SmartDashboard::PutBoolean("GoToScore", input->gotoscore);
	SmartDashboard::PutBoolean("GoToDown", input->gotodown);
	SmartDashboard::PutBoolean("GoToCatapult", input->gotocatapult);
	SmartDashboard::PutNumber("DriveX", input->DriveX);
	SmartDashboard::PutNumber("DriveY", input->DriveY);
	SmartDashboard::PutNumber("DriveR", input->DriveR);
	//SmartDashboard::PutNumber("ClawPower", input->clawmanpower);

}

void COMMS :: Send()
{
	//send outputs
}
