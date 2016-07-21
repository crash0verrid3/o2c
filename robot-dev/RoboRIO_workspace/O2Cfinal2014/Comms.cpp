#include "Comms.h"
//declare the input and output buffers
//they are made global in Definitions.h

COMMS :: COMMS() : Joy1(1), Joy2(2)
{
	
}

void COMMS :: Init()
{
	SmartDashboard::init();
	//send initial data to dashboard
	SmartDashboard::PutNumber("TwoBallMode", 1);
	//looprate = SmartDashboard::GetNumber("LoopsPerSecond");
}

void COMMS :: Get(INPUT* input)
{
	//get inputs
	input->DriveX = Joy1.GetRawAxis(AXIS_DRIVEX) / -1;
	input->DriveY = Joy1.GetRawAxis(AXIS_DRIVEY) / 1;
	input->DriveR = Joy1.GetRawAxis(AXIS_DRIVER) / -1;
	input->ActiveRotation = Joy1.GetRawButton(BTN_ACTIVEROT);
	input->ActiveOff = Joy1.GetRawButton(BTN_ACTIVEOFF);
	input->FieldMode = Joy1.GetRawButton(BTN_DRIVEMODE);
	input->Sprint = Joy1.GetRawButton(BTN_DRIVESPRINT);
	
	input->shoot = Joy1.GetRawButton(BTN_CATSHOOT);
	input->reset = Joy1.GetRawButton(BTN_CATRESET);
	input->catstateon = Joy1.GetRawButton(BTN_CATSTATEON);
	input->catstateoff = Joy1.GetRawButton(BTN_CATSTATEOFF);
//	input->catkill = Joy1.GetRawButton(BTN_CATKILL);
	input->catmanfire = Joy1.GetRawButton(BTN_CATMANFIRE);
	input->catmanpower = 0.9; //Joy1.GetRawAxis(AXIS_CATMANPOWER);
	
	input->gotostart = Joy1.GetRawButton(BTN_CLAWSTART);
	input->gotoscore = Joy1.GetRawButton(BTN_CLAWSCORE);
	input->gotodown = Joy1.GetRawButton(BTN_CLAWDOWN);
	input->gotocatapult = Joy1.GetRawButton(BTN_CLAWCATAPULT);
	input->clawmandown = Joy1.GetRawButton(BTN_CLAWMANDOWN);
	input->clawmanup = Joy1.GetRawButton(BTN_CLAWMANUP);
	
	SmartDashboard::PutBoolean("GoToStart", input->gotostart);
	SmartDashboard::PutBoolean("GoToScore", input->gotoscore);
	SmartDashboard::PutBoolean("GoToDown", input->gotodown);
	SmartDashboard::PutBoolean("GoToCatapult", input->gotocatapult);
	SmartDashboard::PutNumber("DriveX", input->DriveX);
	SmartDashboard::PutNumber("DriveY", input->DriveY);
	SmartDashboard::PutNumber("DriveR", input->DriveR);
	
}

void COMMS :: Send()
{
	//send outputs
}
