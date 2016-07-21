#include "Robot.h"

ROBOT :: ROBOT()
	{
	//init variables
	robotinitcnt = 0;
	hasInited = false;
	}


void ROBOT :: Init()
	{
	//******************************************************************************
	//NOTE: DO NOT ATTEPT to use the SmartDashboard until AFTER COMMS has initialized.
	//******************************************************************************
	++robotinitcnt;
	Comms.Init();  // After this you can use SmartDashboard.
	Drivetrain.Init();
	SmartDashboard::PutNumber("RobotInit", ++robotinitcnt);
	}

//AUTO
void ROBOT :: AutoInit(INPUT* input)
	{
	//auto init goes here

	//Make sure we are stopped.
		input->DriveX = 0;
		input->DriveY = 0;
		input->DriveR = 0;
	}

void ROBOT :: Autonomous(INPUT* input)
	{
	//Autonomous Goes Here
	//Call the Drivetrain to keep motor's refreshed to speed state.  AutoInit value was Stopped @ 0.
	Drivetrain.MecanumDrive(input);	// execute driving instructions
	}

//TELEOP
void ROBOT :: TeleInit(INPUT* input)
	{
		//teleop init goes here
	}

void ROBOT :: Teleop(INPUT* input)
	{
		//Teleop
		Comms.Get(input);				// get input data
		Drivetrain.MecanumDrive(input);	// execute driving instructions
	}

//TEST
void ROBOT :: TestInit(INPUT* input)
	{
		//test init goes here
		//Make sure we are stopped.
		input->DriveX = 0;
		input->DriveY = 0;
		input->DriveR = 0;
	}

void ROBOT :: Test(INPUT* input)
	{
		//test goes here
		//Call the Drivetrain to keep motor's refreshed to speed state.  TestInit value was Stopped @ 0.
		Drivetrain.MecanumDrive(input);	// execute driving instructions
	}

//DISABLED
void ROBOT :: DisabledInit(INPUT* input)
	{
		//disabled init goes here

	}

void ROBOT :: Disabled(INPUT* input)
	{
		//disabled goes here

	}

void ROBOT :: SendData()
	{
	//send data to dashboard.  Currently Unused and for Future Plans.
	}
