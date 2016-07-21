#include "Comms.h"
//declare the input and output buffers
//they are made global in Definitions.h

COMMS :: COMMS()
{
	initcnt = 0;
	Joy1 = new Joystick(0);
	Joy2 = new Joystick(1);
	commsgetcnt = 0;
}

void COMMS :: Init(RobotStateBuffer *pRobotState, INPUT *pInput)
{
	SmartDashboard::init();
	Wait (1);
	SmartDashboard::PutNumber("CommsInit", ++initcnt);
	pMyRobotState = pRobotState;
	pMyInput = pInput;
	pMyInput->SprintModeBtn = false;
	/*
	 * These are the default values.  No need to change them
	Joy1->SetAxisChannel(Joystick::kXAxis, AXIS_DRIVEX);
	Joy1->SetAxisChannel(Joystick::kYAxis, AXIS_DRIVEY);
	Joy1->SetAxisChannel(Joystick::kTwistAxis, AXIS_DRIVER);
	Joy1->SetAxisChannel(Joystick::kThrottleAxis, AXIS_LIFTER);
	 */
	SmartDashboard::PutNumber("AutonomousRuns", 0);
	SmartDashboard::PutNumber("TeleopRuns", 0);
	SmartDashboard::PutNumber("TestRuns", 0);
	SmartDashboard::PutNumber("EnabledRuns", 0);
	SmartDashboard::PutNumber("DisabledRuns", 0);
	SmartDashboard::PutNumber("TotalLoops", 0);
	SmartDashboard::PutNumber("CommsInit", ++initcnt);
	pMyRobotState->Comms_Init = initcnt;
	Wait (1);
//	Send();
}


void COMMS :: Get()
{
	//get inputs
	SmartDashboard::PutNumber("CommsGetCount", commsgetcnt++);

	//get joystick movements
	pMyInput->DriveX = Joy1->GetX();
	pMyInput->DriveY = Joy1->GetY() * INVERT_Y_AXIS_INPUT;
	pMyInput->DriveR = Joy1->GetTwist();
	pMyInput->ToteLift = Joy1->GetThrottle();

	//pMyInput->DriveX = Joy2->GetX();
	//pMyInput->PincerUM = Joy2->GetY() * INVERT_Y_AXIS_INPUT;
	//pMyInput->DriveR = Joy2->GetTwist();
	//pMyInput->PincerLM = Joy2->GetThrottle();

	pMyInput->LeftArm = Joy2->GetX();
	pMyInput->LeftWheel = Joy2->GetY();
	pMyInput->RightArm = Joy2->GetTwist();
	pMyInput->RightWheel = Joy2->GetThrottle();

	//get buttons pressed
	pMyInput->SprintModeBtn = Joy1->GetRawButton(BTN_SPRINTCONTROL);
	pMyInput->SpinControlBtn = Joy1->GetRawButton(BTN_SPINCONTROL);
	pMyInput->OrientModeBtn = Joy1->GetRawButton(BTN_ORIENTMODE);

	pMyInput->ToteUPBtn = Joy1->GetRawButton(LIFT_UP_BTN);
	pMyInput->ToteDNBtn = Joy1->GetRawButton(LIFT_DN_BTN);
	pMyInput->ToteJoyBtn = Joy1->GetRawButton(LIFT_JOY_BTN);

	//print diagnositics
	SmartDashboard::PutNumber("DriveX", pMyInput->DriveX);
	SmartDashboard::PutNumber("DriveY", pMyInput->DriveY);
	SmartDashboard::PutNumber("DriveR", pMyInput->DriveR);
	SmartDashboard::PutNumber("ToteLift", pMyInput->ToteLift);
//	SmartDashboard::PutNumber("PincerUM", pMyInput->PincerUM);
//	SmartDashboard::PutNumber("PincerLM", pMyInput->PincerLM);


	SmartDashboard::PutBoolean("ToteUPBtn",pMyInput->ToteUPBtn);
	SmartDashboard::PutBoolean("ToteDNBtn",pMyInput->ToteDNBtn);
	SmartDashboard::PutBoolean("ToteJoyBtn",pMyInput->ToteJoyBtn);

/*
	SmartDashboard::PutBoolean("SprintModeBtn",pMyInput->SprintModeBtn);
	SmartDashboard::PutBoolean("SpinControlBtn",pMyInput->SpinControlBtn);
	SmartDashboard::PutBoolean("OrientModeBtn",pMyInput->OrientModeBtn);
*/
}

void COMMS :: Send()
	{

	return;			// disabled for now

	//send outputs
	ReportState.Robot_Init = Update("Robot_Init", pMyRobotState->Robot_Init, ReportState.Robot_Init);

	ReportState.Robot_Heading = Update("Robot_Heading", pMyRobotState->Robot_Heading, ReportState.Robot_Heading);
	ReportState.Robot_Direction = Update("Robot_Direction", pMyRobotState->Robot_Direction, ReportState.Robot_Direction);
	ReportState.Robot_Speed = Update("Robot_Speed", pMyRobotState->Robot_Speed, ReportState.Robot_Speed);
	ReportState.Robot_Position_X = Update("Robot_Position_X", pMyRobotState->Robot_Position_X, ReportState.Robot_Position_X);
	ReportState.Robot_Position_Y = Update("Robot_Position_Y", pMyRobotState->Robot_Position_Y, ReportState.Robot_Position_Y);
	ReportState.Comms_Init = Update("Comms_Init", pMyRobotState->Comms_Init, ReportState.Comms_Init);
	ReportState.Battery_Voltage = Update("Battery_Voltage", pMyRobotState->Battery_Voltage, ReportState.Battery_Voltage);
	ReportState.Drivetrain_Init = Update("Drivetrain_Init", pMyRobotState->Drivetrain_Init, ReportState.Drivetrain_Init);
	ReportState.Drivetrain_FL_Motor_Power = Update("Drivetrain_FL_Motor_Power", pMyRobotState->Drivetrain_FL_Motor_Power, ReportState.Drivetrain_FL_Motor_Power);
	ReportState.Drivetrain_FR_Motor_Power = Update("Drivetrain_FR_Motor_Power", pMyRobotState->Drivetrain_FR_Motor_Power, ReportState.Drivetrain_FR_Motor_Power);
	ReportState.Drivetrain_BL_Motor_Power = Update("Drivetrain_BL_Motor_Power", pMyRobotState->Drivetrain_BL_Motor_Power, ReportState.Drivetrain_BL_Motor_Power);
	ReportState.Drivetrain_BR_Motor_Power = Update("Drivetrain_BR_Motor_Power", pMyRobotState->Drivetrain_BR_Motor_Power, ReportState.Drivetrain_BR_Motor_Power);
	ReportState.Drivetrain_Speed_Control = Update("Drivetrain_Speed_Control", pMyRobotState->Drivetrain_Speed_Control, ReportState.Drivetrain_Speed_Control);
	ReportState.Drivetrain_Spin_Control = Update("Drivetrain_Spin_Control", pMyRobotState->Drivetrain_Spin_Control, ReportState.Drivetrain_Spin_Control);
	ReportState.Drivetrain_Heading = Update("Drivetrain_Heading", pMyRobotState->Drivetrain_Heading, ReportState.Drivetrain_Heading);
	ReportState.Drivetrain_Directional_Orientation = Update("Drivetrain_Directional_Orientation", pMyRobotState->Drivetrain_Directional_Orientation, ReportState.Drivetrain_Directional_Orientation);
	ReportState.Navigation_Init = Update("Navigation_Init", pMyRobotState->Navigation_Init, ReportState.Navigation_Init);
	ReportState.Navigation_Robot_Lost = Update("Navigation_Robot_Lost", pMyRobotState->Navigation_Robot_Lost, ReportState.Navigation_Robot_Lost);
	ReportState.Navigation_GyroTilt = Update("Navigation_GyroTilt", pMyRobotState->Navigation_GyroTilt, ReportState.Navigation_GyroTilt);
	ReportState.ToteLift_Init = Update("ToteLift_Init", pMyRobotState->ToteLift_Init, ReportState.ToteLift_Init);
	ReportState.ToteLift_Position = Update("ToteLift_Position", pMyRobotState->ToteLift_Position, ReportState.ToteLift_Position);
	ReportState.ToteLift_Speed = Update("ToteLift_Speed", pMyRobotState->ToteLift_Speed, ReportState.ToteLift_Speed);
	ReportState.ToteLift_Direction = Update("ToteLift_Direction", pMyRobotState->ToteLift_Direction, ReportState.ToteLift_Direction);
	ReportState.ToteLift_ToteCount = Update("ToteLift_ToteCount", pMyRobotState->ToteLift_ToteCount, ReportState.ToteLift_ToteCount);
	ReportState.ToteLift_ToteCount = Update("ToteLift_JoyMode", pMyRobotState->ToteLift_JoyMode, ReportState.ToteLift_JoyMode);
	ReportState.Pincer_Init = Update("Pincer_Init", pMyRobotState->Pincer_Init, ReportState.Pincer_Init);
	ReportState.Pincer_Arm1_Position = Update("Pincer_Arm1_Position", pMyRobotState->Pincer_Arm1_Position, ReportState.Pincer_Arm1_Position);
	ReportState.Pincer_Arm1_Speed = Update("Pincer_Arm1_Speed", pMyRobotState->Pincer_Arm1_Speed, ReportState.Pincer_Arm1_Speed);
	ReportState.Pincer_Arm2_Position = Update("Pincer_Arm2_Position", pMyRobotState->Pincer_Arm2_Position, ReportState.Pincer_Arm2_Position);
	ReportState.Pincer_Arm2_Speed = Update("Pincer_Arm2_Speed", pMyRobotState->Pincer_Arm2_Speed, ReportState.Pincer_Arm2_Speed);
	ReportState.Pincer_Grip_Position = Update("Pincer_Grip_Position", pMyRobotState->Pincer_Grip_Position, ReportState.Pincer_Grip_Position);
	ReportState.Pincer_Grip_Speed = Update("Pincer_Grip_Speed", pMyRobotState->Pincer_Grip_Speed, ReportState.Pincer_Grip_Speed);
	ReportState.Pincer_Grip_Height = Update("Pincer_Grip_Height", pMyRobotState->Pincer_Grip_Height, ReportState.Pincer_Grip_Height);
	ReportState.Pincer_Grip_Reach = Update("Pincer_Grip_Reach", pMyRobotState->Pincer_Grip_Reach, ReportState.Pincer_Grip_Reach);
	ReportState.ToteWheels_Init = Update("ToteWheels_Init", pMyRobotState->ToteWheels_Init, ReportState.ToteWheels_Init);
	//	ReportState.ToteWheels_Position = Update("ToteWheels_Position", pMyRobotState->ToteWheels_Position, ReportState.ToteWheels_Position);
	ReportState.ToteWheelArm_Left_Position = Update("Left_WheelArm", pMyRobotState->ToteWheelArm_Left_Position, ReportState.ToteWheelArm_Left_Position);
	ReportState.ToteWheelArm_Right_Position = Update("Right_WheelArm", pMyRobotState->ToteWheelArm_Right_Position, ReportState.ToteWheelArm_Right_Position);
	ReportState.ToteWheels_Left_Speed = Update("ToteWheels_Left_Speed", pMyRobotState->ToteWheels_Left_Speed, ReportState.ToteWheels_Left_Speed);
	ReportState.ToteWheels_Right_Speed = Update("ToteWheels_Right_Speed", pMyRobotState->ToteWheels_Right_Speed, ReportState.ToteWheels_Right_Speed);
	ReportState.Autonomous_Init = Update("Autonomous_Init", pMyRobotState->Autonomous_Init, ReportState.Autonomous_Init);
	ReportState.Autonomous_Step = Update("Autonomous_Step", pMyRobotState->Autonomous_Step, ReportState.Autonomous_Step);
	}

bool COMMS :: Update(std::string Label, bool CurrentState, bool PrevState)
	{
	if (CurrentState != PrevState)
		{
		SmartDashboard::PutBoolean(Label,CurrentState);
		}
	return CurrentState;
	}

int COMMS :: Update(std::string Label, int CurrentState, int PrevState)
	{
	if (CurrentState != PrevState)
		{
		SmartDashboard::PutNumber(Label,CurrentState);
		}
	return CurrentState;
	}

float COMMS :: Update(std::string Label, float CurrentState, float PrevState)
	{
	if (CurrentState != PrevState)
		{
		SmartDashboard::PutNumber(Label,CurrentState);
		}
	return CurrentState;
	}

double COMMS :: Update(std::string Label, double CurrentState, double PrevState)
	{
	if (CurrentState != PrevState)
		{
		SmartDashboard::PutNumber(Label,CurrentState);
		}
	return CurrentState;
	}

SpeedControl COMMS :: Update(std::string Label, SpeedControl CurrentState, SpeedControl PrevState)
	{
	if (CurrentState != PrevState)
		{
		SmartDashboard::PutNumber(Label,CurrentState);
		}
	return CurrentState;
	}

DriveOrientation COMMS :: Update(std::string Label, DriveOrientation CurrentState, DriveOrientation PrevState)
	{
	if (CurrentState != PrevState)
		{
		SmartDashboard::PutNumber(Label,CurrentState);
		}
	return CurrentState;
	}


