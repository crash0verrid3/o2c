#include "Comms.h"
//declare the input and output buffers
//they are made global in Definitions.h

COMMS :: COMMS()
{
	Joy1 = new Joystick(0);
	Joy2 = new Joystick(1);
	commsgetcnt = 0;
	pMyAutoTarget = 0;
}

void COMMS :: Init(RobotStateBuffer *pRobotState, INPUT *pInput, RobotStateBuffer *pAutoTarget)
{
	SmartDashboard::init();
	Wait (1);
	SmartDashboard::PutNumber("CommsInit", 1);
	pMyRobotState = pRobotState;
	pMyInput = pInput;
	pMyAutoTarget = pAutoTarget;
	pMyInput->SprintModeBtn = false;
	/*
	 * These are the default values.  No need to change them
	Joy1->SetAxisChannel(Joystick::kXAxis, AXIS_DRIVEX);
	Joy1->SetAxisChannel(Joystick::kYAxis, AXIS_DRIVEY);
	Joy1->SetAxisChannel(Joystick::kTwistAxis, AXIS_DRIVER);
	Joy1->SetAxisChannel(Joystick::kThrottleAxis, AXIS_LIFTER);
	 */
	SmartDashboard::PutNumber("CommsInit", 2);
	Wait (1);
//	Send();		// Don't call Send here because the subsystems have not yet had a chance to initialize all state values
}


void COMMS :: Get()
{
	//get inputs

	pMyRobotState->Comms_GetCount++;

	//get joystick movements
	pMyInput->DriveX = Joy1->GetX();
	pMyInput->DriveY = Joy1->GetY() * INVERT_Y_AXIS_INPUT;
	pMyInput->DriveR = Joy1->GetTwist();
	pMyInput->ToteLift = Joy1->GetThrottle();

	pMyInput->LeftArm = Joy2->GetX();
	pMyInput->LeftWheel = Joy2->GetY();
	pMyInput->RightArm = Joy2->GetTwist() * -1;
	pMyInput->RightWheel = Joy2->GetThrottle();

	//get buttons pressed
//	pMyInput->SprintModeBtn = Joy1->GetRawButton(BTN_SPRINTCONTROL);
//	pMyInput->SpinControlBtn = Joy1->GetRawButton(BTN_SPINCONTROL);
//	pMyInput->OrientModeBtn = Joy1->GetRawButton(BTN_ORIENTMODE);

//	pMyInput->ToteUPBtn = Joy1->GetRawButton(LIFT_UP_BTN);
//	pMyInput->ToteDNBtn = Joy1->GetRawButton(LIFT_DN_BTN);

	pMyInput->SprintModeBtn = OneShot(&input.SprintModeBtn, Joy1->GetRawButton(BTN_SPRINTCONTROL));
	pMyInput->SpinControlBtn = OneShot(&input.SpinControlBtn, Joy1->GetRawButton(BTN_SPINCONTROL));
	pMyInput->OrientModeBtn = OneShot(&input.OrientModeBtn, Joy1->GetRawButton(BTN_ORIENTMODE));
	pMyInput->ToteUPBtn = OneShot(&input.ToteUPBtn, Joy1->GetRawButton(LIFT_UP_BTN));
	pMyInput->ToteDNBtn = OneShot(&input.ToteDNBtn, Joy1->GetRawButton(LIFT_DN_BTN));

}

bool COMMS :: OneShot(bool *input, bool button)
/*
 *	When you press a button, the pMyInput->button value goes high for one cycle.
 *	Then it goes low until the button is released and pressed again.
 */
	{
	bool rtnval;
	rtnval = false;
	if (! button)
		*input = false;
	else if (! *input)
		*input = rtnval = true;
	return rtnval;
	}


void COMMS :: Send()
	{
	// If communications with the SmartDashboard becomes a performance issue, we may wish to rewrite this a bit.
	// We could remove the assignment statements and not return any values from the Update function(s).
	// Then, after we have sent all changed values to the SmartDashboard, simply do a memcpy of the entire state buffer contents.

	#ifdef REPORT_FIELD_STATUS
		Update("DisabledRuns", pMyRobotState->Field_Disabledruns, ReportState.Field_Disabledruns);
		Update("EnabledRuns", pMyRobotState->Field_Enabledruns, ReportState.Field_Enabledruns);
		Update("AutonomousRuns", pMyRobotState->Field_Autoruns, ReportState.Field_Autoruns);
		Update("TeleopRuns", pMyRobotState->Field_Teleruns, ReportState.Field_Teleruns);
		Update("TotalLoops", pMyRobotState->Field_Totalruns, ReportState.Field_Totalruns);
	#endif
	#ifdef REPORT_FIELD_DEBUG
		Update("TestRuns", pMyRobotState->Field_Testruns, ReportState.Field_Testruns);
		Update("MainLoopTime", pMyRobotState->Field_LoopTime, ReportState.Field_LoopTime);
	#endif

	Update("Autonomous_Step", pMyRobotState->Autonomous_Step, ReportState.Autonomous_Step);
	Update("Battery_Voltage", pMyRobotState->Battery_Voltage, ReportState.Battery_Voltage);
	Update("Power_Consumption", pMyRobotState->Power_Consumption, ReportState.Power_Consumption);

	#ifdef REPORT_ROBOT_STATUS
		Update("GyroHeading", pMyRobotState->Robot_Heading, ReportState.Robot_Heading);
		Update("Direction", pMyRobotState->Robot_Direction, ReportState.Robot_Direction);
		Update("Speed", pMyRobotState->Robot_Speed, ReportState.Robot_Speed);
		Update("Xcoord", pMyRobotState->Robot_Position_X, ReportState.Robot_Position_X);
		Update("Ycoord", pMyRobotState->Robot_Position_Y, ReportState.Robot_Position_Y);
	#endif
	#ifdef REPORT_ROBOT_DEBUG
	#endif

	#ifdef REPORT_NAV_STATUS
	#endif
	#ifdef REPORT_NAV_DEBUG
		Update("GyroInitCenter", pMyRobotState->Navigation_GyroCenter, ReportState.Navigation_GyroCenter);
		Update("GyroNoiseLimit", pMyRobotState->Navigation_GyroNoise, ReportState.Navigation_GyroNoise);
		Update("GyroSensitivityCW", pMyRobotState->Navigation_GyroCWsens, ReportState.Navigation_GyroCWsens);
		Update("GyroSensitivityCCW", pMyRobotState->Navigation_GyroCCWsens, ReportState.Navigation_GyroCCWsens);
		Update("SpinEffectCW", pMyRobotState->Navigation_GyroCWfast, ReportState.Navigation_GyroCWfast);
		Update("SpinEffectCCW", pMyRobotState->Navigation_GyroCCWfast, ReportState.Navigation_GyroCCWfast);
	#endif

	#ifdef REPORT_COMMS_STATUS
	#endif
	#ifdef REPORT_COMMS_DEBUG
		Update("Comms GetCount", pMyRobotState->Comms_GetCount, ReportState.Comms_GetCount);
	#endif

	#ifdef REPORT_DRIVE_STATUS
		Update("Drive_SprintFactor", pMyRobotState->Drivetrain_SprintFactor, ReportState.Drivetrain_SprintFactor);
		Update("Drive_Spin_Control", pMyRobotState->Drivetrain_Spin_Control, ReportState.Drivetrain_Spin_Control);
		Update("Drive_SpinCorrection", pMyRobotState->Drivetrain_SpinCorrection, ReportState.Drivetrain_SpinCorrection);
		Update("Drive_Heading", pMyRobotState->Drivetrain_Heading, ReportState.Drivetrain_Heading);
		Update("Drive_Orientation", pMyRobotState->Drivetrain_Directional_Orientation, ReportState.Drivetrain_Directional_Orientation);
	#endif
	#ifdef REPORT_DRIVE_DEBUG
		Update("Drive_Speed_Control", pMyRobotState->Drivetrain_Speed_Control, ReportState.Drivetrain_Speed_Control);
		Update("Drive_FL", pMyRobotState->Drivetrain_FL_Motor_Power, ReportState.Drivetrain_FL_Motor_Power);
		Update("Drive_FR", pMyRobotState->Drivetrain_FR_Motor_Power, ReportState.Drivetrain_FR_Motor_Power);
		Update("Drive_BL", pMyRobotState->Drivetrain_BL_Motor_Power, ReportState.Drivetrain_BL_Motor_Power);
		Update("Drive_BR", pMyRobotState->Drivetrain_BR_Motor_Power, ReportState.Drivetrain_BR_Motor_Power);
	#endif

	#ifdef REPORT_AUTO_STATUS
		Update("AutoState", pMyAutoTarget->Autonomous_State, ReportAuto.Autonomous_State);
		Update("Autonomous Scenario", pMyAutoTarget->Autonomous_Scenario, ReportAuto.Autonomous_Scenario);
	#endif
	#ifdef REPORT_AUTO_DEBUG
		Update("Target_Heading", pMyAutoTarget->Robot_Heading, ReportAuto.Robot_Heading);
		Update("Target_X", pMyAutoTarget->Robot_Position_X, ReportAuto.Robot_Position_X);
		Update("Target_Y", pMyAutoTarget->Robot_Position_Y, ReportAuto.Robot_Position_Y);
		Update("Target_Speed", pMyAutoTarget->Robot_Speed, ReportAuto.Robot_Speed);
		Update("Target_TWLspeed", pMyAutoTarget->ToteWheels_Left_Speed, ReportAuto.ToteWheels_Left_Speed);
		Update("Target_TWRspeed", pMyAutoTarget->ToteWheels_Right_Speed, ReportAuto.ToteWheels_Right_Speed);
		Update("Target_TALposn", pMyAutoTarget->ToteWheelArm_Left_Position, ReportAuto.ToteWheelArm_Left_Position);
		Update("Target_TLRposn", pMyAutoTarget->ToteWheelArm_Right_Position, ReportAuto.ToteWheelArm_Right_Position);
		Update("Target_LiftPosn", pMyAutoTarget->ToteLift_Position, ReportAuto.ToteLift_Position);
		Update("Target_Liftspeed", pMyAutoTarget->ToteLift_Speed, ReportAuto.ToteLift_Speed);
	#endif

	#ifdef REPORT_LIFT_STATUS
		Update("Lift_Position", pMyRobotState->ToteLift_Position, ReportState.ToteLift_Position);
		Update("Lift_Speed", pMyRobotState->ToteLift_Speed, ReportState.ToteLift_Speed);
		Update("Lift_ToteCount", pMyRobotState->ToteLift_ToteCount, ReportState.ToteLift_ToteCount);
	#endif
	#ifdef REPORT_LIFT_DEBUG
		Update("Lift_Target", pMyRobotState->ToteLift_Target, ReportState.ToteLift_Target);
		Update("Lift_Direction", pMyRobotState->ToteLift_Direction, ReportState.ToteLift_Direction);
	#endif

	#ifdef REPORT_WHEEL_STATUS
		Update("TWheel Left Posn", pMyRobotState->ToteWheelArm_Left_Position, ReportState.ToteWheelArm_Left_Position);
		Update("TWheel Right Posn", pMyRobotState->ToteWheelArm_Right_Position, ReportState.ToteWheelArm_Right_Position);
		Update("TWheel Left Speed", pMyRobotState->ToteWheels_Left_Speed, ReportState.ToteWheels_Left_Speed);
		Update("TWheel Right Speed", pMyRobotState->ToteWheels_Right_Speed, ReportState.ToteWheels_Right_Speed);
	#endif
	#ifdef REPORT_WHEEL_DEBUG
		Update("TWheel Left Target", pMyRobotState->ToteWheelArm_Left_Target, ReportState.ToteWheelArm_Left_Target);
		Update("TWheel Right Target", pMyRobotState->ToteWheelArm_Right_Target, ReportState.ToteWheelArm_Right_Target);
		Update("TWheel Left Dir", pMyRobotState->ToteWheelArm_Left_Direction, ReportState.ToteWheelArm_Left_Direction);
		Update("TWheel Right Dir", pMyRobotState->ToteWheelArm_Right_Direction, ReportState.ToteWheelArm_Right_Direction);
	#endif

	#ifdef REPORT_PINCER_STATUS
		Update("Pincer_Init", pMyRobotState->Pincer_Init, ReportState.Pincer_Init);
		Update("Pincer_Arm1_Position", pMyRobotState->Pincer_Arm1_Position, ReportState.Pincer_Arm1_Position);
		Update("Pincer_Arm1_Speed", pMyRobotState->Pincer_Arm1_Speed, ReportState.Pincer_Arm1_Speed);
		Update("Pincer_Arm2_Position", pMyRobotState->Pincer_Arm2_Position, ReportState.Pincer_Arm2_Position);
		Update("Pincer_Arm2_Speed", pMyRobotState->Pincer_Arm2_Speed, ReportState.Pincer_Arm2_Speed);
		Update("Pincer_Grip_Position", pMyRobotState->Pincer_Grip_Position, ReportState.Pincer_Grip_Position);
		Update("Pincer_Grip_Speed", pMyRobotState->Pincer_Grip_Speed, ReportState.Pincer_Grip_Speed);
		Update("Pincer_Grip_Height", pMyRobotState->Pincer_Grip_Height, ReportState.Pincer_Grip_Height);
		Update("Pincer_Grip_Reach", pMyRobotState->Pincer_Grip_Reach, ReportState.Pincer_Grip_Reach);
	#endif
	#ifdef REPORT_PINCER_DEBUG
	#endif

	memcpy(&ReportState,pMyRobotState,sizeof(ReportState));
	memcpy(&ReportAuto,pMyAutoTarget,sizeof(ReportAuto));

	#ifdef REPORT_INPUT_STATUS
	#endif

	#ifdef REPORT_INPUT_DEBUG
		Update("DriveX", pMyInput->DriveX, ReportInput.DriveX);
		Update("DriveY", pMyInput->DriveY, ReportInput.DriveY);
		Update("DriveR", pMyInput->DriveR, ReportInput.DriveR);
		Update("ToteLift", pMyInput->ToteLift, ReportInput.ToteLift);
		Update("LeftArm", pMyInput->LeftArm, ReportInput.LeftArm);
		Update("LeftWheel", pMyInput->LeftWheel, ReportInput.LeftWheel);
		Update("RightArm", pMyInput->RightArm, ReportInput.RightArm);
		Update("RightWheel", pMyInput->RightWheel, ReportInput.RightWheel);
		Update("ToteUPBtn", pMyInput->ToteUPBtn, ReportInput.ToteUPBtn);
		Update("ToteDNBtn", pMyInput->ToteDNBtn, ReportInput.ToteDNBtn);
//		Update("ToteJoyBtn", pMyInput->ToteJoyBtn, ReportInput.ToteJoyBtn);
		Update("SprintModeBtn", pMyInput->SprintModeBtn, ReportInput.SprintModeBtn);
		Update("SpinControlBtn", pMyInput->SpinControlBtn, ReportInput.SpinControlBtn);
		Update("OrientModeBtn", pMyInput->OrientModeBtn, ReportInput.OrientModeBtn);
	#endif

	memcpy(&ReportInput,pMyInput,sizeof(ReportInput));

	return;			// disabled for now

	//send outputs

	Update("Navigation_Robot_Lost", pMyRobotState->Navigation_Robot_Lost, ReportState.Navigation_Robot_Lost);
	Update("Navigation_GyroTilt", pMyRobotState->Navigation_GyroTilt, ReportState.Navigation_GyroTilt);
	}

void COMMS :: Update(std::string Label, bool CurrentState, bool PrevState)
	{
	if (CurrentState != PrevState)
		{
		SmartDashboard::PutBoolean(Label,CurrentState);
		}
	}

void COMMS :: Update(std::string Label, int CurrentState, int PrevState)
	{
	if (CurrentState != PrevState)
		{
		SmartDashboard::PutNumber(Label,CurrentState);
		}
	}

void COMMS :: Update(std::string Label, float CurrentState, float PrevState)
	{
	if (CurrentState != PrevState)
		{
		SmartDashboard::PutNumber(Label,CurrentState);
		}
	}

void COMMS :: Update(std::string Label, double CurrentState, double PrevState)
	{
	if (CurrentState != PrevState)
		{
		SmartDashboard::PutNumber(Label,CurrentState);
		}
	}

void COMMS :: Update(std::string Label, SpeedControl CurrentState, SpeedControl PrevState)
	{
	if (CurrentState != PrevState)
		{
		SmartDashboard::PutNumber(Label,CurrentState);
		}
	}

void COMMS :: Update(std::string Label, DriveOrientation CurrentState, DriveOrientation PrevState)
	{
	if (CurrentState != PrevState)
		{
		SmartDashboard::PutNumber(Label,CurrentState);
		}
	}


