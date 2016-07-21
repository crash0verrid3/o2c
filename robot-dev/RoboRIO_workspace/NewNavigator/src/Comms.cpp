#include "Comms.h"
//declare the input and output buffers
//they are made global in Definitions.h

COMMS :: COMMS()
{
	Joy1 = new Joystick(0);
	Joy2 = new Joystick(1);
	commsgetcnt = 0;
	pMyTargetState = 0;
}

void COMMS :: Init(RobotStateBuffer *pRobotState, INPUT *pInput, RobotStateBuffer *pTargetState, FieldStateBuffer *pFieldState)
{
	SmartDashboard::init();
	Wait (1);
	pMyRobotState = pRobotState;
	pMyInput = pInput;
	pMyTargetState = pTargetState;
	pMyFieldState = pFieldState;
	pMyInput->SprintModeBtn = false;
	/*
	 * These are the default values.  No need to change them
	Joy1->SetAxisChannel(Joystick::kXAxis, AXIS_DRIVEX);
	Joy1->SetAxisChannel(Joystick::kYAxis, AXIS_DRIVEY);
	Joy1->SetAxisChannel(Joystick::kTwistAxis, AXIS_DRIVER);
	Joy1->SetAxisChannel(Joystick::kThrottleAxis, AXIS_LIFTER);
	 */
	pMyRobotState->Comms_Init = 2;
	Wait (1);
	ReadSmartDashboard();

	SmartDashboard::PutNumber("RobotInit",pMyRobotState->Robot_Init);

//	Send();		// Don't call Send here because the subsystems have not yet had a chance to initialize all state values
}


void COMMS :: Get()
{
	//get inputs

//	pMyRobotState->Comms_GetCount++;
	SmartDashboard::PutNumber("Comms GetCount",pMyRobotState->Comms_GetCount++);

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

void COMMS :: ReadSmartDashboard()
/*
 * Read initial state and parameter values from the SmartDashboard (if present), or
 * set initial state and parameter values from default definitions.
 */
	{
	pMyRobotState->Nav_Gyro_Noise_Base = (SmartDashboard::GetNumber("GyroNoiseBase",GYRO_NOISE_BASE));
	pMyRobotState->Nav_Gyro_Squelch = SmartDashboard::GetNumber("GyroNoiseSquelch",GYRO_NOISE_SQUELCH);
	pMyRobotState->Nav_Gyro_Noise_Limit = SmartDashboard::GetNumber("GyroNoiseLimit",GYRO_NOISELIMIT);
	pMyRobotState->Nav_Gyro_Temp_Adjust = (SmartDashboard::GetNumber("GyroTempAdjust",GYRO_THERMO_SENSITIVITY));
	pMyRobotState->Nav_Gyro_X_Sensitivity = (SmartDashboard::GetNumber("GyroXSensitiviy",GYRO_X_SENSITIVITY));
	pMyRobotState->Nav_Gyro_Y_Sensitivity = (SmartDashboard::GetNumber("GyroYSensitiviy",GYRO_Y_SENSITIVITY));
	pMyRobotState->Nav_Gyro_Z_CW_Sensitivity = (SmartDashboard::GetNumber("GyroZCWSensitiviy",GYRO_CW_SENSITIVITY));
	pMyRobotState->Nav_Gyro_Z_CCW_Sensitivity = (SmartDashboard::GetNumber("GyroZCCWSensitiviy",GYRO_CCW_SENSITIVITY));
	pMyRobotState->Nav_Gyro_Z_CW_Spin_Effect = (SmartDashboard::GetNumber("GyroZCWSpinEffect",GYRO_CW_SPIN_CORRECTION)) / 1000;
	pMyRobotState->Nav_Gyro_Z_CCW_Spin_Effect = (SmartDashboard::GetNumber("GyroZCCWSpinEffect",GYRO_CCW_SPIN_CORRECTION)) / 1000;
	pMyRobotState->Nav_Acc_Noise_Base = (SmartDashboard::GetNumber("AccelNoiseBase",ACCEL_NOISE_BASE));
	pMyRobotState->Nav_Acc_Squelch = SmartDashboard::GetNumber("AccelNoiseSquelch",ACCEL_NOISE_SQUELCH);
	pMyRobotState->Nav_Acc_Noise_Limit = SmartDashboard::GetNumber("AccelNoiseLimit",ACCEL_NOISELIMIT);
	pMyRobotState->Nav_Acc_Temp_Adjust = (SmartDashboard::GetNumber("AccelTempAdjust",ACCEL_THERMO_SENSITIVITY));
	pMyRobotState->Nav_Acc_X_Sensitivity = (SmartDashboard::GetNumber("AccelXSensitiviy",ACCEL_X_SENSITIVITY));
	pMyRobotState->Nav_Acc_Y_Sensitivity = (SmartDashboard::GetNumber("AccelYSensitiviy",ACCEL_Y_SENSITIVITY));
	pMyRobotState->Nav_Acc_Z_Sensitivity = (SmartDashboard::GetNumber("AccelZSensitiviy",ACCEL_Z_SENSITIVITY));
	pMyRobotState->Nav_Acc_X_Align = (SmartDashboard::GetNumber("AccelXAlign",ACCEL_X_ALIGN));
	pMyRobotState->Nav_Acc_Y_Align = (SmartDashboard::GetNumber("AccelYAlign",ACCEL_Y_ALIGN));
	pMyRobotState->Nav_Acc_Z_Align = (SmartDashboard::GetNumber("AccelZAlign",ACCEL_Z_ALIGN));
	pMyRobotState->Nav_IMU_Temp_Center = SmartDashboard::GetNumber("TempCenter",GYRO_THERMO_CENTER);
	pMyRobotState->IMU_Accel_X_Offset = SmartDashboard::GetNumber("IMU_Accel_X_Offset",ACCEL_X_OFFSET);
	pMyRobotState->IMU_Accel_Y_Offset = SmartDashboard::GetNumber("IMU_Accel_Y_Offset",ACCEL_Y_OFFSET);
	pMyRobotState->IMU_Accel_Z_Offset = SmartDashboard::GetNumber("IMU_Accel_Z_Offset",ACCEL_Z_OFFSET);
	pMyRobotState->IMU_Mag_X_Offset = SmartDashboard::GetNumber("IMU_Mag_X_Offset",MAG_X_OFFSET);
	pMyRobotState->IMU_Mag_Y_Offset = SmartDashboard::GetNumber("IMU_Mag_Y_Offset",MAG_Y_OFFSET);
	pMyRobotState->IMU_Mag_Z_Offset = SmartDashboard::GetNumber("IMU_Mag_Z_Offset",MAG_Z_OFFSET);
	pMyRobotState->IMU_Gyro_X_Offset = SmartDashboard::GetNumber("IMU_Gyro_X_Offset",GYRO_X_OFFSET);
	pMyRobotState->IMU_Gyro_Y_Offset = SmartDashboard::GetNumber("IMU_Gyro_Y_Offset",GYRO_Y_OFFSET);
	pMyRobotState->IMU_Gyro_Z_Offset = SmartDashboard::GetNumber("IMU_Gyro_Z_Offset",GYRO_Z_OFFSET);
	pMyRobotState->IMU_Acc_Radius = SmartDashboard::GetNumber("IMU_Acc_Radius",ACCEL_RADIUS);
	pMyRobotState->IMU_Mag_Radius = SmartDashboard::GetNumber("IMU_Mag_Radius",MAG_RADIUS);
//	pMyRobotState->IMU_Gyro_H_Compensate_CW = SmartDashboard::GetNumber("IMU_Gyro_H_Compensate_CW",GYRO_EULER_H_DRIFT_CW) / 1000;
//	pMyRobotState->IMU_Gyro_H_Compensate_CCW = SmartDashboard::GetNumber("IMU_Gyro_H_Compensate_CCW",GYRO_EULER_H_DRIFT_CCW) / 1000;
//	pMyRobotState->IMU_Gyro_H_Spin_CW = SmartDashboard::GetNumber("IMU_Gyro_H_Spin_CW",GYRO_EULER_H_SPIN_CW) / 1000;
//	pMyRobotState->IMU_Gyro_H_Spin_CCW = SmartDashboard::GetNumber("IMU_Gyro_H_Spin_CCW",GYRO_EULER_H_SPIN_CCW) / 1000;
//	pMyRobotState->IMU_Gyro_R_Compensate = SmartDashboard::GetNumber("IMU_Gyro_R_Compensate",GYRO_EULER_R_DRIFT) / 1000;
//	pMyRobotState->IMU_Gyro_P_Compensate = SmartDashboard::GetNumber("IMU_Gyro_P_Compensate",GYRO_EULER_P_DRIFT) / 1000;
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
		Update("DisabledRuns", pMyFieldState->disabledruns, ReportField.disabledruns);
		Update("EnabledRuns", pMyFieldState->enabledruns, ReportField.enabledruns);
		Update("AutonomousRuns", pMyFieldState->autoruns, ReportField.autoruns);
		Update("TeleopRuns", pMyFieldState->teleruns, ReportField.teleruns);
		Update("TotalLoops", pMyFieldState->totalruns, ReportField.totalruns);
	#endif
	#ifdef REPORT_FIELD_DEBUG
		Update("TestRuns", pMyFieldState->testruns, ReportField.testruns);
		Update("MainLoopRate", pMyFieldState->looprate, ReportField.looprate);
	#endif

	#ifdef REPORT_ROBOT_STATUS
		Update("Heading", pMyRobotState->Robot_Heading, ReportState.Robot_Heading, RADIANS_TO_DEGREES);
		Update("Direction", pMyRobotState->Robot_Direction, ReportState.Robot_Direction, RADIANS_TO_DEGREES);
		Update("Speed", pMyRobotState->Robot_Speed, ReportState.Robot_Speed);
		Update("Xcoord", pMyRobotState->Robot_Position_X, ReportState.Robot_Position_X);
		Update("Ycoord", pMyRobotState->Robot_Position_Y, ReportState.Robot_Position_Y);
		Update("Battery_Voltage", pMyRobotState->Battery_Voltage, ReportState.Battery_Voltage);
		Update("Power_Consumption", pMyRobotState->Power_Consumption, ReportState.Power_Consumption);
	#endif
	#ifdef REPORT_ROBOT_DEBUG
		Update("RobotInit", pMyRobotState->Robot_Init, ReportState.Robot_Init);
		Update("CommsInit", pMyRobotState->Comms_Init, ReportState.Comms_Init);
		Update("DriveInit", pMyRobotState->DriveTrain_Init, ReportState.DriveTrain_Init);
		Update("AutoInit", pMyRobotState->Autonomous_Init, ReportState.Autonomous_Init);
		Update("LiftInit", pMyRobotState->Lift_Init, ReportState.Lift_Init);
		Update("NavInit", pMyRobotState->Navigator_Init, ReportState.Navigator_Init);
		Update("TWheelInit", pMyRobotState->ToteWheel_Init, ReportState.ToteWheel_Init);
		Update("PincerInit", pMyRobotState->Pincer_Init, ReportState.Pincer_Init);
		Update("Pitch", pMyRobotState->Robot_Pitch, ReportState.Robot_Pitch, RADIANS_TO_DEGREES);
		Update("Roll", pMyRobotState->Robot_Roll, ReportState.Robot_Roll, RADIANS_TO_DEGREES);
	#endif

	#ifdef REPORT_NAV_STATUS
		Update("IMU_Temp", pMyRobotState->Nav_Temp, ReportState.Nav_Temp);					// output in degrees celsius
		Update("Robot_Lost", pMyRobotState->Nav_Robot_Lost, ReportState.Nav_Robot_Lost);
		Update("GyroTilt", pMyRobotState->Nav_Gyro_Tilt, ReportState.Nav_Gyro_Tilt);
		Update("NoisyGyroCnt", pMyRobotState->Nav_Gyro_Noisy_Count, ReportState.Nav_Gyro_Noisy_Count);
	#endif
	#ifdef REPORT_NAV_DEBUG
		Update("IMU_ChipID", pMyRobotState->IMU_ChipID, ReportState.IMU_ChipID);
		Update("IMU_CalStat", pMyRobotState->IMU_CalStat, ReportState.IMU_CalStat);
		Update("IMU_SelftTest", pMyRobotState->IMU_SelfTest, ReportState.IMU_SelfTest);
		Update("IMU_IntrStat", pMyRobotState->IMU_IntrStat, ReportState.IMU_IntrStat);
		Update("IMU_ClockStat", pMyRobotState->IMU_ClockStat, ReportState.IMU_ClockStat);
		Update("IMU_SystemStat", pMyRobotState->IMU_SystemStat, ReportState.IMU_SystemStat);
		Update("IMU_SystemError", pMyRobotState->IMU_SystemError, ReportState.IMU_SystemError);
		Update("IMU_Accel_X_Offset", pMyRobotState->IMU_Accel_X_Offset, ReportState.IMU_Accel_X_Offset);
		Update("IMU_Accel_Y_Offset", pMyRobotState->IMU_Accel_Y_Offset, ReportState.IMU_Accel_Y_Offset);
		Update("IMU_Accel_Z_Offset", pMyRobotState->IMU_Accel_Z_Offset, ReportState.IMU_Accel_Z_Offset);
		Update("IMU_Mag_X_Offset", pMyRobotState->IMU_Mag_X_Offset, ReportState.IMU_Mag_X_Offset);
		Update("IMU_Mag_Y_Offset", pMyRobotState->IMU_Mag_Y_Offset, ReportState.IMU_Mag_Y_Offset);
		Update("IMU_Mag_Z_Offset", pMyRobotState->IMU_Mag_Z_Offset, ReportState.IMU_Mag_Z_Offset);
		Update("IMU_Gyro_X_Offset", pMyRobotState->IMU_Gyro_X_Offset, ReportState.IMU_Gyro_X_Offset);
		Update("IMU_Gyro_Y_Offset", pMyRobotState->IMU_Gyro_Y_Offset, ReportState.IMU_Gyro_Y_Offset);
		Update("IMU_Gyro_Z_Offset", pMyRobotState->IMU_Gyro_Z_Offset, ReportState.IMU_Gyro_Z_Offset);
		Update("IMU_Acc_Radius", pMyRobotState->IMU_Acc_Radius, ReportState.IMU_Acc_Radius);
		Update("IMU_Mag_Radius", pMyRobotState->IMU_Mag_Radius, ReportState.IMU_Mag_Radius);
		Update("IMU_Heading", pMyRobotState->Nav_Gyro_Heading, ReportState.Nav_Gyro_Heading, RADIANS_TO_DEGREES);
		Update("IMU_Roll", pMyRobotState->Nav_Gyro_Roll, ReportState.Nav_Gyro_Roll, RADIANS_TO_DEGREES);
		Update("IMU_Pitch", pMyRobotState->Nav_Gyro_Pitch, ReportState.Nav_Gyro_Pitch, RADIANS_TO_DEGREES);
		Update("Accel_X", pMyRobotState->Nav_Acc_Lin_X, ReportState.Nav_Acc_Lin_X, 1000);	// output in mm/sec/sec
		Update("Accel_Y", pMyRobotState->Nav_Acc_Lin_Y, ReportState.Nav_Acc_Lin_Y, 1000);	// output in mm/sec/sec
		Update("Accel_Z", pMyRobotState->Nav_Acc_Lin_Z, ReportState.Nav_Acc_Lin_Z, 1000);	// output in mm/sec/sec
		Update("TempCenter", pMyRobotState->Nav_IMU_Temp_Center, ReportState.Nav_IMU_Temp_Center);
		Update("Nav_Timestamp", pMyRobotState->Nav_Timestamp, ReportState.Nav_Timestamp);
		Update("Nav_X_Coord", pMyRobotState->Nav_X_Coord, ReportState.Nav_X_Coord);
		Update("Nav_Y_Coord", pMyRobotState->Nav_Y_Coord, ReportState.Nav_Y_Coord);
		Update("Nav_X_Speed", pMyRobotState->Nav_X_Speed, ReportState.Nav_X_Speed);
		Update("Nav_Y_Speed", pMyRobotState->Nav_Y_Speed, ReportState.Nav_Y_Speed);
		Update("Nav_X_Accel", pMyRobotState->Nav_X_Accel, ReportState.Nav_X_Accel);
		Update("Nav_Y_Accel", pMyRobotState->Nav_Y_Accel, ReportState.Nav_Y_Accel);
		Update("Nav_DataRate", pMyRobotState->datarate, ReportState.datarate);
		Update("Nav_UpdateRate", pMyRobotState->updaterate, ReportState.updaterate);
		Update("Nav_DataSpeed", pMyRobotState->dataspeed, ReportState.dataspeed);
		Update("Nav_UpdateSpeed", pMyRobotState->updatespeed, ReportState.updatespeed);
	#endif
	#ifdef REPORT_GYRO_DEBUG
		Update("GyroNoiseBase", pMyRobotState->Nav_Gyro_Noise_Base, ReportState.Nav_Gyro_Noise_Base);
		Update("GyroNoiseSquelch", pMyRobotState->Nav_Gyro_Squelch, ReportState.Nav_Gyro_Squelch);
		Update("GyroNoiseLimit", pMyRobotState->Nav_Gyro_Noise_Limit, ReportState.Nav_Gyro_Noise_Limit);
		Update("GyroXCenter", pMyRobotState->Nav_Gyro_X_Center, ReportState.Nav_Gyro_X_Center);
		Update("GyroYCenter", pMyRobotState->Nav_Gyro_Y_Center, ReportState.Nav_Gyro_Y_Center);
		Update("GyroZCenter", pMyRobotState->Nav_Gyro_Z_Center, ReportState.Nav_Gyro_Z_Center);
		Update("GyroHCenter", pMyRobotState->Nav_Gyro_H_Center, ReportState.Nav_Gyro_H_Center);
		Update("GyroRCenter", pMyRobotState->Nav_Gyro_R_Center, ReportState.Nav_Gyro_R_Center);
		Update("GyroPCenter", pMyRobotState->Nav_Gyro_P_Center, ReportState.Nav_Gyro_P_Center);
		Update("GyroTempAdjust", pMyRobotState->Nav_Gyro_Temp_Adjust, ReportState.Nav_Gyro_Temp_Adjust);
		Update("GyroXSensitiviy", pMyRobotState->Nav_Gyro_X_Sensitivity, ReportState.Nav_Gyro_X_Sensitivity);
		Update("GyroYSensitiviy", pMyRobotState->Nav_Gyro_Y_Sensitivity, ReportState.Nav_Gyro_Y_Sensitivity);
		Update("GyroZCWSensitiviy", pMyRobotState->Nav_Gyro_Z_CW_Sensitivity, ReportState.Nav_Gyro_Z_CW_Sensitivity);
		Update("GyroZCCWSensitiviy", pMyRobotState->Nav_Gyro_Z_CCW_Sensitivity, ReportState.Nav_Gyro_Z_CCW_Sensitivity);
		Update("GyroZCWSpinEffect", pMyRobotState->Nav_Gyro_Z_CW_Spin_Effect, ReportState.Nav_Gyro_Z_CW_Spin_Effect, 1000);
		Update("GyroZCCWSpinEffect", pMyRobotState->Nav_Gyro_Z_CCW_Spin_Effect, ReportState.Nav_Gyro_Z_CCW_Spin_Effect, 1000);
		Update("GyroRawX", pMyRobotState->Nav_Gyro_X_Raw, ReportState.Nav_Gyro_X_Raw);
		Update("GyroRawY", pMyRobotState->Nav_Gyro_Y_Raw, ReportState.Nav_Gyro_Y_Raw);
		Update("GyroRawZ", pMyRobotState->Nav_Gyro_Z_Raw, ReportState.Nav_Gyro_Z_Raw);
//		Update("IMU_Gyro_H_Compensate_CW", pMyRobotState->IMU_Gyro_H_Compensate_CW, ReportState.IMU_Gyro_H_Compensate_CW, 1000);
//		Update("IMU_Gyro_H_Compensate_CCW", pMyRobotState->IMU_Gyro_H_Compensate_CCW, ReportState.IMU_Gyro_H_Compensate_CCW, 1000);
//		Update("IMU_Gyro_H_Spin_CW", pMyRobotState->IMU_Gyro_H_Spin_CW, ReportState.IMU_Gyro_H_Spin_CW, 1000);
//		Update("IMU_Gyro_H_Spin_CCW", pMyRobotState->IMU_Gyro_H_Spin_CCW, ReportState.IMU_Gyro_H_Spin_CCW, 1000);
//		Update("IMU_Gyro_R_Compensate", pMyRobotState->IMU_Gyro_R_Compensate, ReportState.IMU_Gyro_R_Compensate, 1000);
//		Update("IMU_Gyro_P_Compensate", pMyRobotState->IMU_Gyro_P_Compensate, ReportState.IMU_Gyro_P_Compensate, 1000);
	#endif
	#ifdef REPORT_ACCEL_DEBUG
		Update("AccelNoiseBase", pMyRobotState->Nav_Acc_Noise_Base, ReportState.Nav_Acc_Noise_Base);
		Update("AccelNoiseSquelch", pMyRobotState->Nav_Acc_Squelch, ReportState.Nav_Acc_Squelch);
		Update("AccelNoiseLimit", pMyRobotState->Nav_Acc_Noise_Limit, ReportState.Nav_Acc_Noise_Limit);
		Update("AccelXCenter", pMyRobotState->Nav_Acc_X_Center, ReportState.Nav_Acc_X_Center);
		Update("AccelYCenter", pMyRobotState->Nav_Acc_Y_Center, ReportState.Nav_Acc_Y_Center);
		Update("AccelZCenter", pMyRobotState->Nav_Acc_Z_Center, ReportState.Nav_Acc_Z_Center);
		Update("AccelTempAdjust", pMyRobotState->Nav_Acc_Temp_Adjust, ReportState.Nav_Acc_Temp_Adjust);
		Update("AccelXSensitiviy", pMyRobotState->Nav_Acc_X_Sensitivity, ReportState.Nav_Acc_X_Sensitivity);
		Update("AccelYSensitiviy", pMyRobotState->Nav_Acc_Y_Sensitivity, ReportState.Nav_Acc_Y_Sensitivity);
		Update("AccelZSensitiviy", pMyRobotState->Nav_Acc_Z_Sensitivity, ReportState.Nav_Acc_Z_Sensitivity);
		Update("AccelXAlign", pMyRobotState->Nav_Acc_X_Align, ReportState.Nav_Acc_X_Align);
		Update("AccelYAlign", pMyRobotState->Nav_Acc_Y_Align, ReportState.Nav_Acc_Y_Align);
		Update("AccelZAlign", pMyRobotState->Nav_Acc_Z_Align, ReportState.Nav_Acc_Z_Align);
		Update("AccelRawX", pMyRobotState->Nav_Acc_X_Raw, ReportState.Nav_Acc_X_Raw);	// output in mm/sec/sec
		Update("AccelRawY", pMyRobotState->Nav_Acc_Y_Raw, ReportState.Nav_Acc_Y_Raw);	// output in mm/sec/sec
		Update("AccelRawZ", pMyRobotState->Nav_Acc_Z_Raw, ReportState.Nav_Acc_Z_Raw);	// output in mm/sec/sec
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
		Update("Drive_Heading", pMyRobotState->Drivetrain_Heading, ReportState.Drivetrain_Heading, RADIANS_TO_DEGREES);
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
		Update("AutoState", pMyTargetState->Auto_State, ReportTarget.Auto_State);
		Update("Auto Scenario", pMyTargetState->Auto_Scenario_Selection, ReportTarget.Auto_Scenario_Selection);
		Update("Auto Steps", pMyTargetState->Auto_Scenario_Steps, ReportTarget.Auto_Scenario_Steps);
		Update("Autonomous_Step", pMyRobotState->Auto_Step, ReportState.Auto_Step);
		Update("Start_X", pMyTargetState->Robot_Position_X, ReportTarget.Robot_Position_X);
		Update("Start_y", pMyTargetState->Robot_Position_Y, ReportTarget.Robot_Position_Y);
		Update("Start_Heading", pMyTargetState->Robot_Heading, ReportTarget.Robot_Heading);
	#endif
	#ifdef REPORT_AUTO_DEBUG
		Update("Target_Heading", pMyTargetState->Robot_Heading, ReportTarget.Robot_Heading);
		Update("Target_X", pMyTargetState->Robot_Position_X, ReportTarget.Robot_Position_X);
		Update("Target_Y", pMyTargetState->Robot_Position_Y, ReportTarget.Robot_Position_Y);
		Update("Target_Speed", pMyTargetState->Robot_Speed, ReportTarget.Robot_Speed);
		Update("Target_TWLspeed", pMyTargetState->ToteWheels_Left_Speed, ReportTarget.ToteWheels_Left_Speed);
		Update("Target_TWRspeed", pMyTargetState->ToteWheels_Right_Speed, ReportTarget.ToteWheels_Right_Speed);
		Update("Target_TALposn", pMyTargetState->ToteWheelArm_Left_Position, ReportTarget.ToteWheelArm_Left_Position);
		Update("Target_TLRposn", pMyTargetState->ToteWheelArm_Right_Position, ReportTarget.ToteWheelArm_Right_Position);
		Update("Target_LiftPosn", pMyTargetState->ToteLift_Position, ReportTarget.ToteLift_Position);
		Update("Target_Liftspeed", pMyTargetState->ToteLift_Speed, ReportTarget.ToteLift_Speed);
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
	memcpy(&ReportTarget,pMyTargetState,sizeof(ReportTarget));

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

void COMMS :: Update(std::string Label, float CurrentState, float PrevState, float DisplayFactor)
	{
	if (CurrentState != PrevState)
		{
		SmartDashboard::PutNumber(Label,CurrentState * DisplayFactor);
		}
	}

void COMMS :: Update(std::string Label, double CurrentState, double PrevState)
	{
	if (CurrentState != PrevState)
		{
		SmartDashboard::PutNumber(Label,CurrentState);
		}
	}

void COMMS :: Update(std::string Label, double CurrentState, double PrevState, float DisplayFactor)
	{
	if (CurrentState != PrevState)
		{
		SmartDashboard::PutNumber(Label,CurrentState * DisplayFactor);
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


