#include "Comms.h"
//declare the input and output buffers
//they are made global in Definitions.h

COMMS :: COMMS()
{
	Joy1 = new Joystick(0);
	Joy2 = new Joystick(1);
	commsgetcnt = 0;
}

void COMMS :: Init(RobotStateBuffer *pRobotState, RobotStateBuffer *pTargetState, INPUT *pInput)
{
	pMyRobotState = pRobotState;
	pMyInput = pInput;
	pMyTargetState = pTargetState;
	memcpy(&ReportState,pMyRobotState,sizeof(ReportState));
	memcpy(&ReportTarget,pMyTargetState,sizeof(ReportTarget));
	memcpy(&ReportInput,pMyInput,sizeof(ReportInput));
	pMyRobotState->Comms_Init++;
	#ifdef REPORT_COMMS_DEBUG
		SmartDashboard::PutNumber("CommsInit",pMyRobotState->Comms_Init);		// Just push the Robot and Comms Init values
	#endif
	ReadSmartDashboard();
	#ifdef CAMERA
		CameraServer::GetInstance()->SetQuality(pMyRobotState->Sensors_Camera_Quality);
		//the camera name (ex "cam0") can be found through the roborio web interface
		CameraServer::GetInstance()->StartAutomaticCapture("cam0");
	#endif
	Send();		// Call Send while pMyRobotState->Comms_Init is still less than 2, so it writes out all initial values to the SmartDashboard
	pMyRobotState->Comms_Init++;
}


void COMMS :: Get()
{
	//get inputs

	pMyRobotState->Comms_GetCount++;
//	SmartDashboard::PutNumber("Comms GetCount",pMyRobotState->Comms_GetCount++);

	//get joystick movements
	#if DRIVECONTROL==TankDriveControl
		pMyInput->DriveL = Joy1->GetY() * INVERT_Y_AXIS_INPUT;
		pMyInput->DriveR = Joy1->GetThrottle() * INVERT_Y_AXIS_INPUT;
	#elif DRIVECONTROL==ArcadeDriveControl
		pMyInput->DriveX = Joy1->GetX();
		pMyInput->DriveY = Joy1->GetY() * INVERT_Y_AXIS_INPUT;
		pMyInput->DriveRotation = Joy1->GetTwist();
		pMyInput->ToteLift = Joy1->GetThrottle();
	#endif

	//get buttons pressed
		pMyInput->SprintModeBtn = Joy1->GetRawButton(BTN_SPRINTCONTROL);
		pMyInput->SpinControlBtn = Joy1->GetRawButton(BTN_SPINCONTROL);
#if DRIVETRAINTYPE==MecanumDriveTrain
		pMyInput->OrientModeBtn = Joy1->GetRawButton(BTN_ORIENTMODE);
#endif
		/*
			pMyInput->SprintModeBtn = OneShot(&input.SprintModeBtn, Joy1->GetRawButton(BTN_SPRINTCONTROL));
			pMyInput->SpinControlBtn = OneShot(&input.SpinControlBtn, Joy1->GetRawButton(BTN_SPINCONTROL));
			pMyInput->OrientModeBtn = OneShot(&input.OrientModeBtn, Joy1->GetRawButton(BTN_ORIENTMODE));
		*/

		pMyInput->ScooperUpBtn = Joy2->GetRawButton(BTN_SCOOPER_UP);
		pMyInput->ScooperDnBtn = Joy2->GetRawButton(BTN_SCOOPER_DOWN);
		pMyInput->ScopperIntakeBtn = Joy2->GetRawButton(BTN_SCOOPER_INTAKE);
		pMyInput->ShooterFireBtn = Joy2->GetRawButton(BTN_SHOOTER_FIRE);
		pMyInput->ShooterReadyBtn = Joy2->GetRawButton(BTN_SHOOTER_READY);
		pMyInput->ShooterAutoBtn = Joy2->GetRawButton(BTN_SHOOTER_AUTO);

		pMyInput->WinchUpBtn = Joy1->GetRawButton(BTN_WINCH_EXTEND);
		pMyInput->WinchDnBtn = Joy1->GetRawButton(BTN_WINCH_RETRACT);

		pMyInput->NavigationRestBtn = Joy1->GetRawButton(BTN_NAV_RESET);



}

/*
bool COMMS :: OneShot(bool *input, bool button)
 *
 *	When you press a button, the pMyInput->button value goes high for one cycle.
 *	Then it goes low until the button is released and pressed again.
 *
 *
 *	Bummer.  Does not help when the button bounces many times in the split second that you touch it.
 *			Need a new gamepad.
 *
	{
	bool rtnval;
	rtnval = false;
	if (! button)
		*input = false;
	else if (! *input)
		*input = rtnval = true;
	return rtnval;
	}
*/


void COMMS :: ReadSmartDashboard()
/*
 * Read initial state and parameter values from the SmartDashboard (if present), or
 * set initial state and parameter values from default definitions.
 */
	{
	#ifdef CAMERA
		pMyRobotState->Sensors_Camera_Quality = (SmartDashboard::GetNumber("CameraQuality",CAMERA_IMAGE_QUALITY));
	#endif
	pMyRobotState->Nav_Gyro_Noise_Base = (SmartDashboard::GetNumber("GyroNoiseBase",GYRO_NOISE_BASE));
	pMyRobotState->Nav_Gyro_Squelch = SmartDashboard::GetNumber("GyroNoiseSquelch",GYRO_NOISE_SQUELCH);
	pMyRobotState->Nav_Gyro_Noise_Limit = SmartDashboard::GetNumber("GyroNoiseLimit",GYRO_NOISELIMIT);
	pMyRobotState->Nav_Gyro_Temp_Adjust = (SmartDashboard::GetNumber("GyroTempAdjust",GYRO_THERMO_SENSITIVITY));
	pMyRobotState->Nav_Gyro_X_U_Sensitivity = (SmartDashboard::GetNumber("GyroXUSensitiviy",GYRO_X_U_SENSITIVITY));
	pMyRobotState->Nav_Gyro_X_D_Sensitivity = (SmartDashboard::GetNumber("GyroXDSensitiviy",GYRO_X_D_SENSITIVITY));
	pMyRobotState->Nav_Gyro_Y_L_Sensitivity = (SmartDashboard::GetNumber("GyroYLSensitiviy",GYRO_Y_L_SENSITIVITY));
	pMyRobotState->Nav_Gyro_Y_R_Sensitivity = (SmartDashboard::GetNumber("GyroYRSensitiviy",GYRO_Y_R_SENSITIVITY));
	pMyRobotState->Nav_Gyro_Z_CW_Sensitivity = (SmartDashboard::GetNumber("GyroZCWSensitiviy",GYRO_CW_SENSITIVITY));
	pMyRobotState->Nav_Gyro_Z_CCW_Sensitivity = (SmartDashboard::GetNumber("GyroZCCWSensitiviy",GYRO_CCW_SENSITIVITY));
	pMyRobotState->Nav_Gyro_Z_CW_Spin_Effect = (SmartDashboard::GetNumber("GyroZCWSpinEffect",GYRO_CW_SPIN_CORRECTION)) / 1000;
	pMyRobotState->Nav_Gyro_Z_CCW_Spin_Effect = (SmartDashboard::GetNumber("GyroZCCWSpinEffect",GYRO_CCW_SPIN_CORRECTION)) / 1000;
	pMyRobotState->Nav_Gyro_X_PWR_Sensitivity = (SmartDashboard::GetNumber("GyroXPwrSensitivity",GYRO_X_PWR_SENSITIVITY)) / 1000;
	pMyRobotState->Nav_Gyro_Y_PWR_Sensitivity = (SmartDashboard::GetNumber("GyroYPwrSensitivity",GYRO_Y_PWR_SENSITIVITY)) / 1000;
	pMyRobotState->Nav_Gyro_Z_PWR_Sensitivity = (SmartDashboard::GetNumber("GyroZPwrSensitivity",GYRO_Z_PWR_SENSITIVITY)) / 1000;
	#ifdef MPU6050_6DOF
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
	#endif
	#ifdef BNO055_9DOF
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
	#endif	// BNO)%%_9DOF
//	pMyRobotState->IMU_Gyro_H_Compensate_CW = SmartDashboard::GetNumber("IMU_Gyro_H_Compensate_CW",GYRO_EULER_H_DRIFT_CW) / 1000;
//	pMyRobotState->IMU_Gyro_H_Compensate_CCW = SmartDashboard::GetNumber("IMU_Gyro_H_Compensate_CCW",GYRO_EULER_H_DRIFT_CCW) / 1000;
//	pMyRobotState->IMU_Gyro_H_Spin_CW = SmartDashboard::GetNumber("IMU_Gyro_H_Spin_CW",GYRO_EULER_H_SPIN_CW) / 1000;
//	pMyRobotState->IMU_Gyro_H_Spin_CCW = SmartDashboard::GetNumber("IMU_Gyro_H_Spin_CCW",GYRO_EULER_H_SPIN_CCW) / 1000;
//	pMyRobotState->IMU_Gyro_R_Compensate = SmartDashboard::GetNumber("IMU_Gyro_R_Compensate",GYRO_EULER_R_DRIFT) / 1000;
//	pMyRobotState->IMU_Gyro_P_Compensate = SmartDashboard::GetNumber("IMU_Gyro_P_Compensate",GYRO_EULER_P_DRIFT) / 1000;
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
		Update("MainLoopRate", pMyRobotState->Field_LoopTime, ReportState.Field_LoopTime);
//		for (int i=0; i<=4; i++)
//			{
//			Update("Defense["+std::to_string(i)+"]", pMyRobotState->Field_Defense[i], ReportState.Field_Defense[i]);
//			}
	#endif

	#ifdef REPORT_ROBOT_STATUS
		Update("Heading", pMyRobotState->Robot_Heading, ReportState.Robot_Heading, RADIANS_TO_DEGREES);
		Update("Pitch", pMyRobotState->Robot_Pitch, ReportState.Robot_Pitch, RADIANS_TO_DEGREES);
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
		Update("NavInit", pMyRobotState->Navigator_Init, ReportState.Navigator_Init);
		Update("ScoopInit", pMyRobotState->Scooper_Init, ReportState.Scooper_Init);
		Update("WinchInit", pMyRobotState->Winch_Init, ReportState.Winch_Init);
		Update("Roll", pMyRobotState->Robot_Roll, ReportState.Robot_Roll, RADIANS_TO_DEGREES);
		Update("SpinRate", pMyRobotState->Robot_Spin_Rate, ReportState.Robot_Spin_Rate);
	#endif

	#ifdef REPORT_SENSORS_STATUS
		Update("ShooterPower", pMyRobotState->Shooter_Launch_Power, ReportState.Shooter_Launch_Power);
		#ifdef CAMERA
			Update("CameraQuality", pMyRobotState->Sensors_Camera_Quality, ReportState.Sensors_Camera_Quality);
		#endif
		#ifdef PHOTO_SENSORS
			Update("LeftTargetSensor", pMyRobotState->Sensors_LeftPhoto, ReportState.Sensors_LeftPhoto);
			Update("RightTargetSensor", pMyRobotState->Sensors_RightPhoto, ReportState.Sensors_RightPhoto);
		#endif
		#ifdef LIGHT_SENSOR
			Update("LightSensor", pMyRobotState->Sensors_LightDetect, ReportState.Sensors_LightDetect);
		#endif
		#ifdef RANGE_FINDER
			Update("Range", pMyRobotState->Sensors_Range, ReportState.Sensors_Range);
		#endif
	#endif

	#ifdef REPORT_SENSORS_DEBUG
	#endif

	#ifdef REPORT_NAV_STATUS
		Update("IMU_Temp", pMyRobotState->Nav_Temp, ReportState.Nav_Temp);					// output in degrees celsius
		Update("Robot_Lost", pMyRobotState->Nav_Robot_Lost, ReportState.Nav_Robot_Lost);
		Update("GyroTilt", pMyRobotState->Nav_Gyro_Tilt, ReportState.Nav_Gyro_Tilt);
		Update("NoisyGyroCnt", pMyRobotState->Nav_Gyro_Noisy_Count, ReportState.Nav_Gyro_Noisy_Count);
	#endif
	#ifdef REPORT_NAV_DEBUG
		#ifdef BNO055_9DOF
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
		#endif	// BNO055_9DOF
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
		#ifdef BNO055_9DOF
			Update("GyroHCenter", pMyRobotState->Nav_Gyro_H_Center, ReportState.Nav_Gyro_H_Center);
			Update("GyroRCenter", pMyRobotState->Nav_Gyro_R_Center, ReportState.Nav_Gyro_R_Center);
			Update("GyroPCenter", pMyRobotState->Nav_Gyro_P_Center, ReportState.Nav_Gyro_P_Center);
		#endif
		Update("GyroTempAdjust", pMyRobotState->Nav_Gyro_Temp_Adjust, ReportState.Nav_Gyro_Temp_Adjust);
		Update("GyroXPwrSensitivity", pMyRobotState->Nav_Gyro_X_PWR_Sensitivity, ReportState.Nav_Gyro_X_PWR_Sensitivity, 1000);
		Update("GyroYPwrSensitivity", pMyRobotState->Nav_Gyro_Y_PWR_Sensitivity, ReportState.Nav_Gyro_Y_PWR_Sensitivity, 1000);
		Update("GyroZPwrSensitivity", pMyRobotState->Nav_Gyro_Z_PWR_Sensitivity, ReportState.Nav_Gyro_Z_PWR_Sensitivity, 1000);
		Update("GyroXUSensitiviy", pMyRobotState->Nav_Gyro_X_U_Sensitivity, ReportState.Nav_Gyro_X_U_Sensitivity);
		Update("GyroXDSensitiviy", pMyRobotState->Nav_Gyro_X_D_Sensitivity, ReportState.Nav_Gyro_X_D_Sensitivity);
		Update("GyroYLSensitiviy", pMyRobotState->Nav_Gyro_Y_L_Sensitivity, ReportState.Nav_Gyro_Y_L_Sensitivity);
		Update("GyroYRSensitiviy", pMyRobotState->Nav_Gyro_Y_R_Sensitivity, ReportState.Nav_Gyro_Y_R_Sensitivity);
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
		Update("Drive_Orientation", pMyRobotState->Drivetrain_Directional_Orientation, ReportState.Drivetrain_Directional_Orientation);
	#endif
	#ifdef REPORT_DRIVE_DEBUG
		#if DRIVETRAINTYPE==TankDriveTrain
			Update("Drive_Left", pMyRobotState->Drivetrain_Left_Motor_Power, ReportState.Drivetrain_Left_Motor_Power);
			Update("Drive_Right", pMyRobotState->Drivetrain_Right_Motor_Power, ReportState.Drivetrain_Right_Motor_Power);
		#elif DRIVETRAINTYPE==MecanumDriveTrain
			Update("Drive_FL", pMyRobotState->Drivetrain_FL_Motor_Power, ReportState.Drivetrain_FL_Motor_Power);
			Update("Drive_FR", pMyRobotState->Drivetrain_FR_Motor_Power, ReportState.Drivetrain_FR_Motor_Power);
			Update("Drive_BL", pMyRobotState->Drivetrain_BL_Motor_Power, ReportState.Drivetrain_BL_Motor_Power);
			Update("Drive_BR", pMyRobotState->Drivetrain_BR_Motor_Power, ReportState.Drivetrain_BR_Motor_Power);
		#endif
		Update("Drive_Speed_Control", pMyRobotState->Drivetrain_Speed_Control, ReportState.Drivetrain_Speed_Control);
		Update("DriftAngle", pMyRobotState->Drivetrain_DriftAngle, ReportState.Drivetrain_DriftAngle, RADIANS_TO_DEGREES);
	#endif

	#ifdef REPORT_AUTO_STATUS
		Update("AutoState", pMyRobotState->Auto_State, ReportState.Auto_State);
		Update("Start_X", pMyRobotState->Auto_Start_X, ReportState.Auto_Start_X);
		Update("Start_Y", pMyRobotState->Auto_Start_Y, ReportState.Auto_Start_Y);
//		Update("Start_Z", pMyRobotState->Auto_Start_Z, ReportState.Auto_Start_Z);						// no vertical starting position this year
		Update("Start_Heading", pMyRobotState->Auto_Start_Heading, ReportState.Auto_Start_Heading);
		Update("Autonomous_Step", pMyRobotState->Auto_Step, ReportState.Auto_Step);
		Update("Auto Total Steps", pMyRobotState->Auto_Total_Steps, ReportState.Auto_Total_Steps);
	#endif
	#ifdef REPORT_AUTO_DEBUG
		Update("Target_Heading", pMyTargetState->Robot_Heading, ReportTarget.Robot_Heading);
		Update("Target_X", pMyTargetState->Robot_Position_X, ReportTarget.Robot_Position_X);
		Update("Target_Y", pMyTargetState->Robot_Position_Y, ReportTarget.Robot_Position_Y);
		Update("Target_Speed", pMyTargetState->Robot_Speed, ReportTarget.Robot_Speed);
		Update("Auto_Start_Position", pMyRobotState->Auto_Start_Position, ReportState.Auto_Start_Position);
		Update("Auto Goal", pMyRobotState->Auto_Goal, ReportState.Auto_Goal);
//		Update("Auto Goal Steps", pMyRobotState->Auto_Goal_Steps, ReportState.Auto_Goal_Steps);
		Update("Auto Defense", pMyRobotState->Auto_Defense, ReportState.Auto_Defense);
//		Update("Auto Defense Steps", pMyRobotState->Auto_Defense_Steps, ReportState.Auto_Defense_Steps);
	#endif


	#ifdef REPORT_SCOOPER_STATUS
		Update("ScooperElevation", pMyRobotState->Shooter_Elevation, ReportState.Shooter_Elevation);
	#endif
	#ifdef REPORT_SCOOPER_DEBUG
		Update("TargetElevation", pMyTargetState->Shooter_Elevation, ReportTarget.Shooter_Elevation);
		Update("Shooter_Elevator_State", pMyRobotState->Shooter_Elevator_State, ReportState.Shooter_Elevator_State);
		Update("Scooper_Elevation_Speed", pMyRobotState->Shooter_Elevation_Speed, ReportState.Shooter_Elevation_Speed);
	#endif

#ifdef REPORT_SHOOTER_STATUS
	Update("Boulder_Position", pMyRobotState->Shooter_Boulder_Position, ReportState.Shooter_Boulder_Position);
#endif
#ifdef REPORT_SHOOTER_DEBUG
	Update("TargetFirePower", pMyTargetState->Shooter_Launch_Power, ReportTarget.Shooter_Launch_Power);
#endif

#ifdef REPORT_WINCH_STATUS
	Update("Winch_Position", pMyRobotState->Winch_Position, ReportState.Winch_Position);
	Update("Winch_Power", pMyRobotState->Winch_Power, ReportState.Winch_Power);
	Update("Winch_Lock_Posn", pMyRobotState->Winch_Lock_Posn, ReportState.Winch_Lock_Posn);
#endif
#ifdef REPORT_WINCH_DEBUG
//	Update("Climber_Elevation_Speed", pMyRobotState->Climber_Elevation_Speed, ReportState.Climber_Elevation_Speed);
//	Update("Climber_Extension_Power", pMyRobotState->Climber_Extension_Power, ReportState.Climber_Extension_Power);
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


	memcpy(&ReportState,pMyRobotState,sizeof(ReportState));
	memcpy(&ReportTarget,pMyTargetState,sizeof(ReportTarget));

	#ifdef REPORT_INPUT_STATUS
	#endif

	#ifdef REPORT_INPUT_DEBUG
		Update("DriveL", pMyInput->DriveL, ReportInput.DriveL);
		Update("DriveR", pMyInput->DriveR, ReportInput.DriveR);
//		Update("SprintModeBtn", pMyInput->SprintModeBtn, ReportInput.SprintModeBtn);
//		Update("SpinControlBtn", pMyInput->SpinControlBtn, ReportInput.SpinControlBtn);
		Update("ScopperIntakeBtn", pMyInput->ScopperIntakeBtn, ReportInput.ScopperIntakeBtn);
		Update("ShooterReadyBtn", pMyInput->ShooterReadyBtn, ReportInput.ShooterReadyBtn);
		Update("ShooterFireBtn", pMyInput->ShooterFireBtn, ReportInput.ShooterFireBtn);
	#endif

	memcpy(&ReportInput,pMyInput,sizeof(ReportInput));

	return;			// disabled for now

	//send outputs

	}

void COMMS :: Update(std::string Label, bool CurrentState, bool PrevState)
	{
	if ((CurrentState != PrevState) || (pMyRobotState->Comms_Init < 2))
		{
		SmartDashboard::PutBoolean(Label,CurrentState);
		}
	}

void COMMS :: Update(std::string Label, int CurrentState, int PrevState)
	{
	if ((CurrentState != PrevState) || (pMyRobotState->Comms_Init < 2))
		{
		SmartDashboard::PutNumber(Label,CurrentState);
		}
	}

void COMMS :: Update(std::string Label, float CurrentState, float PrevState)
	{
	if ((CurrentState != PrevState) || (pMyRobotState->Comms_Init < 2))
		{
		SmartDashboard::PutNumber(Label,CurrentState);
		}
	}

void COMMS :: Update(std::string Label, float CurrentState, float PrevState, float DisplayFactor)
	{
	if ((CurrentState != PrevState) || (pMyRobotState->Comms_Init < 2))
		{
		SmartDashboard::PutNumber(Label,CurrentState * DisplayFactor);
		}
	}

void COMMS :: Update(std::string Label, double CurrentState, double PrevState)
	{
	if ((CurrentState != PrevState) || (pMyRobotState->Comms_Init < 2))
		{
		SmartDashboard::PutNumber(Label,CurrentState);
		}
	}

void COMMS :: Update(std::string Label, double CurrentState, double PrevState, float DisplayFactor)
	{
	if ((CurrentState != PrevState) || (pMyRobotState->Comms_Init < 2))
		{
		SmartDashboard::PutNumber(Label,CurrentState * DisplayFactor);
		}
	}

void COMMS :: Update(std::string Label, SpeedControl CurrentState, SpeedControl PrevState)
	{
	if ((CurrentState != PrevState) || (pMyRobotState->Comms_Init < 2))
		{
		SmartDashboard::PutNumber(Label,CurrentState);
		}
	}

void COMMS :: Update(std::string Label, DriveOrientation CurrentState, DriveOrientation PrevState)
	{
	if ((CurrentState != PrevState) || (pMyRobotState->Comms_Init < 2))
		{
		SmartDashboard::PutNumber(Label,CurrentState);
		}
	}


