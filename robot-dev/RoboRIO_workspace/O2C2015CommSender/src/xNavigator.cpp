#include <Navigator.h>

NAVIGATOR :: NAVIGATOR() :	INS_semaphore(0)

	{
	initcnt = 0;
	INS_Calibrating = false;
	pReadSensors = new Notifier(NAVIGATOR::CallReadSensors,this);
	pINSupdate = new Notifier(NAVIGATOR::CallUpdateINS,this);
	Navigation_Data_Queue_Depth = 0;
	Navigation_Data_Frame_Size = sizeof(Navigation_Data_Queue) / NAVIGATION_QUEUE_SIZE;
	Navigation_Data_Queue_Top = Navigation_Data_Queue_Bottom = 0;
	x_spin_rate = y_spin_rate = z_spin_rate = 0;
	sample_counter = 0;
	NavClockTime = 0;
	prevx = x_accel = y_accel = z_accel = 0;
	Xdeviation = XWHEEL_ANGLE_OFFSET;
	Ydeviation = YWHEEL_ANGLE_OFFSET;
	XtipAngle = YtipAngle = 0;
	XspeedCenter = YspeedCenter = 0;
	GyroInitCenter = 0;
	AccelInitCenterX = AccelInitCenterY = AccelInitCenterZ = 0;
	temperature = GyroCenter_T = GYRO_THERMO_CENTER;
	NavElapsedTime = NAVtemperature = NAVx_accel = NAVy_accel = NAVz_accel = 0;
	NAVx_spin = NAVy_spin = NAVz_spin = 0;
	x_wheel = y_wheel = NAVx_wheel = NAVy_wheel = prev_x_wheel = prev_y_wheel = 0;
	fifo_queue_cnt = 0;

	Navigation_Status_Buffer.Xcoord = Navigation_Status_Buffer.Ycoord
									= Navigation_Status_Buffer.Xspeed
									= Navigation_Status_Buffer.Yspeed
									= Navigation_Status_Buffer.Xaccel
									= Navigation_Status_Buffer.Yaccel
									= Navigation_Status_Buffer.Timestamp = 0;
	#ifndef MPU6050_6DOF
		#ifdef ITG3200_GYRO
			MyGyro = new I2C(I2C::kOnboard, GYRO_I2C_ADDR);	// locate the gyro on the I2C bus
//			MyGyro = new I2C(I2C::kOnboard, GYRO_I2C_ALTADDR);	// locate the gyro on the I2C bus
			cw_sensitivity = GYRO_CW_SENSITIVITY / 1000000000;		// Gyro is not consistent between clockwise turns and counter-clockwise turns
			ccw_sensitivity = GYRO_CCW_SENSITIVITY / 1000000000;	// Up to about 0.5 degrees per revolution difference between them
			GyroNoiseLimit = GYRO_NOISELIMIT;
			GyroCenter_X = GyroCenter_Y = GyroCenter_Z = GYRO_RATECENTER / 1000;
		#endif
		#ifdef ADXL345_ACCEL
			MyAccel = new I2C(I2C::kOnboard, ACCEL_I2C_ADDR);	// locate the gyro on the I2C bus
//			MyAccel = new I2C(I2C::kOnboard, ACCEL_I2C_ALTADDR);	// locate the gyro on the I2C bus
			XaccelCenter = ACCEL_XCENTER / 1000;
			YaccelCenter = ACCEL_YCENTER / 1000;
			ZaccelCenter = ACCEL_ZCENTER / 1000;
		#endif
		#ifdef BUILTIN_ACCEL
			MyAccel = new BuiltInAccelerometer();
			XaccelCenter = ACCEL_XCENTER / 1000;
			YaccelCenter = ACCEL_YCENTER / 1000;
			ZaccelCenter = ACCEL_ZCENTER / 1000;
		#endif
	#endif
	#ifdef MPU6050_6DOF
		MyGyro = new I2C(I2C::kOnboard, DOF6_I2C_ADDR);	// locate the gyro on the I2C bus
		cw_sensitivity = GYRO_CW_SENSITIVITY / 1000000000;		// Gyro is not consistent between clockwise turns and counter-clockwise turns
		ccw_sensitivity = GYRO_CCW_SENSITIVITY / 1000000000;	// Up to about 0.5 degrees per revolution difference between them
		GyroNoiseLimit = GYRO_NOISELIMIT;
		GyroCenter_X = GyroCenter_Y = GyroCenter_Z = GYRO_RATECENTER / 1000;
		XaccelCenter = ACCEL_XCENTER / 1000;
		YaccelCenter = ACCEL_YCENTER / 1000;
		ZaccelCenter = ACCEL_ZCENTER / 1000;
	#endif
	Xwheel = new Encoder(XWHEEL_DIO_PORT1, XWHEEL_DIO_PORT2, XWHEEL_REVERSE_DIRECTION, Encoder::k4X);
	Ywheel = new Encoder(YWHEEL_DIO_PORT1, YWHEEL_DIO_PORT2, YWHEEL_REVERSE_DIRECTION, Encoder::k4X);

	NavLoopCnt = 0;
	SampleElapsedTime = 0;
	SampleRunTime = 0;
	GyroHeading = 0;	// actually want to read from SmartDashboard 'cause we might want to start the robot facing otherwise
	RobotDirection = 0;
	GyroTilt = false;
	noisyGyrocnt = 0;
	noisyAccelcnt = 0;
	gyro_accumulator_count = 0;
	dataNotRdyCnt = 0;
	prevcount = 0;
	SpinEffectCW = 0;
	SpinEffectCCW = 0;
	GyroSensitivity = (cw_sensitivity + ccw_sensitivity) / 2;
	GyroXsensitivity = GyroYsensitivity = 0;
	Sensitivity_Offset = 0;		// we'll use this later to adjust sensitivity as a function of temperature (it does matter)
	thermo_sensitivity = 0;
	Xsensitivity = ACCEL_XSENSITIVITY;
	Ysensitivity = ACCEL_YSENSITIVITY;
	FrictionEffect = ACCEL_FRICTION;
	gyro_frame_index = 0;
	accel_frame_index = 6;
	gyro_noise_base = GYRO_NOISE_BASE / 1000;
	gyro_noise_squelch = GYRO_NOISE_SQUELCH;
	accel_noise_base = ACCEL_NOISE_BASE / 1000;
	accel_noise_squelch = ACCEL_NOISE_SQUELCH;
	}


#ifdef ITG3200_GYRO
void NAVIGATOR :: InitMyGyro()
	{
	// Get the gyro tuning parameters from the SmartDashboard.
	GyroCenter_Z = GyroInitCenter = (SmartDashboard::GetNumber("GyroInitCenter",GYRO_RATECENTER) / 1000);
	GyroNoiseLimit = SmartDashboard::GetNumber("GyroNoiseLimit",GYRO_NOISELIMIT);
	// Did I mention that the gyro doesn't give the same
	// voltage shift / dps for clockwise and counter-clockwise.
	// In other words, the voltage shift does not appear to be entirely linear.
	// It's just a teeny, tiny bit off.
	cw_sensitivity = SmartDashboard::GetNumber("GyroSensitivityCW",GYRO_CW_SENSITIVITY) / 1000000000;
	ccw_sensitivity = SmartDashboard::GetNumber("GyroSensitivityCCW",GYRO_CCW_SENSITIVITY) / 1000000000;
	GyroSensitivity = (cw_sensitivity + ccw_sensitivity) / 2;
	// Did I mention that, if the gyro is not placed in the exact center of rotation
	// of the robot, then the above mentioned issue is further exagerated as speed
	// of rotation increases.
	SpinEffectCW = SmartDashboard::GetNumber("SpinEffectCW",GYRO_CW_SPIN_CORRECTION) / 1000000000000;
	SpinEffectCCW = SmartDashboard::GetNumber("SpinEffectCCW",GYRO_CCW_SPIN_CORRECTION) / 1000000000000;

	// Configure and initialize the gyro
	if (! MyGyro->AddressOnly())		// if the gyro is present, initialize its registers
		{
		initcnt++;
//		if ((! MyGyro->Write(GYRO_Reg_PWR_MGM, GYRO_H_Reset)))
			{
			Wait(1);
			if ((! MyGyro->Write((uint8_t) GYRO_Reg_PWR_MGM, (uint8_t) GYRO_Clock_Z))
					&& (! MyGyro->Write((uint8_t) GYRO_Reg_DLPF_FS, (uint8_t) (GYRO_2000dps | GYRO_8kHz)))
					&& (! MyGyro->Write((uint8_t) GYRO_Reg_SMPLRT_DIV, (uint8_t) GYRO_5Samples))
					&& (! MyGyro->Write((uint8_t) GYRO_Reg_INT_CFG, (uint8_t) (GYRO_DataRdy | GYRO_Latch))))
//				&& (! MyGyro->Write((uint8_t) GYRO_Reg_SMPLRT_DIV, (uint8_t) GYRO_SampleDiv))
					//&& (! MyGyro->Write((uint8_t) GYRO_Reg_INT_CFG, (uint8_t) (GYRO_DataRdy | GYRO_Latch | GYRO_AnyRd2Clr))))
					//&& (! MyGyro->Write((uint8_t) GYRO_Reg_ZoffsH, (uint8_t) ((int16_t) GYRO_RATECENTER >> 8)))
					//&& (! MyGyro->Write((uint8_t) GYRO_Reg_ZoffsL, (uint8_t) ((int16_t) GYRO_RATECENTER & 0xFF))))
					//	&& (! MyGyro->Write((uint8_t) GYRO_Reg_FIFOen, (uint8_t) GYRO_FIFO_GX | GYRO_FIFO_GZ | GYRO_FIFO_ft)))
				{
				initcnt++;
				}
			}
		}
	// If these parameters are not currently on the SmartDashboard, the values will default
	// to pre-programmed values.  Write out those values so the operator can change them to
	// whatever they want and then restart the RoboRIO User Program to pick up the new values.
	SmartDashboard::PutNumber("GyroInitCenter",GyroCenter_Z * 1000);
	SmartDashboard::PutNumber("GyroNoiseLimit",GyroNoiseLimit);
	SmartDashboard::PutNumber("GyroSensitivityCW",cw_sensitivity * 1000000000);
	SmartDashboard::PutNumber("GyroSensitivityCCW",ccw_sensitivity * 1000000000);
	SmartDashboard::PutNumber("SpinEffectCW",SpinEffectCW * 1000000000000);
	SmartDashboard::PutNumber("SpinEffectCCW",SpinEffectCCW * 1000000000000);
	}
#endif


#ifdef ITG3200_GYRO
void NAVIGATOR :: InitMyThermostat()
	{
	thermo_sensitivity = (SmartDashboard::GetNumber("Thermo_sensitivity",GYRO_THERMO_SENSITIVITY) / 1000000000);
	temperature = GyroCenter_T = GYRO_THERMO_CENTER;
	SmartDashboard::PutNumber("Thermo_sensitivity",thermo_sensitivity * 1000000000);
	// The temperature probe is built into the I2C gyro and was configured with the gyro.
	}
#endif


#ifdef BUILTIN_ACCEL
void NAVIGATOR :: InitMyAccelerometer()
	{
//	MyAccel->SetRange(BuiltInAccelerometer::Accelerometer::kRange_2G);
	MyAccel->SetRange(BuiltInAccelerometer::kRange_2G);
	XaccelCenter = (SmartDashboard::GetNumber("XaccelCenter",ACCEL_XCENTER) / 1000);
	YaccelCenter = (SmartDashboard::GetNumber("YaccelCenter",ACCEL_YCENTER) / 1000);
	ZaccelCenter = (SmartDashboard::GetNumber("ZaccelCenter",ACCEL_ZCENTER) / 1000);
	Xsensitivity = SmartDashboard::GetNumber("Xsensitivity",ACCEL_XSENSITIVITY);
	Ysensitivity = SmartDashboard::GetNumber("Ysensitivity",ACCEL_YSENSITIVITY);
	FrictionEffect = SmartDashboard::GetNumber("FrictionEffect",ACCEL_FRICTION);
	SmartDashboard::PutNumber("XaccelCenter", XaccelCenter);
	SmartDashboard::PutNumber("YaccelCenter", YaccelCenter);
	SmartDashboard::PutNumber("ZaccelCenter", ZaccelCenter);
	SmartDashboard::PutNumber("Xsensitivity", Xsensitivity);
	SmartDashboard::PutNumber("Ysensitivity", Ysensitivity);
	SmartDashboard::PutNumber("FrictionEffect",FrictionEffect);
	}
#endif


#ifdef ADXL345_ACCEL
void NAVIGATOR :: InitMyAccelerometer()
	{
//	byte	bw_rate = 0;

	if (! MyAccel->AddressOnly()				// if the accelerometer is present, initialize its registers
			&& (! MyAccel->Write((uint8_t) ACCEL_PWR_CTL, (uint8_t) ACCEL_PWR_STANDBY)))
		{
		Wait (0.3);
		initcnt++;
		if (! MyAccel->Write((uint8_t) ACCEL_PWR_CTL, (uint8_t) ACCEL_PWR_MEASURE))
			{
			if ((! MyAccel->Write((uint8_t) ACCEL_BW_RATE, (uint8_t) ACCEL_RATE_50Hz))
				&& (! MyAccel->Write((uint8_t) ACCEL_INT_ENA, (uint8_t) ACCEL_DATA_RDY))
				&& (! MyAccel->Write((uint8_t) ACCEL_DATA_FMT, (uint8_t) (ACCEL_FULL_RES | ACCEL_2g))))
				//&& (! MyAccel->Write((uint8_t) ACCEL_DATA_FMT, (uint8_t) ACCEL_16g)))
				//&& (! MyAccel->Write((uint8_t) ACCEL_FIFO_CTL, (uint8_t) (ACCEL_FIFO_STREAM | ACCEL_FIFO_SAMPLES))))
				{
				initcnt++;
//				MyAccel->Read((uint8_t) ACCEL_BW_RATE, 1, &bw_rate);
//				SmartDashboard::PutNumber("ACCEL_BW_RATE", bw_rate & 0x1F);
				}
			}
		}
	XaccelCenter = (SmartDashboard::GetNumber("XaccelCenter",ACCEL_XCENTER) / 1000);
	YaccelCenter = (SmartDashboard::GetNumber("YaccelCenter",ACCEL_YCENTER) / 1000);
	ZaccelCenter = (SmartDashboard::GetNumber("ZaccelCenter",ACCEL_ZCENTER) / 1000);
	Xdeviation = SmartDashboard::GetNumber("Xalign",ACCEL_XALIGN);
	Ydeviation = SmartDashboard::GetNumber("Yalign",ACCEL_XALIGN);
	Xsensitivity = SmartDashboard::GetNumber("Xsensitivity",ACCEL_XSENSITIVITY);
	Ysensitivity = SmartDashboard::GetNumber("Ysensitivity",ACCEL_YSENSITIVITY);
	FrictionEffect = SmartDashboard::GetNumber("FrictionEffect",ACCEL_FRICTION);
	SmartDashboard::PutNumber("XaccelCenter", XaccelCenter);
	SmartDashboard::PutNumber("YaccelCenter", YaccelCenter);
	SmartDashboard::PutNumber("ZaccelCenter", ZaccelCenter);
	SmartDashboard::PutNumber("Xsensitivity", Xsensitivity);
	SmartDashboard::PutNumber("Ysensitivity", Ysensitivity);
	SmartDashboard::PutNumber("Xalign", Xdeviation);
	SmartDashboard::PutNumber("Yalign", Ydeviation);
	SmartDashboard::PutNumber("FrictionEffect",FrictionEffect);
	}
#endif


#ifdef MPU6050_6DOF
void NAVIGATOR :: InitMyGyro()
	{
	// Configure and initialize the gyro
	if (! MyGyro->AddressOnly())		// if the gyro is present, initialize its registers
		{
		initcnt++;
		if ((! MyGyro->Write((uint8_t) DOF6_Reg_PWR_MGT1, (uint8_t) DOF6_PLL_Z))
				&& (! MyGyro->Write((uint8_t) DOF6_Reg_MOT_CTL, (uint8_t) DOF6_PON_DELAY))
				&& (! MyGyro->Write((uint8_t) DOF6_Reg_CONFIG, (uint8_t) DOF6_8kHz))
				&& (! MyGyro->Write((uint8_t) DOF6_Reg_GCONFIG, (uint8_t) DOF6_2000dps))
				&& (! MyGyro->Write((uint8_t) DOF6_Reg_ACONFIG, (uint8_t) DOF6_2g))
				&& (! MyGyro->Write((uint8_t) DOF6_Reg_SMPLRT_DIV, (uint8_t) DOF6_SampleDiv8))
				&& (! MyGyro->Write((uint8_t) DOF6_Reg_INT_CFG, (uint8_t) DOF6_Latch))
				&& (! MyGyro->Write((uint8_t) DOF6_Reg_INT_ENA, (uint8_t) DOF6_DataRdy)))
			{
			initcnt++;
#ifdef MPU6050_USING_FIFO
			if ((! MyGyro->Write((uint8_t) DOF6_Reg_FIFO_EN, (uint8_t) (DOF6_FIFO_GX | DOF6_FIFO_GY | DOF6_FIFO_GZ | DOF6_FIFO_ACC)))
					&& (! MyGyro->Write((uint8_t) DOF6_Reg_USER_CTL, (uint8_t) DOF6_FIFO_EN))
					&& (! MyGyro->Write((uint8_t) DOF6_Reg_USER_CTL, (uint8_t) DOF6_FIFO_RST))
					&& (! MyGyro->Write((uint8_t) DOF6_Reg_USER_CTL, (uint8_t) DOF6_FIFO_EN)))
				{
				initcnt++;
				}
#endif
			}
/*
 * Diagnostics to read and display gyroscope initialization parameters

		MyGyro->Read(DOF6_Reg_MOT_CTL, DOF6_READ_DATA_LEN, DOF6_rcv_buff); //read the fifo queue count to see how much data is on the queue
		byte motctl = DOF6_rcv_buff[0];
		byte usrctl = DOF6_rcv_buff[1];
		byte pwrmgt1 = DOF6_rcv_buff[2];
		byte pwrmgt2 = DOF6_rcv_buff[3];
		short fifocnt = (((short)DOF6_rcv_buff[4]) << 8) | ((short)DOF6_rcv_buff[5]);
		SmartDashboard::PutNumber("motctl", motctl);
		SmartDashboard::PutNumber("usrctl", usrctl);
		SmartDashboard::PutNumber("pwrmgt1", pwrmgt1);
		SmartDashboard::PutNumber("pwrmgt2", pwrmgt2);
		SmartDashboard::PutNumber("fifocnt", fifocnt);

		MyGyro->Read(DOF6_Reg_SMPLRT_DIV, DOF6_READ_DATA_LEN, DOF6_rcv_buff); //read the fifo queue count to see how much data is on the queue
		byte smplrt = DOF6_rcv_buff[0];
		byte cfg = DOF6_rcv_buff[1];
		byte gcfg = DOF6_rcv_buff[2];
		byte acfg = DOF6_rcv_buff[3];
		byte motdet = DOF6_rcv_buff[4];
		byte fifoen = DOF6_rcv_buff[5];
		SmartDashboard::PutNumber("smplrt", smplrt);
		SmartDashboard::PutNumber("cfg", cfg);
		SmartDashboard::PutNumber("gcfg", gcfg);
		SmartDashboard::PutNumber("acfg", acfg);
		SmartDashboard::PutNumber("motdet", motdet);
		SmartDashboard::PutNumber("fifoen", fifoen);
*/
		}
	}


void NAVIGATOR :: InitMyThermostat()
	{
	// The temperature probe is built into the I2C gyro and was configured with the gyro.
	thermo_sensitivity = (SmartDashboard::GetNumber("Thermo_sensitivity",GYRO_THERMO_SENSITIVITY) / 1000000000);
	temperature = GyroCenter_T = GYRO_THERMO_CENTER;
	SmartDashboard::PutNumber("Thermo_sensitivity",thermo_sensitivity * 1000000000);
	}

void NAVIGATOR :: InitMyAccelerometer()
	{
	// The accelerometer sensors are built into the I2C gyro and were configured with the gyro.
	}

#endif	// #ifdef MPU6050_6DOF


void NAVIGATOR :: SetStartingPosition()
	{
	pMyRobotState->Robot_Position_X = SmartDashboard::GetNumber("Start_X",0);
	pMyRobotState->Robot_Position_Y = SmartDashboard::GetNumber("Start_Y",0);
	pMyRobotState->Robot_Heading = SmartDashboard::GetNumber("Start_Heading",0);
	pMyRobotState->Robot_Direction = pMyRobotState->Robot_Heading;
	pMyRobotState->Drivetrain_Heading = pMyRobotState->Robot_Heading;
	pMyRobotState->Robot_Speed = 0;
	pMyRobotState->Navigation_Robot_Lost = false;
//	SmartDashboard::PutNumber("Start_Heading", pMyRobotState->Robot_Heading);
//	SmartDashboard::PutNumber("Start_X", pMyRobotState->Robot_Position_X);
//	SmartDashboard::PutNumber("Start_Y", pMyRobotState->Robot_Position_Y);
	}

void NAVIGATOR :: Init(RobotStateBuffer *pRobotState, INPUT *pInput)
	{
	SmartDashboard::PutNumber("NavInit", ++initcnt);
	pMyRobotState = pRobotState;
	pMyInput = pInput;
	SetStartingPosition();

	// Get the gyro tuning parameters from the SmartDashboard.
	GyroCenter_Z = GyroInitCenter = (SmartDashboard::GetNumber("GyroInitCenter",GYRO_RATECENTER) / 1000);
	GyroNoiseLimit = SmartDashboard::GetNumber("GyroNoiseLimit",GYRO_NOISELIMIT);
	// Did I mention that some gyros don't give the same
	// voltage shift / dps for clockwise and counter-clockwise.
	// In other words, the voltage shift does not appear to be entirely linear.
	// It can be just a teeny, tiny bit off.  But after 2.5 minutes of competition, it can become significant.
	cw_sensitivity = SmartDashboard::GetNumber("GyroSensitivityCW",GYRO_CW_SENSITIVITY) / 1000000000;
	ccw_sensitivity = SmartDashboard::GetNumber("GyroSensitivityCCW",GYRO_CCW_SENSITIVITY) / 1000000000;
	GyroSensitivity = (cw_sensitivity + ccw_sensitivity) / 2;
	// Did I mention that, if the gyro is not placed exactly level and in the exact center of rotation
	// of the robot, then the above mentioned issue is further exagerated as speed of rotation increases.
	// There is also some amount of cross-axis error if the gyro axes are not perfectly aligned.
	SpinEffectCW = SmartDashboard::GetNumber("SpinEffectCW",GYRO_CW_SPIN_CORRECTION) / 1000000000000;
	SpinEffectCCW = SmartDashboard::GetNumber("SpinEffectCCW",GYRO_CCW_SPIN_CORRECTION) / 1000000000000;
	// There is also a certain amount of noise on the device, which cannot be fully controlled.
	// The best approach is to find a "quiet place" on the robot to limit this noise level.
	// After that, you need to set a noise level, below which everything is considered noise.
	// However, some of that noise may actually be buried signal (very slow turning).
	// So, the trick is to curtail the noise so that is more or less cancels itself out with very
	// little drift effect and capture some of the slow turning signal.
	// These two tuning factors will help.

	GyroXsensitivity = SmartDashboard::GetNumber("GyroXsensitivity",GYRO_X_SENSITIVITY) / 1000;
	GyroYsensitivity = SmartDashboard::GetNumber("GyroYsensitivity",GYRO_Y_SENSITIVITY) / 1000;

	gyro_noise_base = SmartDashboard::GetNumber("GyroNoiseBase",GYRO_NOISE_BASE) / 1000;
	gyro_noise_squelch = SmartDashboard::GetNumber("GyroNoiseSqch",GYRO_NOISE_SQUELCH) / 1000;
	// If these parameters are not currently on the SmartDashboard, the values will default
	// to pre-programmed values.  Write out those values so the operator can change them to
	// whatever they want and then restart the RoboRIO User Program to pick up the new values.
	SmartDashboard::PutNumber("GyroInitCenter",GyroCenter_Z * 1000);
	SmartDashboard::PutNumber("GyroNoiseLimit",GyroNoiseLimit);
	SmartDashboard::PutNumber("GyroSensitivityCW",cw_sensitivity * 1000000000);
	SmartDashboard::PutNumber("GyroSensitivityCCW",ccw_sensitivity * 1000000000);
	SmartDashboard::PutNumber("SpinEffectCW",SpinEffectCW * 1000000000000);
	SmartDashboard::PutNumber("SpinEffectCCW",SpinEffectCCW * 1000000000000);
	SmartDashboard::PutNumber("GyroXsensitivity",GyroXsensitivity * 1000);
	SmartDashboard::PutNumber("GyroYsensitivity",GyroYsensitivity * 1000);
	SmartDashboard::PutNumber("GyroNoiseBase",gyro_noise_base * 1000);
	SmartDashboard::PutNumber("GyroNoiseSqch",gyro_noise_squelch * 1000);
	XaccelCenter = AccelInitCenterX = SmartDashboard::GetNumber("XAinitCenter",ACCEL_XCENTER);
	YaccelCenter = AccelInitCenterY = SmartDashboard::GetNumber("YAinitCenter",ACCEL_YCENTER);
	ZaccelCenter = AccelInitCenterZ = SmartDashboard::GetNumber("ZAinitCenter",ACCEL_ZCENTER);
	Xdeviation = SmartDashboard::GetNumber("Xalign",XWHEEL_ANGLE_OFFSET);
	Ydeviation = SmartDashboard::GetNumber("Yalign",YWHEEL_ANGLE_OFFSET);
	Xsensitivity = SmartDashboard::GetNumber("Xsensitivity",ACCEL_XSENSITIVITY);
	Ysensitivity = SmartDashboard::GetNumber("Ysensitivity",ACCEL_YSENSITIVITY);
	FrictionEffect = SmartDashboard::GetNumber("FrictionEffect",ACCEL_FRICTION);
	accel_noise_base = SmartDashboard::GetNumber("AccelNoiseBase",ACCEL_NOISE_BASE);
	accel_noise_squelch = SmartDashboard::GetNumber("AccelNoiseSqch",ACCEL_NOISE_SQUELCH) / 1000;
	SmartDashboard::PutNumber("XAinitCenter", XaccelCenter);
	SmartDashboard::PutNumber("YAinitCenter", YaccelCenter);
	SmartDashboard::PutNumber("ZAinitCenter", ZaccelCenter);
	SmartDashboard::PutNumber("Xsensitivity", Xsensitivity);
	SmartDashboard::PutNumber("Ysensitivity", Ysensitivity);
	SmartDashboard::PutNumber("Xalign", Xdeviation);
	SmartDashboard::PutNumber("Yalign", Ydeviation);
	SmartDashboard::PutNumber("FrictionEffect",FrictionEffect);
	SmartDashboard::PutNumber("AccelNoiseBase",accel_noise_base);
	SmartDashboard::PutNumber("AccelNoiseSqch",accel_noise_squelch * 1000);

	NavTimer.Reset();
	SmartDashboard::PutNumber("NavInit", ++initcnt);
	InitMyGyro();
	SmartDashboard::PutNumber("NavInit", ++initcnt);
	InitMyThermostat();
	SmartDashboard::PutNumber("NavInit", ++initcnt);
	InitMyAccelerometer();
	SmartDashboard::PutNumber("NavInit", ++initcnt);
//	Xwheel->SetDistancePerPulse(XWHEEL_CM_PER_TICK);
//	Ywheel->SetDistancePerPulse(YWHEEL_CM_PER_TICK);

	NavTimer.Start();
	for (int i=1; (i < 200) && (fabs(NavTimer.Get()) < .000001); i++) Wait (.000001);  // wait for timer to start running

	INS_Calibrating = true;
	INS_semaphore = initializeMutexRecursive();
	//	here we will schedule the Gyro Data Collector interrupt service
	pReadSensors->StartPeriodic(SAMPLEPERIOD);
	//	here we will schedule the Robot Navigation Update interrupt service
	pINSupdate->StartPeriodic(INSUPDATEPERIOD);

	SmartDashboard::PutNumber("NavInit", ++initcnt);
	}


#ifdef ITG3200_GYRO
void NAVIGATOR :: ReadGyro()
	{
//	byte	gyro_data_ready = 1;

//	MyGyro->Read(GYRO_Reg_INT_STATUS, 3, gyro_rcv_buff); //read the interupt status to determine if new data is ready
//	gyro_data_ready = gyro_rcv_buff[0] & GYRO_DataRdy;
//	temperature = ((int) gyro_rcv_buff[1] << 24) | ((int) gyro_rcv_buff[2] << 16) >> 16;
//	if (! gyro_data_ready)
//		{
		// new data is not yet available, we'll catch it next time around and assume nothing bad happened
//		dataNotRdyCnt++;
//		SmartDashboard::PutNumber("DataNotReady", dataNotRdyCnt);
//		return 0;
//		}
//	else
//		{
		MyGyro->Read(GYRO_Reg_XOUT_H, GYRO_READ_CNT, gyro_rcv_buff);	// I mounted the gyro 90 degrees off ?
		y_spin_rate = ((((int) gyro_rcv_buff[0] << 24) | ((int) gyro_rcv_buff[1] << 16)) >> 16) - GyroCenter_Y;
		x_spin_rate = ((((int) gyro_rcv_buff[2] << 24) | ((int) gyro_rcv_buff[3] << 16)) >> 16) - GyroCenter_X;
		z_spin_rate = ((((int) gyro_rcv_buff[4] << 24) | ((int) gyro_rcv_buff[5] << 16)) >> 16) - GyroCenter_Z;

		// Now compensate for the temperature
//		Sensitivity_Offset = 1 + ((GyroCenter_T - NAVtemperature) * thermo_sensitivity);
/* getting super-crazy value for temperature, so disabling thermo compensation for now.
		x_spin_rate *= Sensitivity_Offset;
		y_spin_rate *= Sensitivity_Offset;
		NAVz_spin *= Sensitivity_Offset;
//		SmartDashboard::PutNumber("ThermoComp", (Sensitivity_Offset - 1) * 1000);
//		SmartDashboard::PutNumber("Temperature", temperature);
		*/

		if (INS_Calibrating)
			{
			if (fmax(fmax(fabs(x_spin_rate),fabs(y_spin_rate)),fabs(z_spin_rate)) > GYRO_NOISELIMIT)
//			if (fabs(z_spin_rate) > GYRO_NOISELIMIT)
				{
				//	The robot got moved during the calibration sequence.
				noisyGyrocnt++;
				}
			GyroCenter_X += (double) (10 * x_spin_rate / ((NavElapsedTime * 1000) + 10.0));		// adjust GyroCenter_Z by 0.1% with each pass
			GyroCenter_Y += (double) (10 * y_spin_rate / ((NavElapsedTime * 1000) + 10.0));		// adjust GyroCenter_Z by 0.1% with each pass
			GyroCenter_Z += (double) (10 * z_spin_rate / ((NavElapsedTime * 1000) + 10.0));		// adjust GyroCenter_Z by 0.1% with each pass
			}

		if ((z_spin_rate * cw_sensitivity) > 0)	// going clockwise
			{
			z_spin_rate = z_spin_rate * (cw_sensitivity + (z_spin_rate * SpinEffectCW));		// calculate z_spin_rate
			}
		else if ((z_spin_rate * cw_sensitivity) < 0)	// going counter-clockwise
			{
			z_spin_rate = z_spin_rate * (ccw_sensitivity + (z_spin_rate * SpinEffectCCW));	// calculate z_spin_rate
			}
		x_spin_rate *= GyroSensitivity;		// calculate z_spin_rate
		y_spin_rate *= GyroSensitivity;		// calculate z_spin_rate

		// Chop noise
		x_spin_rate *= (fabs(x_spin_rate) < gyro_noise_base) ? gyro_noise_squelch : 1;
		y_spin_rate *= (fabs(y_spin_rate) < gyro_noise_base) ? gyro_noise_squelch : 1;
		z_spin_rate *= (fabs(z_spin_rate) < gyro_noise_base) ? gyro_noise_squelch : 1;
//		}
	}

void NAVIGATOR :: ReadThermostat()
	{
	MyGyro->Read(GYRO_Reg_TEMP_OUT_H, 2, gyro_rcv_buff);
	temperature = (((short) gyro_rcv_buff[0]) << 8) | ((short) gyro_rcv_buff[1]);
	}

#endif	//	#ifdef ITG3200_GYRO


#ifdef ADXL345_ACCEL
void NAVIGATOR :: ReadAccel()
	{

	byte	accel_data_ready = 1;
//	byte	accel_fifo_queue_depth = 0;

	MyAccel->Read(ACCEL_INT_SRC, 1, accel_rcv_buff); //read the interupt status to determine if new data is ready
	accel_data_ready = accel_rcv_buff[0] & ACCEL_DATA_RDY;
	if (! accel_data_ready)
		{
		// new data is not yet available, we'll catch it next time around and assume nothing bad happened
		dataNotRdyCnt++;
//		SmartDashboard::PutNumber("DataNotReady", dataNotRdyCnt);
		}
//	MyAccel->Read(ACCEL_FIFO_STAT, 1, &accel_fifo_queue_depth); //read the acceleration data from the ADXL345
//	accel_fifo_queue_depth = (accel_fifo_queue_depth & ACCEL_FIFO_QUE_MSK) - 1;	// we may need to come get more readings

//	if (accel_fifo_queue_depth >= 0)	//note: FIFO data count not working.  So grab data even when count is 0
//	else
		{
//		if (accel_fifo_queue_depth > 0) SmartDashboard::PutNumber("AccelQcount",accel_fifo_queue_depth);
		MyAccel->Read(ACCEL_X0_DATA, ACCEL_READ_CNT, accel_rcv_buff);
		//read the acceleration data from the ADXL345
		// each axis reading comes in 2 bytes.  Least Significant Byte first!!
		// thus we are converting both bytes in to one short int

		/*
		 * Alternative mode is to use the FIFO queue on the chip.  One issue with this is
		 * if we are using an analog gyro, we only get one composite value each time we
		 * read the sensor inputs.  So, we would need to create a composite value from
		 * all entries in the FIFO queue.  For example, if there were 2 entries in the
		 * FIFO queue (a1 and a2), then the formula for the composite value would be
		 * a = (0.75 * a1) + (0.25 * a2).  I have not yet worked out the general formula
		 * for n entries in the queue.
		 *
		 * So, we are setting the accelerometer to sample at 1600Hz and queue the samples
		 * into a FIFO queue.  We will then read the data at 1ms intervals.  This will
		 * generally produce 2 samples per cycles, but occassionally only 1 sample.
		 * So, the max samples we will read per cycle is 2 and the formula above will
		 * appropriately average those samples.
		 */

		// Little endien (per documentation)
		x_accel = ((((int) accel_rcv_buff[1] << 24) | ((int) accel_rcv_buff[0] << 16)) >> 16) - XaccelCenter;
		y_accel = ((((int) accel_rcv_buff[3] << 24) | ((int) accel_rcv_buff[2] << 16)) >> 16) - YaccelCenter;
		z_accel = ((((int) accel_rcv_buff[5] << 24) | ((int) accel_rcv_buff[4] << 16)) >> 16) - ZaccelCenter;

		// Chop noise
		x_accel *= (fabs(x_accel) < accel_noise_base) ? accel_noise_squelch : 1;
		y_accel *= (fabs(y_accel) < accel_noise_base) ? accel_noise_squelch : 1;
		z_accel *= (fabs(z_accel) < accel_noise_base) ? accel_noise_squelch : 1;

		// Now compensate for the temperature
//		Sensitivity_Offset = 1 + ((GyroCenter_T - NAVtemperature) * thermo_sensitivity);
/* getting super-crazy value for temperature, so disabling thermo compensation for now.
		x_spin_rate *= Sensitivity_Offset;
		y_spin_rate *= Sensitivity_Offset;
		NAVz_spin *= Sensitivity_Offset;
//		SmartDashboard::PutNumber("ThermoComp", (Sensitivity_Offset - 1) * 1000);
//		SmartDashboard::PutNumber("Temperature", temperature);
		*/

		if (INS_Calibrating)
			{
			if (fabs(x_accel) + fabs(y_accel) + fabs(z_accel) > ACCEL_NOISELIMIT)
				{
				//	The robot got moved during the calibration sequence.
				noisyAccelcnt++;
				}
			XaccelCenter += (double) (10 * x_accel / ((NavElapsedTime * 1000) + 10.0));
			YaccelCenter += (double) (10 * y_accel / ((NavElapsedTime * 1000) + 10.0));
			ZaccelCenter += (double) (10 * z_accel / ((NavElapsedTime * 1000) + 10.0));
			}
		}
	}
#endif	//	#ifdef ADXL345_ACCEL



#ifdef BUILTIN_ACCEL
void NAVIGATOR :: ReadAccel()
	{
	x_accel = MyAccel->GetX() - XaccelCenter;
	y_accel = MyAccel->GetY() - YaccelCenter;
	z_accel = MyAccel->GetZ() - ZaccelCenter;

	// Chop noise
	x_accel *= (fabs(x_accel) < accel_noise_base) ? accel_noise_squelch : 1;
	y_accel *= (fabs(y_accel) < accel_noise_base) ? accel_noise_squelch : 1;
	z_accel *= (fabs(z_accel) < accel_noise_base) ? accel_noise_squelch : 1;

	// Now compensate for the temperature
//		Sensitivity_Offset = 1 + ((GyroCenter_T - NAVtemperature) * thermo_sensitivity);
/* getting super-crazy value for temperature, so disabling thermo compensation for now.
	x_spin_rate *= Sensitivity_Offset;
	y_spin_rate *= Sensitivity_Offset;
	NAVz_spin *= Sensitivity_Offset;
//		SmartDashboard::PutNumber("ThermoComp", (Sensitivity_Offset - 1) * 1000);
//		SmartDashboard::PutNumber("Temperature", temperature);
	*/

	if (INS_Calibrating)
		{
		if (fabs(x_accel) + fabs(y_accel) + fabs(z_accel) > ACCEL_NOISELIMIT)
			{
			//	The robot got moved during the calibration sequence.
			noisyAccelcnt++;
			}
			XaccelCenter += (double) (10 * x_accel / ((NavElapsedTime * 1000) + 10.0));
			YaccelCenter += (double) (10 * y_accel / ((NavElapsedTime * 1000) + 10.0));
			ZaccelCenter += (double) (10 * z_accel / ((NavElapsedTime * 1000) + 10.0));
		}
//	return 0;
	}
#endif	//	#ifdef BUILTIN_ACCEL


#ifndef MPU6050_6DOF
int NAVIGATOR :: CheckFIFOcnt()
	{
	return int(fifo_queue_cnt++) % 2;
	}


#endif	//ifndef MPU6050_6DOF

#ifdef MPU6050_6DOF
int NAVIGATOR :: CheckFIFOcnt()
	{
	#ifdef MPU6050_USING_FIFO
	short	fifo_queue_cnt = 0;

	gyro_frame_index = 6;
	accel_frame_index = 0;
	MyGyro->Read(DOF6_Reg_FIFO_CNT_H, DOF6_READ_FIFO_CNT, DOF6_rcv_buff); //read the fifo queue count to see how much data is on the queue
	fifo_queue_cnt = (((short)DOF6_rcv_buff[0]) << 8) | ((short)DOF6_rcv_buff[1]);
	if (fifo_queue_cnt < DOF6_FRAME_SIZE)
		{
		// new data is not yet available, we'll catch it next time around and assume nothing bad happened
		dataNotRdyCnt++;
		SmartDashboard::PutNumber("DataNotReady", dataNotRdyCnt);
		}
	else
		{
//		MyGyro->Read(DOF6_Reg_FIFO_CNT_H, DOF6_FRAME_SIZE, DOF6_rcv_buff); //read one data frame from the FIFO queue register
		MyGyro->Read(DOF6_Reg_FIFO_CNT_H, fifo_queue_cnt, DOF6_rcv_buff); //read all data frames from the FIFO queue register
		}
	SmartDashboard::PutNumber("fifo_queue_cnt", fifo_queue_cnt);
	return int(fifo_queue_cnt / 12);	// this tells how many data frames are on the queue
	#endif

	#ifndef MPU6050_USING_FIFO
//	int		data_ready = 0;

	gyro_frame_index = 6;
	accel_frame_index = 0;
//	MyGyro->Read(DOF6_Reg_Accel_XH, DOF6_FRAME_SIZE, &DOF6_rcv_buff[accel_frame_index]); //read the interupt status and all sensor data in burst mode
	MyGyro->Read(DOF6_Reg_GYRO_XH, DOF6_FRAME_SIZE, &DOF6_rcv_buff[gyro_frame_index]); //read the interupt status and all sensor data in burst mode
//	data_ready = DOF6_rcv_buff[0] & DOF6_DataRdy;
//	SmartDashboard::PutNumber("DataReady", int(DOF6_rcv_buff[0]));
//	data_ready = 1;
//	if (! data_ready)
//		{	// new data is not yet available, we'll catch it next time around and assume nothing bad happened
//		dataNotRdyCnt++;
//		SmartDashboard::PutNumber("DataNotReady", dataNotRdyCnt);
//		}
	return	(1);		// int(data_ready);
	#endif
	}


void NAVIGATOR :: ReadGyro()
	{
	// Big endien (per documentation)
#ifdef REALBOT
	x_spin_rate = ((((int) DOF6_rcv_buff[gyro_frame_index] << 24) | ((int) DOF6_rcv_buff[gyro_frame_index + 1] << 16)) >> 16) - GyroCenter_X;
	y_spin_rate = ((((int) DOF6_rcv_buff[gyro_frame_index + 2] << 24) | ((int) DOF6_rcv_buff[gyro_frame_index + 3] << 16)) >> 16) - GyroCenter_Y;
	z_spin_rate = ((((int) DOF6_rcv_buff[gyro_frame_index + 4] << 24) | ((int) DOF6_rcv_buff[gyro_frame_index + 5] << 16)) >> 16) - GyroCenter_Z;
#endif
#ifdef PRACTICEBOT
	//	Sideways orientation;  Cheap $3.00 Gyro.  Z-axis is Fubar.  Mounting gyro sideways and using y-axis instead
	x_spin_rate = ((((int) DOF6_rcv_buff[gyro_frame_index] << 24) | ((int) DOF6_rcv_buff[gyro_frame_index + 1] << 16)) >> 16) - GyroCenter_X;
	z_spin_rate = ((((int) DOF6_rcv_buff[gyro_frame_index + 2] << 24) | ((int) DOF6_rcv_buff[gyro_frame_index + 3] << 16)) >> 16) - GyroCenter_Z;
	y_spin_rate = ((((int) DOF6_rcv_buff[gyro_frame_index + 4] << 24) | ((int) DOF6_rcv_buff[gyro_frame_index + 5] << 16)) >> 16) - GyroCenter_Y;
#endif
	gyro_frame_index += 12;

	// Now compensate for the temperature
//		Sensitivity_Offset = 1 + ((GyroCenter_T - NAVtemperature) * thermo_sensitivity);
/* getting super-crazy value for temperature, so disabling thermo compensation for now.
	x_spin_rate *= Sensitivity_Offset;
	y_spin_rate *= Sensitivity_Offset;
	NAVz_spin *= Sensitivity_Offset;
//		SmartDashboard::PutNumber("ThermoComp", (Sensitivity_Offset - 1) * 1000);
//		SmartDashboard::PutNumber("Temperature", temperature);
	*/

	if (INS_Calibrating)
		{
		if (fmax(fmax(fabs(x_spin_rate),fabs(y_spin_rate)),fabs(z_spin_rate)) > GYRO_NOISELIMIT)
//			if (fabs(z_spin_rate) > GYRO_NOISELIMIT)
			{
			//	The robot got moved during the calibration sequence.
			noisyGyrocnt++;
			}
		GyroCenter_X += (double) (10 * x_spin_rate / ((NavElapsedTime * 1000) + 10.0));		// adjust GyroCenter_Z by 0.1% with each pass
		GyroCenter_Y += (double) (10 * y_spin_rate / ((NavElapsedTime * 1000) + 10.0));		// adjust GyroCenter_Z by 0.1% with each pass
		GyroCenter_Z += (double) (10 * z_spin_rate / ((NavElapsedTime * 1000) + 10.0));		// adjust GyroCenter_Z by 0.1% with each pass
		}

	if ((z_spin_rate * cw_sensitivity) > 0)	// going clockwise
		{
		z_spin_rate = z_spin_rate * (cw_sensitivity + (z_spin_rate * SpinEffectCW));		// calculate z_spin_rate
		}
	else if ((z_spin_rate * cw_sensitivity) < 0)	// going counter-clockwise
		{
		z_spin_rate = z_spin_rate * (ccw_sensitivity + (z_spin_rate * SpinEffectCCW));	// calculate z_spin_rate
		}

	// Chop noise
	x_spin_rate *= (fabs(x_spin_rate) < gyro_noise_base) ? gyro_noise_squelch : 1;
	y_spin_rate *= (fabs(y_spin_rate) < gyro_noise_base) ? gyro_noise_squelch : 1;
	z_spin_rate *= (fabs(z_spin_rate) < gyro_noise_base) ? gyro_noise_squelch : 1;
	}


void NAVIGATOR :: ReadAccel()
	{
//	MyGyro->Read(DOF6_Reg_FIFO_CNT_H, DOF6_READ_DATA_LEN, DOF6_rcv_buff); //read the fifo queue count to see how much data is on the queue

	// Big endien (per documentation)
	x_accel = ((((int) DOF6_rcv_buff[accel_frame_index] << 24) | ((int) DOF6_rcv_buff[accel_frame_index + 1] << 16)) >> 16) - XaccelCenter;
	y_accel = ((((int) DOF6_rcv_buff[accel_frame_index + 2] << 24) | ((int) DOF6_rcv_buff[accel_frame_index + 3] << 16)) >> 16) - YaccelCenter;
	z_accel = ((((int) DOF6_rcv_buff[accel_frame_index + 4] << 24) | ((int) DOF6_rcv_buff[accel_frame_index + 5] << 16)) >> 16) - ZaccelCenter;
	accel_frame_index += 12;

	x_accel -= (ZaccelCenter * sin(XtipAngle * DEGREES_TO_RADIANS));  // remove gravity component due to axis misalignment and robot tipping
	y_accel -= (ZaccelCenter * sin(YtipAngle * DEGREES_TO_RADIANS));
	x_accel *= (cos(XtipAngle * DEGREES_TO_RADIANS));	// and re-orient to a horizontal plane
	y_accel *= (cos(YtipAngle * DEGREES_TO_RADIANS));

	if (INS_Calibrating)
		{
		if (fabs(x_accel) + fabs(y_accel) + fabs(z_accel) > ACCEL_NOISELIMIT)
			{
			//	The robot got moved during the calibration sequence.
			noisyAccelcnt++;
			}
		XaccelCenter += (double) (10 * x_accel / ((NavElapsedTime * 1000) + 10.0));
		YaccelCenter += (double) (10 * y_accel / ((NavElapsedTime * 1000) + 10.0));
		ZaccelCenter += (double) (10 * z_accel / ((NavElapsedTime * 1000) + 10.0));
		}

	// Chop noise
	x_accel *= (fabs(x_accel) < accel_noise_base) ? accel_noise_squelch : 1;
	y_accel *= (fabs(y_accel) < accel_noise_base) ? accel_noise_squelch : 1;
	z_accel *= (fabs(z_accel) < accel_noise_base) ? accel_noise_squelch : 1;

	// Now compensate for the temperature
//		Sensitivity_Offset = 1 + ((GyroCenter_T - NAVtemperature) * thermo_sensitivity);
/* getting super-crazy value for temperature, so disabling thermo compensation for now.
	x_spin_rate *= Sensitivity_Offset;
	y_spin_rate *= Sensitivity_Offset;
	NAVz_spin *= Sensitivity_Offset;
//		SmartDashboard::PutNumber("ThermoComp", (Sensitivity_Offset - 1) * 1000);
//		SmartDashboard::PutNumber("Temperature", temperature);
	*/

	}

#endif	//ifdef MPU6050_6DOF


void NAVIGATOR :: ReadWheelEncoders()
	{
#ifdef XYWHEELS
	// read the X and Y wheel encoders, convert to centimeters and only count the change.
	int x_ticks = Xwheel->Get();
	int y_ticks = Ywheel->Get();
	double x_dist = ((double) x_ticks) * (x_ticks > 0 ? XWHEEL_IN_PER_TICK_FWD : XWHEEL_IN_PER_TICK_REV);
	double y_dist = ((double) y_ticks) * (y_ticks > 0 ? YWHEEL_IN_PER_TICK_FWD : YWHEEL_IN_PER_TICK_REV);
	/*
	 * On error, the encoder Get method returns a zero.  That would be bad.
	 * So, if the Get method returns a zero, maintain the previous value (i.e. no movement)
	 */
	x_wheel = (x_ticks == 0) ? 0 : x_dist - prev_x_wheel;
	y_wheel = (y_ticks == 0) ? 0 : y_dist - prev_y_wheel;
	prev_x_wheel += x_wheel;
	prev_y_wheel += y_wheel;
	if (sample_counter % 20 == 0)
		{
//		SmartDashboard::PutNumber("Xdist", x_dist);
//		SmartDashboard::PutNumber("Ydist", y_dist);
		SmartDashboard::PutNumber("Xdist", prev_x_wheel);
		SmartDashboard::PutNumber("Ydist", prev_y_wheel);
		}
#endif
	}

void NAVIGATOR :: Push_Navigation_Data_Frame(double timestamp, double x_gyro, double y_gyro, double z_gyro, double x_accel, double y_accel, double z_accel, double x_wheel, double y_wheel)
	{
	int queue_depth;
	CRITICAL_REGION(INS_semaphore)		// we need to reference values that are updated outside of this procedure
		{								// and we don't want those values getting changed while we are referencing them
		queue_depth = (NAVIGATION_QUEUE_SIZE + Navigation_Data_Queue_Top - Navigation_Data_Queue_Bottom) % NAVIGATION_QUEUE_SIZE;
		}
	END_REGION;
	if (queue_depth++ >= NAVIGATION_QUEUE_SIZE - 1)
		{
		// Queue is full and we are lost;
		// If in Autonomous, stop all motors
		pMyRobotState->Navigation_Robot_Lost = true;
		SmartDashboard::PutBoolean("NavOverflow",true);
		Wait(5);
		}
	else
		{
		if (Navigation_Data_Queue_Top++ >= NAVIGATION_QUEUE_SIZE - 1)
			{
			Synchronized sync(INS_semaphore);	// we are going to update values referenced by other procedure.
			Navigation_Data_Queue_Top = 0;
			}
		Navigation_Data_Queue[Navigation_Data_Queue_Top].timestamp = timestamp;
		Navigation_Data_Queue[Navigation_Data_Queue_Top].gyro_x = x_gyro;
		Navigation_Data_Queue[Navigation_Data_Queue_Top].gyro_y = y_gyro;
		Navigation_Data_Queue[Navigation_Data_Queue_Top].gyro_z = z_gyro;
		Navigation_Data_Queue[Navigation_Data_Queue_Top].accel_x = x_accel;
		Navigation_Data_Queue[Navigation_Data_Queue_Top].accel_y = y_accel;
		Navigation_Data_Queue[Navigation_Data_Queue_Top].accel_z = z_accel;
		Navigation_Data_Queue[Navigation_Data_Queue_Top].wheel_x = x_wheel;
		Navigation_Data_Queue[Navigation_Data_Queue_Top].wheel_y = y_wheel;
			{
			Synchronized sync(INS_semaphore);	// we are going to update values referenced by other procedure.
			Navigation_Data_Queue_Depth++;
			}
		}
	}


void NAVIGATOR :: UpdateTipAngle()
	{
	/*
	 * Integrate spin rates to give tip angles and convert to radians
	 *  Since the chip is not
	 * mounted perfectly level, and the robot's frame is not perfectly level, and the floor is not perfectly level, any
	 * x and y accelerations that we measure will have a component of gravity added to them.  We need to remove that component
	 * in order to realize how much of the x and y measurements are actually associated with x and y movements.
	 *
	 * Also, when we pick up an off-center load, it will cause the robot to tip a teeny-tiny bit.  But a eensy-weensy, teeny-tiny,
	 * little-bitty tip will add a gravitational component to the x and y acceleration values, which will very quickly double-integrate
	 * to a rather large movement error.  Note also, that since the drivetrain imparts a force on the robot below its center of mass,
	 * the robot will always tip whenever the drivetrain changes power or direction.  And this tipping will also be measured on the
	 * x and y accelerometer sensors and will tend to accentuate the amount of acceleration read by the sensors.  This would be a
	 * "wash" if we always started and stopped at the same rate of acceleration.  But of course, we do not.  So we need to remove
	 * the gravitational component associated with the robot tipping.
	 *
	 * We do this by constantly tracking the amount of x and y tipping with a gyroscope.  We have a known starting point for the
	 * tip angle that is dependent on the alignment error of the chip relative to the robot.  The value of the x and y alignment
	 * errors is specific to each robot and each accelerometer chip.  It can only be set through repeated trials and measurements
	 * after the robot is assembled and accelerometer permanently attached.  Any changes to the robot structure or sensor placement
	 * will require recalibration.  This calibration needs to be within about 5/1000 of a degree in order to perform adequately
	 * for the duration of the autonomous period.
	 */
	if (! INS_Calibrating)
		{
		XtipAngle += y_spin_rate * GyroYsensitivity;
		YtipAngle += x_spin_rate * GyroXsensitivity;
		}
	}


void NAVIGATOR :: ReadSensors()
	{
	int	data_frame_count = 0;

	NavClockTime = NavTimer.Get();

	do	{
		data_frame_count = CheckFIFOcnt();
		ReadGyro();
		ReadAccel();
		ReadWheelEncoders();
		pMyRobotState->Robot_Spin_Rate = z_spin_rate;
//		The next two lines are commented out because this is done in the UpdateHeading method
//		pMyRobotState->Robot_Heading += z_spin_rate;
//		pMyRobotState->Robot_Heading += pMyRobotState->Robot_Heading > 360 ? -360 : pMyRobotState->Robot_Heading < 0 ? 360 : 0;

		UpdateTipAngle();

		Push_Navigation_Data_Frame(NavClockTime, x_spin_rate, y_spin_rate, z_spin_rate, x_accel, y_accel, z_accel, x_wheel, y_wheel);
		if (sample_counter++ % 20 == 0)
			{
	//		SmartDashboard::PutNumber("NoisyGyroCount",noisyGyrocnt);
	//		SmartDashboard::PutNumber("DataNotReady", dataNotRdyCnt);
			SmartDashboard::PutNumber("GyroCenter_Z", (GyroCenter_Z - GyroInitCenter));
			SmartDashboard::PutNumber("RawGyro", z_spin_rate);
			SmartDashboard::PutBoolean("GyroCalibrating",INS_Calibrating);
			SmartDashboard::PutNumber("SampleRunTime", SampleRunTime);
			SmartDashboard::PutNumber("SampleLoopTime", (NavClockTime - SampleElapsedTime) * 50);
			SampleElapsedTime = NavClockTime;
	//		SmartDashboard::PutNumber("AccumValue", accumulator);
	//		SmartDashboard::PutNumber("AccumCount", gyro_accumulator_count);
	//		//double t = GyroCenter_T - (double) GYRO_THERMO_CENTER;
	//		SmartDashboard::PutNumber("Temperature", GyroCenter_T);
	//		SmartDashboard::PutNumber("Temperature", temperature);
			SmartDashboard::PutNumber("X Axis", x_accel);
			SmartDashboard::PutNumber("Y Axis", y_accel);
			SmartDashboard::PutNumber("Z Axis", z_accel);
			SmartDashboard::PutNumber("X Tip", XtipAngle);
			SmartDashboard::PutNumber("Y Tip", YtipAngle);
	//		SmartDashboard::PutNumber("NoisyAccelCount",noisyAccelcnt);
			SmartDashboard::PutNumber("XaccelCenter", XaccelCenter - AccelInitCenterX);
			SmartDashboard::PutNumber("YaccelCenter", YaccelCenter - AccelInitCenterY);
			SmartDashboard::PutNumber("ZaccelCenter", ZaccelCenter - AccelInitCenterZ);
	//		SmartDashboard::PutBoolean("AccelCalibrating",INS_Calibrating);
			}
		} while (data_frame_count > 1);

	SampleRunTime = (NavTimer.Get() - NavClockTime) * 1000;	// exec time of this method in msec

	}


//  schedule this to run independently every 0.1 milliseconds
void NAVIGATOR :: CallReadSensors(void *controller)
	{
	NAVIGATOR *sample_sensors = (NAVIGATOR*) controller;
	sample_sensors->ReadSensors();
	}


void NAVIGATOR :: Zero()
	{
//	CalibrateGyro->Stop();				//	Stop the Cage scheduler;
	SmartDashboard::PutNumber("NavInit", ++initcnt);
	if (INS_Calibrating)
		{
		Synchronized sync(INS_semaphore);	// we are going to update values referenced by other procedure.
		INS_Calibrating = false;
		SetStartingPosition();
		GyroHeading = pMyRobotState->Robot_Heading;	// This should be specified on the dashboard next to the autonomous program selection
		Navigation_Data_Queue_Bottom = Navigation_Data_Queue_Top;  // Clear the data input queue
		Navigation_Status_Buffer.Xcoord = pMyRobotState->Robot_Position_X;
		Navigation_Status_Buffer.Ycoord = pMyRobotState->Robot_Position_Y;
		Navigation_Status_Buffer.Xspeed = 0;
		Navigation_Status_Buffer.Yspeed = 0;
		Navigation_Status_Buffer.Xaccel = 0;
		Navigation_Status_Buffer.Yaccel = 0;
		NavTimer.Reset();					// either reset the NavTimer, or update the NavStartTime
		//NavStartTime = NavTimer.Get();	// I chose to reset the NavTimer, which keeps the math very simple
		Navigation_Status_Buffer.Timestamp = NavTimer.Get();
//		Navigation_Status_Buffer.Timestamp = sample_counter = 0;
		Push_Navigation_Data_Frame(Navigation_Status_Buffer.Timestamp, x_spin_rate, y_spin_rate, z_spin_rate, x_accel, y_accel, z_accel, x_wheel, y_wheel);
		}
	SmartDashboard::PutNumber("NavInit", ++initcnt);
	}


bool NAVIGATOR :: Pop_Navigation_Data_Frame()
	{
	int queue_depth;
//	double noiselevelfilter = 25;
	CRITICAL_REGION(INS_semaphore)		// we need to reference values that are updated outside of this procedure
		{								// and we don't want those values getting changed while we are referencing them
		queue_depth = (NAVIGATION_QUEUE_SIZE + Navigation_Data_Queue_Top - Navigation_Data_Queue_Bottom) % NAVIGATION_QUEUE_SIZE;
		}
	END_REGION;
	if (queue_depth <= 1) return false;
	if (Navigation_Data_Queue_Bottom++ >= NAVIGATION_QUEUE_SIZE - 1)
		{
		Synchronized sync(INS_semaphore);	// we are going to update values referenced by other procedure.
		Navigation_Data_Queue_Bottom = 0;
		}
	NavElapsedTime = Navigation_Data_Queue[Navigation_Data_Queue_Bottom].timestamp;
//	NAVtemperature = Navigation_Data_Queue[Navigation_Data_Queue_Bottom].temperature;
	NAVx_spin = Navigation_Data_Queue[Navigation_Data_Queue_Bottom].gyro_x;
	NAVy_spin = Navigation_Data_Queue[Navigation_Data_Queue_Bottom].gyro_y;
	NAVz_spin = Navigation_Data_Queue[Navigation_Data_Queue_Bottom].gyro_z;
	NAVx_accel = (double) Navigation_Data_Queue[Navigation_Data_Queue_Bottom].accel_x;
	NAVy_accel = (double) Navigation_Data_Queue[Navigation_Data_Queue_Bottom].accel_y;
	NAVz_accel = (double) Navigation_Data_Queue[Navigation_Data_Queue_Bottom].accel_z;
	NAVx_wheel = (double) Navigation_Data_Queue[Navigation_Data_Queue_Bottom].wheel_x;
	NAVy_wheel = (double) Navigation_Data_Queue[Navigation_Data_Queue_Bottom].wheel_y;
		{
		Synchronized sync(INS_semaphore);	// we are going to update values referenced by other procedure.
		Navigation_Data_Queue_Depth--;
		}
	return true;
	}


void NAVIGATOR :: UpdateHeading()
	{
	// NAVspin is already compensated for temperature and converted to degrees, so just add it onto the GyroHeading
	GyroHeading += NAVz_spin;
	GyroHeading += GyroHeading > 360 ? -360 : GyroHeading < 0 ? 360 : 0;
	pMyRobotState->Robot_Heading = GyroHeading;
	if (fabs(NAVz_spin) > GYRO_RATELIMIT)
		{
		pMyRobotState->Navigation_GyroTilt = true;
		}
	}


void NAVIGATOR :: UpdatePosition()
	{
	double TripTime = NavElapsedTime - Navigation_Status_Buffer.Timestamp;
//	SampleElapsedTime = TripTime;
	double x_distance = (cos((GyroHeading + Xdeviation) * DEGREES_TO_RADIANS) * NAVx_wheel) + (sin((GyroHeading + Ydeviation) * DEGREES_TO_RADIANS) * NAVy_wheel);
	double y_distance = (cos((GyroHeading + Ydeviation) * DEGREES_TO_RADIANS) * NAVy_wheel) - (sin((GyroHeading + Xdeviation) * DEGREES_TO_RADIANS) * NAVx_wheel);
	Navigation_Status_Buffer.Xspeed = (x_distance / TripTime);
	Navigation_Status_Buffer.Yspeed = (y_distance / TripTime);
	pMyRobotState->Robot_Position_X += x_distance;
	pMyRobotState->Robot_Position_Y += y_distance;
	Navigation_Status_Buffer.Timestamp = NavElapsedTime;
	Navigation_Status_Buffer.Xcoord = pMyRobotState->Robot_Position_X;
	Navigation_Status_Buffer.Ycoord = pMyRobotState->Robot_Position_Y;
	RobotDirection = (Navigation_Status_Buffer.Xspeed < 0 ? 270 : 90) - (atan(Navigation_Status_Buffer.Yspeed / Navigation_Status_Buffer.Xspeed) / DEGREES_TO_RADIANS);
	RobotDirection += RobotDirection > 360 ? -360 : RobotDirection < 0 ? 360 : 0;
	pMyRobotState->Robot_Direction = RobotDirection;
	pMyRobotState->Robot_Speed = pow((pow(Navigation_Status_Buffer.Xspeed,2) + pow(Navigation_Status_Buffer.Yspeed,2)),0.5);
	}


// schedule this to run independently every 10 milliseconds
void NAVIGATOR :: UpdateINS()
	{
	while (Pop_Navigation_Data_Frame())
		{
		if (NavElapsedTime > (5 * SAMPLEPERIOD))		// throw out the first 5th samples (~0.005 seconds)
			{
			UpdateHeading();
			UpdatePosition();
			if (NavLoopCnt++ % 10 == 0)		// update SmartDashboard every 10 loops
				{
//				SmartDashboard::PutNumber("GyroHeading", GyroHeading); // * NavElapsedTime / (double) NavLoopCnt);
//				SmartDashboard::PutNumber("NavElapsedTime", NavElapsedTime);
//				SmartDashboard::PutNumber("GyroHeading", pMyRobotState->Robot_Heading);
//				SmartDashboard::PutNumber("Xcoord", pMyRobotState->Robot_Position_X);
//				SmartDashboard::PutNumber("Ycoord", pMyRobotState->Robot_Position_Y);
//				SmartDashboard::PutNumber("Direction", pMyRobotState->Robot_Direction);
//				SmartDashboard::PutNumber("Speed", pMyRobotState->Robot_Speed);
//				SmartDashboard::PutNumber("SampleLoopTime", SampleElapsedTime * 1000);
				SmartDashboard::PutNumber("X Deviation", Xdeviation - ACCEL_XALIGN);
				SmartDashboard::PutNumber("Y Deviation", Ydeviation - ACCEL_YALIGN);
				}
			}
		Navigation_Status_Buffer.Timestamp = NavElapsedTime;
		}
	}


//  schedule this to run independently every 10 milliseconds
void NAVIGATOR :: CallUpdateINS(void *controller)
	{
	NAVIGATOR *ins_update = (NAVIGATOR*) controller;
	ins_update->UpdateINS();
	}


