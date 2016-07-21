#include <Navigator.h>

NAVIGATOR :: NAVIGATOR() :	INS_semaphore(0)

	{
	initcnt = 0;
	pMyTargetState = 0;
	pMyInput = 0;
	pReadSensors = new Notifier(NAVIGATOR::CallReadSensors,this);
	pINSupdate = new Notifier(NAVIGATOR::CallUpdateINS,this);
	pMyI2Cdatabuffer = new I2C_RCV_BUFF;							// instantiate the I2C data receive buffer
	memset(pMyI2Cdatabuffer, 0, sizeof(*pMyI2Cdatabuffer));			// initialize all values to 0
	MyIMU = new I2C(I2C::kOnboard, IMU_I2C_ADDR);
	#ifdef XYWHEELS
		pLWheel = new Encoder(XWHEEL_DIO_PORT1, XWHEEL_DIO_PORT2, XWHEEL_REVERSE_DIRECTION, Encoder::k4X);
		pLWheel = new Encoder(YWHEEL_DIO_PORT1, YWHEEL_DIO_PORT2, YWHEEL_REVERSE_DIRECTION, Encoder::k4X);
		NAVx_wheel = NAVy_wheel = prev_x_wheel = prev_y_wheel = 0;
;
	#endif
	#ifdef LRWHEELS
		pLWheel = new Encoder(LWHEEL_DIO_PORT1, LWHEEL_DIO_PORT2, LWHEEL_REVERSE_DIRECTION, Encoder::k4X);
		pRWheel = new Encoder(RWHEEL_DIO_PORT1, RWHEEL_DIO_PORT2, RWHEEL_REVERSE_DIRECTION, Encoder::k4X);
		NAVl_wheel = NAVr_wheel = prev_l_wheel = prev_r_wheel = 0;
	#endif
	INS_Calibrating = false;
	Navigation_Data_Queue_Depth = 0;
	Navigation_Data_Frame_Size = sizeof(Navigation_Data_Queue) / NAVIGATION_QUEUE_SIZE;
	Navigation_Data_Queue_Top = Navigation_Data_Queue_Bottom = 0;
	NavClockTime = NavUpdateTime = DataClockTime = DataCollectTime = 0;
	XtipAngle = YtipAngle = 0;
	XspeedCenter = YspeedCenter = 0;
	temperature = 0;
	NavElapsedTime = NAVtemperature = NAVx_accel = NAVy_accel = NAVz_accel = NAVx_gyro = NAVy_gyro = NAVz_gyro = 0;
	NavLoopCnt = 0;
	SampleCount = 1;
	SampleElapsedTime = 0;
	SampleRunTime = 0;
	GyroHeading = 0;	// actually want to read from SmartDashboard 'cause we might want to start the robot facing otherwise
	RobotDirection = 0;
	noisyGyrocnt = 0;
	noisyAccelcnt = 0;
	dataNotRdyCnt = 0;
//	Sensitivity_Offset = 0;		// we'll use this later to adjust sensitivity as a function of temperature (it does matter)
	gyro_frame_index = 0;
	accel_frame_index = 6;
	GyroCenter_H = GyroCenter_R = GyroCenter_P = 0;
	GyroDrift_H = GyroDrift_R = GyroDrift_P = 0;
	Heading = Roll = Pitch = 0;
	prev_Heading = prev_Roll = prev_Pitch = 0;
	}


void NAVIGATOR :: InitMyIMU()
	{
	// Configure and initialize the gyro
	#ifdef BNO055_9DOF
		if ((! MyIMU->AddressOnly()) && (! MyIMU->Write((uint8_t) BNO055_PAGE_ID_ADDR, 0)))
			{
			for (initcnt=1; (initcnt<90) && (pMyI2Cdatabuffer->CHIP_ID != BNO055_ID); initcnt+=10)	// wait up to 0.9 sec for the IMU Chip to boot up, checking every 1/10 sec
				{
				MyIMU->Read(BNO055_CHIP_ID_ADDR, I2C_RCV_ID_LEN, &pMyI2Cdatabuffer->ID_Buffer);	// read the Device ID data registers
				pMyRobotState->IMU_ChipID = ((int) pMyI2Cdatabuffer->CHIP_ID) & 0xFF;
				Wait(0.1);
				}
			if (pMyI2Cdatabuffer->CHIP_ID == BNO055_ID)							// good news!  The IMU has booted and is responsive, so begin Configuration
				{
				initcnt++;			// = x2			(x = number of 100msec waits for the chipID to show up)
				if (! MyIMU->Write((uint8_t) BNO055_OPR_MODE_ADDR, (uint8_t) BNO055_OPERATION_MODE_CONFIG))
					{
					initcnt++;			// = x3
					Wait(0.03);										// wait 30 msec for the IMU Chip enter Config mode (Chip Specs says 19ms)
					if ((! MyIMU->Write((uint8_t) BNO055_PWR_MODE_ADDR, (uint8_t) BNO055_POWER_MODE_NORMAL))
							&& (! MyIMU->Write((uint8_t) BNO055_AXIS_MAP_ZYX_ADDR, (uint8_t) BNO055_AXIS_MAP_ZYX))
							&& (! MyIMU->Write((uint8_t) BNO055_AXIS_SIGN_XYZ_ADDR, (uint8_t) BNO055_AXIS_SIGN_PPN))
							&& (! MyIMU->Write((uint8_t) BNO055_ACCEL_OFFSET_X_LSB_ADDR, (uint8_t) (pMyRobotState->IMU_Accel_X_Offset & 0xFF)))
							&& (! MyIMU->Write((uint8_t) BNO055_ACCEL_OFFSET_X_MSB_ADDR, (uint8_t) ((pMyRobotState->IMU_Accel_X_Offset >> 8) & 0xFF)))
							&& (! MyIMU->Write((uint8_t) BNO055_ACCEL_OFFSET_Y_LSB_ADDR, (uint8_t) (pMyRobotState->IMU_Accel_Y_Offset & 0xFF)))
							&& (! MyIMU->Write((uint8_t) BNO055_ACCEL_OFFSET_Y_MSB_ADDR, (uint8_t) ((pMyRobotState->IMU_Accel_Y_Offset >> 8) & 0xFF)))
							&& (! MyIMU->Write((uint8_t) BNO055_ACCEL_OFFSET_Z_LSB_ADDR, (uint8_t) (pMyRobotState->IMU_Accel_Z_Offset & 0xFF)))
							&& (! MyIMU->Write((uint8_t) BNO055_ACCEL_OFFSET_Z_MSB_ADDR, (uint8_t) ((pMyRobotState->IMU_Accel_Z_Offset >> 8) & 0xFF)))
							&& (! MyIMU->Write((uint8_t) BNO055_MAG_OFFSET_X_LSB_ADDR, (uint8_t) (pMyRobotState->IMU_Mag_X_Offset & 0xFF)))
							&& (! MyIMU->Write((uint8_t) BNO055_MAG_OFFSET_X_MSB_ADDR, (uint8_t) ((pMyRobotState->IMU_Mag_X_Offset >> 8) & 0xFF)))
							&& (! MyIMU->Write((uint8_t) BNO055_MAG_OFFSET_Y_LSB_ADDR, (uint8_t) (pMyRobotState->IMU_Mag_Y_Offset & 0xFF)))
							&& (! MyIMU->Write((uint8_t) BNO055_MAG_OFFSET_Y_MSB_ADDR, (uint8_t) ((pMyRobotState->IMU_Mag_Y_Offset >> 8) & 0xFF)))
							&& (! MyIMU->Write((uint8_t) BNO055_MAG_OFFSET_Z_LSB_ADDR, (uint8_t) (pMyRobotState->IMU_Mag_Z_Offset & 0xFF)))
							&& (! MyIMU->Write((uint8_t) BNO055_MAG_OFFSET_Z_MSB_ADDR, (uint8_t) ((pMyRobotState->IMU_Mag_Z_Offset >> 8) & 0xFF)))
							&& (! MyIMU->Write((uint8_t) BNO055_GYRO_OFFSET_X_LSB_ADDR, (uint8_t) (pMyRobotState->IMU_Gyro_X_Offset & 0xFF)))
							&& (! MyIMU->Write((uint8_t) BNO055_GYRO_OFFSET_X_MSB_ADDR, (uint8_t) ((pMyRobotState->IMU_Gyro_X_Offset >> 8) & 0xFF)))
							&& (! MyIMU->Write((uint8_t) BNO055_GYRO_OFFSET_Y_LSB_ADDR, (uint8_t) (pMyRobotState->IMU_Gyro_Y_Offset & 0xFF)))
							&& (! MyIMU->Write((uint8_t) BNO055_GYRO_OFFSET_Y_MSB_ADDR, (uint8_t) ((pMyRobotState->IMU_Gyro_Y_Offset >> 8) & 0xFF)))
							&& (! MyIMU->Write((uint8_t) BNO055_GYRO_OFFSET_Z_LSB_ADDR, (uint8_t) (pMyRobotState->IMU_Gyro_Z_Offset & 0xFF)))
							&& (! MyIMU->Write((uint8_t) BNO055_GYRO_OFFSET_Z_MSB_ADDR, (uint8_t) ((pMyRobotState->IMU_Gyro_Z_Offset >> 8) & 0xFF)))
							&& (! MyIMU->Write((uint8_t) BNO055_ACCEL_RADIUS_LSB_ADDR, (uint8_t) (pMyRobotState->IMU_Acc_Radius & 0xFF)))
							&& (! MyIMU->Write((uint8_t) BNO055_ACCEL_RADIUS_MSB_ADDR, (uint8_t) ((pMyRobotState->IMU_Acc_Radius >> 8) & 0xFF)))
							&& (! MyIMU->Write((uint8_t) BNO055_MAG_RADIUS_LSB_ADDR, (uint8_t) (pMyRobotState->IMU_Mag_Radius & 0xFF)))
							&& (! MyIMU->Write((uint8_t) BNO055_MAG_RADIUS_MSB_ADDR, (uint8_t) ((pMyRobotState->IMU_Mag_Radius >> 8) & 0xFF)))
							&& (! MyIMU->Write((uint8_t) BNO055_UNIT_SEL_ADDR, (uint8_t) (BNO055_ORIENTATION_ANDROID |
									BNO055_TEMP_CELSIUS | BNO055_EULER_RADIANS | BNO055_ANGULAR_RATE_RPS | BNO055_ACCEL_MPS2))))
						{
						initcnt++;			// = x4
						Wait(0.01);
//						if (! MyIMU->Write((uint8_t) BNO055_SYS_TRIGGER_ADDR, (uint8_t) (BNO55_TRIGGER_EXTCLK | BNO55_TRIGGER_RSTSYS)))
						if (! MyIMU->Write((uint8_t) BNO055_SYS_TRIGGER_ADDR, (uint8_t) BNO55_TRIGGER_EXTCLK))
							{
							initcnt++;			// = x5
							Wait(2);
							pMyI2Cdatabuffer->CHIP_ID = 0;
							for (; (initcnt<900) && (pMyI2Cdatabuffer->CHIP_ID != BNO055_ID); initcnt+=100)	// wait up to .9 sec for the IMU Chip to boot up, checking every 1/10 sec
								{
								MyIMU->Read(BNO055_CHIP_ID_ADDR, I2C_RCV_ID_LEN, &pMyI2Cdatabuffer->ID_Buffer);	// read the Device ID data registers
								Wait(0.1);
								}
							if (pMyI2Cdatabuffer->CHIP_ID == BNO055_ID)							// good news!  The IMU has booted and is responsive, so begin Configuration
								{
								initcnt++;			// = xx6
								/*
								 * We might need to loop here until the Calibration status is non-zero
								 * We waited 10 seconds and the Calibration Status remained zero
								 *
								 * Most likely, we will need run the device through a multi-axis calibration sequence, tipping it onto each side
								 * in at least six different orientations in order to complete the Calibration Status comes back complete.
								 * This would necessitate mounting the device on a set of servo controlled gimbles.  Not sure we can control those
								 * servos until after the RobotInit() or (worse) RobotEnable signal is passed.
								 */
//								for (int i=0; i<1000 && pMyI2Cdatabuffer->Status.CALIB_STAT == 0; i++)
									{
									MyIMU->Read(BNO055_CALIB_STAT_ADDR, I2C_RCV_STATUS_LEN, &pMyI2Cdatabuffer->Status_Buffer);
									Wait(0.01);										// wait 10 msec for the IMU Chip enter Config mode (Chip Specs says 19ms)
									}
								initcnt++;			// = xx7
//								if ((! MyIMU->Write((uint8_t) BNO055_OPR_MODE_ADDR, (uint8_t) BNO055_OPERATION_MODE_IMU)))
								if ((! MyIMU->Write((uint8_t) BNO055_OPR_MODE_ADDR, (uint8_t) BNO055_OPERATION_MODE_NDOF)))
//										&& (! MyIMU->Write((uint8_t) BNO055_SYS_TRIGGER_ADDR, (uint8_t) BNO55_TRIGGER_EXTCLK)))
									{
									initcnt += 1000;			// = 1xx7
									Wait(0.01);										// wait 10 msec for the IMU Chip enter Operational mode (Chip Specs says 7ms)
									}
								}
							}
						}
					}
				}
			}
		pMyRobotState->IMU_ChipID = ((int) pMyI2Cdatabuffer->CHIP_ID) & 0xFF;
		pMyRobotState->IMU_CalStat = ((int) pMyI2Cdatabuffer->CALIB_STAT) & 0xFF;
		pMyRobotState->IMU_SelfTest = ((int) pMyI2Cdatabuffer->ST_RESULT) & 0xFF;
		pMyRobotState->IMU_IntrStat = ((int) pMyI2Cdatabuffer->INT_STA) & 0xFF;
		pMyRobotState->IMU_ClockStat = ((int) pMyI2Cdatabuffer->SYS_CLK_STATUS) & 0xFF;
		pMyRobotState->IMU_SystemStat = ((int) pMyI2Cdatabuffer->SYS_STATUS) & 0xFF;
		pMyRobotState->IMU_SystemError = ((int) pMyI2Cdatabuffer->SYS_ERR) & 0xFF;
	#endif
	#ifdef MPU6050_6DOF
//		cw_sensitivity = GYRO_CW_SENSITIVITY / 1000000000;		// Gyro is not consistent between clockwise turns and counter-clockwise turns
//		ccw_sensitivity = GYRO_CCW_SENSITIVITY / 1000000000;	// Up to about 0.5 degrees per revolution difference between them
//		GyroNoiseLimit = GYRO_NOISELIMIT;
//		GyroCenter_X = GyroCenter_Y = GyroCenter_Z = GYRO_RATECENTER / 1000;
//		XaccelCenter = ACCEL_XCENTER / 1000;
//		YaccelCenter = ACCEL_YCENTER / 1000;
//		ZaccelCenter = ACCEL_ZCENTER / 1000;


		// Configure and initialize the gyro
		if (! MyIMU->AddressOnly())		// if the gyro is present, initialize its registers
			{
			initcnt++;
			if ((! MyIMU->Write((uint8_t) DOF6_Reg_PWR_MGT1, (uint8_t) DOF6_PLL_Z))
			&& (! MyIMU->Write((uint8_t) DOF6_Reg_MOT_CTL, (uint8_t) DOF6_PON_DELAY))
			&& (! MyIMU->Write((uint8_t) DOF6_Reg_CONFIG, (uint8_t) DOF6_8kHz))
			&& (! MyIMU->Write((uint8_t) DOF6_Reg_GCONFIG, (uint8_t) DOF6_500dps))
			&& (! MyIMU->Write((uint8_t) DOF6_Reg_ACONFIG, (uint8_t) DOF6_2g))
			&& (! MyIMU->Write((uint8_t) DOF6_Reg_SMPLRT_DIV, (uint8_t) DOF6_SampleDiv8))
			&& (! MyIMU->Write((uint8_t) DOF6_Reg_INT_CFG, (uint8_t) (DOF6_Latch | DOF6_AnyRd2Clr)))
			&& (! MyIMU->Write((uint8_t) DOF6_Reg_INT_ENA, (uint8_t) DOF6_DataRdy)))
				{
				initcnt++;
	#ifdef MPU6050_USING_FIFO
				if ((! MyIMU->Write((uint8_t) DOF6_Reg_FIFO_EN, (uint8_t) (DOF6_FIFO_GX | DOF6_FIFO_GY | DOF6_FIFO_GZ | DOF6_FIFO_ACC)))
						&& (! MyIMU->Write((uint8_t) DOF6_Reg_USER_CTL, (uint8_t) DOF6_FIFO_EN))
						&& (! MyIMU->Write((uint8_t) DOF6_Reg_USER_CTL, (uint8_t) DOF6_FIFO_RST))
						&& (! MyIMU->Write((uint8_t) DOF6_Reg_USER_CTL, (uint8_t) DOF6_FIFO_EN)))
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

			// The temperature probe is built into the I2C gyro and was configured with the gyro.
			thermo_sensitivity = (SmartDashboard::GetNumber("Thermo_sensitivity",GYRO_THERMO_SENSITIVITY) / 1000000000);
			temperature = GyroCenter_T = GYRO_THERMO_CENTER;
			SmartDashboard::PutNumber("Thermo_sensitivity",thermo_sensitivity * 1000000000);

	*/
			}
	#endif

	#ifdef ITG3200_GYRO
	// Did I mention that the gyro doesn't give the same
	// voltage shift / dps for clockwise and counter-clockwise.
	// In other words, the voltage shift does not appear to be entirely linear.
	// It's just a teeny, tiny bit off.
	// Did I mention that, if the gyro is not placed in the exact center of rotation
	// of the robot, then the above mentioned issue is further exagerated as speed
	// of rotation increases.
	// Configure and initialize the gyro
	if (! MyIMU->AddressOnly())		// if the gyro is present, initialize its registers
		{
		initcnt++;
//		if ((! MyIMU->Write(GYRO_Reg_PWR_MGM, GYRO_H_Reset)))
			{
			Wait(1);
			if ((! MyIMU->Write((uint8_t) GYRO_Reg_PWR_MGM, (uint8_t) GYRO_Clock_Z))
					&& (! MyIMU->Write((uint8_t) GYRO_Reg_DLPF_FS, (uint8_t) (GYRO_2000dps | GYRO_1kx5Hz)))
					&& (! MyIMU->Write((uint8_t) GYRO_Reg_SMPLRT_DIV, (uint8_t) GYRO_SampDiv1))
					&& (! MyIMU->Write((uint8_t) GYRO_Reg_INT_CFG, (uint8_t) (GYRO_DataRdy | GYRO_Latch | GYRO_AnyRd2Clr))))
//				&& (! MyIMU->Write((uint8_t) GYRO_Reg_SMPLRT_DIV, (uint8_t) GYRO_SampleDiv))
					//&& (! MyIMU->Write((uint8_t) GYRO_Reg_INT_CFG, (uint8_t) (GYRO_DataRdy | GYRO_Latch | GYRO_AnyRd2Clr))))
					//&& (! MyIMU->Write((uint8_t) GYRO_Reg_ZoffsH, (uint8_t) ((int16_t) GYRO_RATECENTER >> 8)))
					//&& (! MyIMU->Write((uint8_t) GYRO_Reg_ZoffsL, (uint8_t) ((int16_t) GYRO_RATECENTER & 0xFF))))
					//	&& (! MyIMU->Write((uint8_t) GYRO_Reg_FIFOen, (uint8_t) GYRO_FIFO_GX | GYRO_FIFO_GZ | GYRO_FIFO_ft)))
				{
				initcnt++;
				}
			}
		}
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
	}


void NAVIGATOR :: SetStartingPosition()
	{
	pMyRobotState->Robot_Position_X = pMyRobotState->Auto_Start_X;
	pMyRobotState->Robot_Position_Y = pMyRobotState->Auto_Start_Y;
	pMyRobotState->Robot_Heading = pMyRobotState->Auto_Start_Heading;
	pMyRobotState->Robot_Direction = 0;
	pMyRobotState->Robot_Speed = 0;
	pMyRobotState->Nav_Robot_Lost = false;
	pMyRobotState->Nav_Gyro_Tilt = false;
	}

void NAVIGATOR :: Init(RobotStateBuffer *pRobotState)
	{
	pMyRobotState = pRobotState;
	SetStartingPosition();
	NavTimer.Reset();
	initcnt = 0;
	InitMyIMU();
	INS_Calibrating = true;
	Wait(1);				// Wait for new sensor values to stabilize
//	ReadThermostat();
//	ReadGyro();
//	ReadAccel();
//	for (SampleCount=1; (SampleCount < 5) || ((ReadEuler() > 0.0001) && (SampleCount < 20)); SampleCount++) SmartDashboard::PutNumber("EulerStabilizationCount",SampleCount);;	// read gyro until it stabilizes
//	for (SampleCount=1; (SampleCount < 5) || ((ReadGyro() > 0.0001) && (SampleCount < 20)); SampleCount++) SmartDashboard::PutNumber("GyroStabilizationCount",SampleCount);;	// read gyro until it stabilizes
//	SampleCount = 0;		// Use the first set of values to set center points.
	NavTimer.Start();
	for (int i=1; (i < 200) && (fabs(NavTimer.Get()) < .000001); i++) Wait (.000001);  // wait for timer to start running
	INS_semaphore = initializeMutexNormal();
	pReadSensors->StartPeriodic(SAMPLEPERIOD);	// schedule periodic sampling of Gyro and Accel data
	pINSupdate->StartPeriodic(INSUPDATEPERIOD);	// schedule periodic updates of the Robot position, orientation, speed and direction of travel
	pMyRobotState->Navigator_Init = initcnt;
	}

/*
void NAVIGATOR :: ReadIMUCalStat()
	{
	#ifdef BNO055_9DOF
		int16_t	x, y, z;
		int	mag_stat, acc_stat, gyro_stat, mcu_stat;
		int	mag_cal, acc_cal, gyro_cal, sys_cal;
		mag_stat = acc_stat = gyro_stat = mcu_stat = 0;
		mag_cal = acc_cal = gyro_cal = sys_cal = 0;
		memset(IMU_rcv_buff, 0, sizeof(IMU_rcv_buff));
		MyIMU->Read(BNO055_CALIB_STAT_ADDR, BNO055_STATUS_CNT, IMU_rcv_buff);
		pMyRobotState->IMU_CalStat = ((int) IMU_rcv_buff[0]) & 0xFF;
		pMyRobotState->IMU_SelfTest = ((int) IMU_rcv_buff[1]) & 0xFF;
		pMyRobotState->IMU_IntrStat = ((int) IMU_rcv_buff[2]) & 0xFF;
		pMyRobotState->IMU_ClockStat = ((int) IMU_rcv_buff[3]) & 0xFF;
		pMyRobotState->IMU_SystemStat = ((int) IMU_rcv_buff[4]) & 0xFF;
		pMyRobotState->IMU_SystemError = ((int) IMU_rcv_buff[5]) & 0xFF;
		acc_stat = (pMyRobotState->IMU_SelfTest & BNO055_ST_STAT_ACC) >> 0;
		mag_stat = (pMyRobotState->IMU_SelfTest & BNO055_ST_STAT_MAG) >> 1;
		gyro_stat = (pMyRobotState->IMU_SelfTest & BNO055_ST_STAT_GYRO) >> 2;
		mcu_stat = (pMyRobotState->IMU_SelfTest & BNO055_ST_STAT_MCU) >> 3;
		mag_cal = pMyRobotState->IMU_CalStat & BNO055_CALIB_STAT_MAG;
		acc_cal = (pMyRobotState->IMU_CalStat & BNO055_CALIB_STAT_ACC) >> 2;
		gyro_cal = (pMyRobotState->IMU_CalStat & BNO055_CALIB_STAT_GYRO) >> 4;
		sys_cal = (pMyRobotState->IMU_CalStat & BNO055_CALIB_STAT_SYS) >> 6;
		SmartDashboard::PutNumber("Mag_Stat",mag_stat);
		SmartDashboard::PutNumber("Acc_Stat",acc_stat);
		SmartDashboard::PutNumber("Gyro_Stat",gyro_stat);
		SmartDashboard::PutNumber("MCU_Stat",mcu_stat);
		SmartDashboard::PutNumber("Mag_Cal",mag_cal);
		SmartDashboard::PutNumber("Acc_Cal",acc_cal);
		SmartDashboard::PutNumber("Gyro_Cal",gyro_cal);
		SmartDashboard::PutNumber("Sys_Cal",sys_cal);
		if ((sys_cal == 3) && (pMyRobotState->Navigator_Init < 10000) && (! MyIMU->Write((uint8_t) BNO055_OPR_MODE_ADDR, (uint8_t) BNO055_OPERATION_MODE_CONFIG)))
			{
			memset(IMU_rcv_buff, 0, sizeof(IMU_rcv_buff));
			MyIMU->Read(BNO055_ACCEL_OFFSET_X_LSB_ADDR, BNO055_STATUS_CNT, IMU_rcv_buff);
			pMyRobotState->IMU_Accel_X_Offset = (x = ((uint16_t) IMU_rcv_buff[1] << 8) | (uint16_t) IMU_rcv_buff[0]);
			pMyRobotState->IMU_Accel_Y_Offset = (y = ((uint16_t) IMU_rcv_buff[3] << 8) | (uint16_t) IMU_rcv_buff[2]);
			pMyRobotState->IMU_Accel_Z_Offset = (z = ((uint16_t) IMU_rcv_buff[5] << 8) | (uint16_t) IMU_rcv_buff[4]);

			memset(IMU_rcv_buff, 0, sizeof(IMU_rcv_buff));
			MyIMU->Read(BNO055_MAG_OFFSET_X_LSB_ADDR, BNO055_STATUS_CNT, IMU_rcv_buff);
			pMyRobotState->IMU_Mag_X_Offset = (x = ((uint16_t) IMU_rcv_buff[1] << 8) | (uint16_t) IMU_rcv_buff[0]);
			pMyRobotState->IMU_Mag_Y_Offset = (y = ((uint16_t) IMU_rcv_buff[3] << 8) | (uint16_t) IMU_rcv_buff[2]);
			pMyRobotState->IMU_Mag_Z_Offset = (z = ((uint16_t) IMU_rcv_buff[5] << 8) | (uint16_t) IMU_rcv_buff[4]);

			memset(IMU_rcv_buff, 0, sizeof(IMU_rcv_buff));
			MyIMU->Read(BNO055_GYRO_OFFSET_X_LSB_ADDR, BNO055_STATUS_CNT, IMU_rcv_buff);
			pMyRobotState->IMU_Gyro_X_Offset = (x = ((uint16_t) IMU_rcv_buff[1] << 8) | (uint16_t) IMU_rcv_buff[0]);
			pMyRobotState->IMU_Gyro_Y_Offset = (y = ((uint16_t) IMU_rcv_buff[3] << 8) | (uint16_t) IMU_rcv_buff[2]);
			pMyRobotState->IMU_Gyro_Z_Offset = (z = ((uint16_t) IMU_rcv_buff[5] << 8) | (uint16_t) IMU_rcv_buff[4]);

			memset(IMU_rcv_buff, 0, sizeof(IMU_rcv_buff));
			MyIMU->Read(BNO055_ACCEL_RADIUS_LSB_ADDR, BNO055_STATUS_CNT, IMU_rcv_buff);
			pMyRobotState->IMU_Acc_Radius = (x = ((uint16_t) IMU_rcv_buff[1] << 8) | (uint16_t) IMU_rcv_buff[0]);
			pMyRobotState->IMU_Mag_Radius = (y = ((uint16_t) IMU_rcv_buff[3] << 8) | (uint16_t) IMU_rcv_buff[2]);

			MyIMU->Write((uint8_t) BNO055_OPR_MODE_ADDR, (uint8_t) BNO055_OPERATION_MODE_IMU);
			pMyRobotState->Navigator_Init += 10000;
			}
		}
	#endif
*/

void NAVIGATOR :: ReadThermostat()
	{
	#ifdef BNO055_9DOF
		int8_t	temp;
		pMyRobotState->Nav_Temp = (temp = (uint8_t) pMyI2Cdatabuffer->TEMP);
		NAVtemperature = pMyRobotState->Nav_Temp - pMyRobotState->Nav_IMU_Temp_Center;
		/*
		 * Rather than automagically detecting the "Normal" (Center) value of the temperature sensor, we want to
		 * hard-code the Center value that corresponds to zero compensation from the prescribed sensor sensivities.
		 * Per the book, that is 25 degrees Celsius.
		 */
	#endif
	#ifdef MPU6050_6DOF
		int16_t	temp;
		temp = ((uint16_t) pMyI2Cdatabuffer->TEMP_DATA_MSB << 8) | ((uint16_t) pMyI2Cdatabuffer->TEMP_DATA_LSB);
		pMyRobotState->Nav_Temp = (temp / 340) + 36.53;
		NAVtemperature = pMyRobotState->Nav_Temp - pMyRobotState->Nav_IMU_Temp_Center;
	#endif

	#ifdef ITG3200_GYRO
		int16_t	temp;
		pMyRobotState->Nav_Temp = temp = ((uint16_t) pMyI2Cdatabuffer->TEMP_DATA_MSB << 8) | ((uint16_t) pMyI2Cdatabuffer->TEMP_DATA_LSB);
		NAVtemperature = pMyRobotState->Nav_Temp - pMyRobotState->Nav_IMU_Temp_Center;
	#endif

	}

void NAVIGATOR :: ReadEuler()
	{
	#ifdef BNO055_9DOF
		int16_t	Euler_Heading, Euler_Roll, Euler_Pitch;
		double	Hdelta, Rdelta, Pdelta;
		double	heading, roll, pitch;

		Heading = (Euler_Heading = ((uint16_t) pMyI2Cdatabuffer->EUL_DATA_Z_MSB << 8) | ((uint16_t) pMyI2Cdatabuffer->EUL_DATA_Z_LSB)) / GYRO_RADIANS;
		Roll = (Euler_Roll = ((uint16_t) pMyI2Cdatabuffer->EUL_DATA_Y_MSB << 8) | ((uint16_t) pMyI2Cdatabuffer->EUL_DATA_Y_LSB)) / GYRO_RADIANS;
		Pitch = (Euler_Pitch = ((uint16_t) pMyI2Cdatabuffer->EUL_DATA_X_MSB << 8) | ((uint16_t) pMyI2Cdatabuffer->EUL_DATA_X_LSB)) / GYRO_RADIANS;
		Hdelta = (Heading - prev_Heading) - pMyRobotState->Nav_Gyro_H_Center;
		Rdelta = (Roll - prev_Roll) - pMyRobotState->Nav_Gyro_R_Center;
		Pdelta = (Pitch - prev_Pitch) - pMyRobotState->Nav_Gyro_P_Center;
		prev_Heading = Heading;
		prev_Roll = Roll;
		prev_Pitch = Pitch;
		// Scale Heading change value if wrapping around the upper limit (360 degrees) (or is that 0 degrees?).
		/*
		 * Note: we are assuming that in .01 seconds, the robot cannot turn more than HALF_PI radians.
		 * 		 Otherwise, it is capable of turning 1500 RPM, which is 540000 degrees per second (270 times our instrumentation limit).
		 * 		 We only need to do this for the Heading change, since the only way to wrap pitch and roll values is to tip the robot over.
		 */
		Hdelta += (Hdelta > HALF_PI) ? -1.0 * TWO_PI : (Hdelta < (-1.0 * HALF_PI)) ? TWO_PI : 0;
		/*
		 * May need to compensate Heading, Roll and Pitch change values for temperature variations.  Not sure at this point.
		 */

		/*
		 * Accumulate the changes in heading, roll and pitch
		 */
		heading = pMyRobotState->Nav_Gyro_Heading + Hdelta;
		roll = pMyRobotState->Nav_Gyro_Roll + Rdelta;
		pitch = pMyRobotState->Nav_Gyro_Pitch + Pdelta;

		if (INS_Calibrating)	// && (SampleCount > 1))
			{
			/*
			 * Not clear at this time whether moving/repositioning the robot during the calibration sequence should cause
			 * the Center values to be updated.  If you do not re-center, the movement will register as drift.  If you do
			 * re-center, drift will always be zero because the center point will be updated to remove any drift.
			 * For now, I think we need to re-center and if the gyro tends to drift, we will need to detect that from the
			 * gyro rate values.
			 */
			pMyRobotState->Nav_Gyro_H_Center = heading / (double) SampleCount;
			pMyRobotState->Nav_Gyro_R_Center = roll / (double) SampleCount;
			pMyRobotState->Nav_Gyro_P_Center = pitch / (double) SampleCount;
			heading = roll = pitch = 0;
			}

		// scale Heading to range 0..TWO_PI
		while (heading < 0.0) heading += TWO_PI;
		while (heading >= TWO_PI) heading -= TWO_PI;
		// scale Roll and Pitch to range -PI..+PI
		while (roll <= (-1 * PI)) roll += TWO_PI;
		while (roll > PI) roll -= TWO_PI;
		while (pitch <= (-1 * PI)) pitch += TWO_PI;
		while (pitch > PI) pitch -= TWO_PI;
		// update the state buffer
		pMyRobotState->Nav_Gyro_Heading = heading;
		pMyRobotState->Nav_Gyro_Roll = roll;
		pMyRobotState->Nav_Gyro_Pitch = pitch;
		//return fabs(Hdelta);
	#endif
}

void NAVIGATOR :: ReadGyro()
	{
	int16_t	Xrate, Yrate, Zrate;
	double	gx, gy, gz, pwr_correction_X = 0, pwr_correction_Y = 0, pwr_correction_Z = 0;

	// pick off the gyro rates for each axis
// 	gx = ((Xrate = ((uint16_t) pMyI2Cdatabuffer->GYR_DATA_X_MSB << 8) | ((uint16_t) pMyI2Cdatabuffer->GYR_DATA_X_LSB))) - pMyRobotState->Nav_Gyro_X_Center;
// 	gy = ((Yrate = ((uint16_t) pMyI2Cdatabuffer->GYR_DATA_Y_MSB << 8) | ((uint16_t) pMyI2Cdatabuffer->GYR_DATA_Y_LSB))) - pMyRobotState->Nav_Gyro_Y_Center;
// 	gz = ((Zrate = ((uint16_t) pMyI2Cdatabuffer->GYR_DATA_Z_MSB << 8) | ((uint16_t) pMyI2Cdatabuffer->GYR_DATA_Z_LSB))) - pMyRobotState->Nav_Gyro_Z_Center;
	/*
	 * IMU is reoriented.  Map X, Y, Z axis accordingly
	 */

	/*
	 * Newest discovery:  The voltage regulator on the MPU-6050 is inadquate to manage the power fluctuations on the I2C power pin.
	 * This causes the gyro to appear to spin, because the center position of the gyro deviates as the voltage on the I2C power pin fluctuates.
	 * So, now we introduce a factor to account for this.
	 */

	#ifdef MPU6050_6DOF
		pwr_correction_X = pMyRobotState->Nav_Gyro_X_PWR_Sensitivity * pMyRobotState->Power_Consumption;
		pwr_correction_Y = pMyRobotState->Nav_Gyro_Y_PWR_Sensitivity * pMyRobotState->Power_Consumption;
		pwr_correction_Z = pMyRobotState->Nav_Gyro_Z_PWR_Sensitivity * pMyRobotState->Power_Consumption;
	#endif

 	gx = ((Xrate = ((uint16_t) pMyI2Cdatabuffer->GYR_DATA_X_MSB << 8) | ((uint16_t) pMyI2Cdatabuffer->GYR_DATA_X_LSB)))
		+ pwr_correction_X - pMyRobotState->Nav_Gyro_X_Center;
 	gy = ((Yrate = ((uint16_t) pMyI2Cdatabuffer->GYR_DATA_Z_MSB << 8) | ((uint16_t) pMyI2Cdatabuffer->GYR_DATA_Z_LSB)))
		+ pwr_correction_Y - pMyRobotState->Nav_Gyro_Y_Center;
 	gz = ((Zrate = ((uint16_t) pMyI2Cdatabuffer->GYR_DATA_Y_MSB << 8) | ((uint16_t) pMyI2Cdatabuffer->GYR_DATA_Y_LSB)))
		+ pwr_correction_Z - pMyRobotState->Nav_Gyro_Z_Center;

//	SmartDashboard::PutNumber("DirectZ",gz);

	// Squelch noise
	gx *= (fabs(gx / pMyRobotState->Nav_Gyro_X_U_Sensitivity) < pMyRobotState->Nav_Gyro_Noise_Base) ? pMyRobotState->Nav_Gyro_Squelch : 1;
	gy *= (fabs(gy / pMyRobotState->Nav_Gyro_Y_L_Sensitivity) < pMyRobotState->Nav_Gyro_Noise_Base) ? pMyRobotState->Nav_Gyro_Squelch : 1;
	gz *= (fabs(gz / pMyRobotState->Nav_Gyro_Z_CW_Sensitivity) < pMyRobotState->Nav_Gyro_Noise_Base) ? pMyRobotState->Nav_Gyro_Squelch : 1;

	if (INS_Calibrating)
		{
		if (fmax(fmax(fabs(gx),fabs(gy)),fabs(gz)) > pMyRobotState->Nav_Gyro_Noise_Limit)
			{
			//	The robot got moved during the calibration sequence.
			pMyRobotState->Nav_Gyro_Noisy_Count++;
			}
		// set the center value to the average of all raw values sampled during calibration
//		pMyRobotState->Nav_Gyro_X_Center = ((pMyRobotState->Nav_Gyro_X_Center * (double) (SampleCount - 1)) + gx) / (double) SampleCount;
//		pMyRobotState->Nav_Gyro_Y_Center = ((pMyRobotState->Nav_Gyro_Y_Center * (double) (SampleCount - 1)) + gy) / (double) SampleCount;
//		pMyRobotState->Nav_Gyro_Z_Center = ((pMyRobotState->Nav_Gyro_Z_Center * (double) (SampleCount - 1)) + gz) / (double) SampleCount;
//		pMyRobotState->Nav_Gyro_X_Center += (gx / (double) SampleCount);
//		pMyRobotState->Nav_Gyro_Y_Center += (gy / (double) SampleCount);
//		pMyRobotState->Nav_Gyro_Z_Center += (gz / (double) SampleCount);
//		pMyRobotState->Nav_Gyro_X_Center += gx;
//		pMyRobotState->Nav_Gyro_Y_Center += gy;
//		pMyRobotState->Nav_Gyro_Z_Center += gz;
		double s = sqrt((double) SampleCount);
		pMyRobotState->Nav_Gyro_X_Center += (gx / s);
		pMyRobotState->Nav_Gyro_Y_Center += (gy / s);
		pMyRobotState->Nav_Gyro_Z_Center += (gz / s);
//		gx = gy = gz = 0;
		}

	// Scale for rate sensitivity
	/*
	 * According to the BNO055 manufacturer's guide, it should be 900 = 1 radian per second.
	 * But, I'm finding that value to be very far from reality.  It's coming out more like 804 = 1 radian per second.
	 * That's way out of tolerance.  And the scale is also not linear.
	 *
	 * According to the MPU6050 guide, @500 dps max rate setting, 65.5 LSB = 1 dps., So, 3752.87 LSB = 1 rad/s.
	 */

// 	gx /= pMyRobotState->Nav_Gyro_X_Sensitivity;
//	gy /= pMyRobotState->Nav_Gyro_Y_Sensitivity;
	// Also, compensate for non-linear spin sensitivity
	/*
	 * Sometimes gyro mems devices do not produce the same absolute signal level for the same rate of spin in opposite directions.
	 * Also, they do not always produce the same change in signal level for equal differences in spin rate at various spin rates.
	 */
	if (gx > 0.0)	gx /= pMyRobotState->Nav_Gyro_X_U_Sensitivity;
	else			gx /= pMyRobotState->Nav_Gyro_X_D_Sensitivity;

	if (gy > 0.0)	gy /= pMyRobotState->Nav_Gyro_Y_R_Sensitivity;
	else			gy /= pMyRobotState->Nav_Gyro_Y_L_Sensitivity;

	if (gz > 0.0)	gz /= (pMyRobotState->Nav_Gyro_Z_CW_Sensitivity + (gz * pMyRobotState->Nav_Gyro_Z_CW_Spin_Effect));
	else			gz /= (pMyRobotState->Nav_Gyro_Z_CCW_Sensitivity + (gz * pMyRobotState->Nav_Gyro_Z_CCW_Spin_Effect));

	// Now compensate for the temperature
	// Typical BNO055 gyroscope rate sensivity to temperature deviation is +/-0.03% per degree Celsius
	pMyRobotState->Nav_Gyro_X_Raw = gx * (1.0 + (NAVtemperature * GYRO_THERMO_SENSITIVITY));
	pMyRobotState->Nav_Gyro_Y_Raw = gy * (1.0 + (NAVtemperature * GYRO_THERMO_SENSITIVITY));
	pMyRobotState->Nav_Gyro_Z_Raw = gz * (1.0 + (NAVtemperature * GYRO_THERMO_SENSITIVITY));

	pMyRobotState->Robot_Spin_Rate = (pMyRobotState->Robot_Spin_Rate + pMyRobotState->Nav_Gyro_Z_Raw) / 2;

	//return (fabs(pMyRobotState->Nav_Gyro_X_Raw) + fabs(pMyRobotState->Nav_Gyro_Y_Raw) + fabs(pMyRobotState->Nav_Gyro_Z_Raw));
	}


#ifndef ITG3200_GYRO
void NAVIGATOR :: ReadAccel()
	{
	int16_t	XGs, YGs, ZGs;
	double	ax, ay, az, ab;

	ax = (XGs = ((uint16_t) pMyI2Cdatabuffer->ACC_DATA_X_MSB << 8) | ((uint16_t) pMyI2Cdatabuffer->ACC_DATA_X_LSB)) - pMyRobotState->Nav_Acc_X_Center;
	ay = (YGs = ((uint16_t) pMyI2Cdatabuffer->ACC_DATA_Y_MSB << 8) | ((uint16_t) pMyI2Cdatabuffer->ACC_DATA_Y_LSB)) - pMyRobotState->Nav_Acc_Y_Center;
	az = (ZGs = ((uint16_t) pMyI2Cdatabuffer->ACC_DATA_Z_MSB << 8) | ((uint16_t) pMyI2Cdatabuffer->ACC_DATA_Z_LSB)) - pMyRobotState->Nav_Acc_Z_Center;
	/*
	 * IMU is reoriented.  Map X, Y, Z axis accordingly
	 */
//	ax = (XGs = ((uint16_t) pMyI2Cdatabuffer->ACC_DATA_X_MSB << 8) | ((uint16_t) pMyI2Cdatabuffer->ACC_DATA_X_LSB)) - pMyRobotState->Nav_Acc_X_Center;
//	ay = (YGs = ((uint16_t) pMyI2Cdatabuffer->ACC_DATA_Z_MSB << 8) | ((uint16_t) pMyI2Cdatabuffer->ACC_DATA_Z_LSB)) - pMyRobotState->Nav_Acc_Y_Center;
//	az = (ZGs = ((uint16_t) pMyI2Cdatabuffer->ACC_DATA_Y_MSB << 8) | ((uint16_t) pMyI2Cdatabuffer->ACC_DATA_Y_LSB)) - pMyRobotState->Nav_Acc_Z_Center;

	// Squelch noise
	ab = pMyRobotState->Nav_Acc_Noise_Base * ACCEL_IPS2;
	ax *= (fabs(ax) < ab) ? pMyRobotState->Nav_Acc_Squelch : 1;
	ay *= (fabs(ay) < ab) ? pMyRobotState->Nav_Acc_Squelch : 1;
	az *= (fabs(az) < ab) ? pMyRobotState->Nav_Acc_Squelch : 1;

	if (INS_Calibrating)
		{
		if (fmax(fmax(fabs(XGs),fabs(YGs)),fabs(ZGs)) > pMyRobotState->Nav_Acc_Noise_Limit)
			{
			//	The robot got moved during the calibration sequence.
			noisyAccelcnt++;
			}
		// set the center value to the average of all raw values sampled during calibration
//		pMyRobotState->Nav_Acc_X_Center = ((pMyRobotState->Nav_Acc_X_Center * (double) (SampleCount - 1)) + ax) / (double) SampleCount;
//		pMyRobotState->Nav_Acc_Y_Center = ((pMyRobotState->Nav_Acc_Y_Center * (double) (SampleCount - 1)) + ay) / (double) SampleCount;
//		pMyRobotState->Nav_Acc_Z_Center = ((pMyRobotState->Nav_Acc_Z_Center * (double) (SampleCount - 1)) + az) / (double) SampleCount;
		double s = sqrt((double) SampleCount);
		pMyRobotState->Nav_Acc_X_Center += (ax / s);
		pMyRobotState->Nav_Acc_Y_Center += (ay / s);
		pMyRobotState->Nav_Acc_Z_Center += (az / s);
		}

	// Scale for rate sensitivity
 	ax /= ACCEL_IPS2;
	ay /= ACCEL_IPS2;
	az /= ACCEL_IPS2;

	// Now compensate for the temperature
	// Typical BNO055 gyroscope rate sensivity to temperature deviation is +/-0.03% per degree Celsius
	pMyRobotState->Nav_Acc_X_Raw = ax * (1.0 + (NAVtemperature * ACCEL_THERMO_SENSITIVITY));
	pMyRobotState->Nav_Acc_Y_Raw = ay * (1.0 + (NAVtemperature * ACCEL_THERMO_SENSITIVITY));
	pMyRobotState->Nav_Acc_Z_Raw = az * (1.0 + (NAVtemperature * ACCEL_THERMO_SENSITIVITY));
	}
#endif


void NAVIGATOR :: ReadWheelEncoders()
	{
	#ifdef XYWHEELS
		// read the X and Y wheel encoders, convert to centimeters and only count the change.
		int x_ticks = pXWheel->Get();
		int y_ticks = pYWheel->Get();
		double x_dist = ((double) x_ticks) * (x_ticks > 0 ? XWHEEL_IN_PER_TICK_FWD : XWHEEL_IN_PER_TICK_REV);
		double y_dist = ((double) y_ticks) * (y_ticks > 0 ? YWHEEL_IN_PER_TICK_FWD : YWHEEL_IN_PER_TICK_REV);
		/*
		 * On error, the encoder Get method returns a zero.  That would be bad.
		 * So, if the Get method returns a zero, maintain the previous value (i.e. no movement)
		 */
		pMyRobotState->Nav_X_Wheel = (x_ticks == 0) ? 0 : x_dist - prev_x_wheel;
		pMyRobotState->Nav_Y_Wheel = (y_ticks == 0) ? 0 : y_dist - prev_y_wheel;
		prev_x_wheel += pMyRobotState->Nav_X_Wheel;
		prev_y_wheel += pMyRobotState->Nav_Y_Wheel;
	#endif
	#ifdef LRWHEELS
		// read the L and R wheel encoders, convert to inches and only count the change.
		int l_ticks = pLWheel->Get();
		int r_ticks = pRWheel->Get();
		#ifdef REPORT_NAV_DEBUG
			SmartDashboard::PutNumber("l_ticks",l_ticks);
			SmartDashboard::PutNumber("r_ticks",r_ticks);
		#endif
		double l_dist = ((double) l_ticks) * (l_ticks > 0 ? LWHEEL_IN_PER_TICK_FWD : LWHEEL_IN_PER_TICK_REV);
		double r_dist = ((double) r_ticks) * (r_ticks > 0 ? RWHEEL_IN_PER_TICK_FWD : RWHEEL_IN_PER_TICK_REV);
		/*
		 * On error, the encoder Get method returns a zero.  That would be bad.
		 * So, if the Get method returns a zero, maintain the previous value (i.e. no movement)
		 */
		pMyRobotState->Nav_L_Wheel = (l_ticks == 0) ? 0 : l_dist - prev_l_wheel;
		pMyRobotState->Nav_R_Wheel = (r_ticks == 0) ? 0 : r_dist - prev_r_wheel;
		prev_l_wheel += pMyRobotState->Nav_L_Wheel;
		prev_r_wheel += pMyRobotState->Nav_R_Wheel;
	#endif
	}



void NAVIGATOR :: Push_Navigation_Data_Frame(double timestamp, double x_gyro, double y_gyro, double z_gyro, double x_accel, double y_accel, double z_accel, double a_wheel, double b_wheel)
	{
	int queue_depth;
//	CRITICAL_REGION(INS_semaphore)		// we need to reference values that are updated outside of this procedure
		{								// and we don't want those values getting changed while we are referencing them
		takeMutex(INS_semaphore);
		queue_depth = (NAVIGATION_QUEUE_SIZE + Navigation_Data_Queue_Top - Navigation_Data_Queue_Bottom) % NAVIGATION_QUEUE_SIZE;
		giveMutex(INS_semaphore);
		}
//	END_REGION;
	if (queue_depth++ >= NAVIGATION_QUEUE_SIZE - 1)
		{
		// Queue is full and we are lost;
		// If in Autonomous, stop all motors
		pMyRobotState->Nav_Robot_Lost = true;
		}
	else
		{
		if (Navigation_Data_Queue_Top++ >= NAVIGATION_QUEUE_SIZE - 1)
			{
			takeMutex(INS_semaphore);
//			Synchronized sync(INS_semaphore);	// we are going to update values referenced by other procedure.
			Navigation_Data_Queue_Top = 0;
			giveMutex(INS_semaphore);
			}
		Navigation_Data_Queue[Navigation_Data_Queue_Top].timestamp = timestamp;
		Navigation_Data_Queue[Navigation_Data_Queue_Top].gyro_x = x_gyro;
		Navigation_Data_Queue[Navigation_Data_Queue_Top].gyro_y = y_gyro;
		Navigation_Data_Queue[Navigation_Data_Queue_Top].gyro_z = z_gyro;
		Navigation_Data_Queue[Navigation_Data_Queue_Top].accel_x = x_accel;
		Navigation_Data_Queue[Navigation_Data_Queue_Top].accel_y = y_accel;
		Navigation_Data_Queue[Navigation_Data_Queue_Top].accel_z = z_accel;
		#ifdef XYWHEELS
			Navigation_Data_Queue[Navigation_Data_Queue_Top].wheel_x = a_wheel;
			Navigation_Data_Queue[Navigation_Data_Queue_Top].wheel_y = b_wheel;
		#endif
		#ifdef LRWHEELS
			Navigation_Data_Queue[Navigation_Data_Queue_Top].wheel_l = a_wheel;
			Navigation_Data_Queue[Navigation_Data_Queue_Top].wheel_r = b_wheel;
		#endif
			{
			takeMutex(INS_semaphore);
//			Synchronized sync(INS_semaphore);	// we are going to update values referenced by other procedure.
			Navigation_Data_Queue_Depth++;
			giveMutex(INS_semaphore);
			}
		}
	}


double NAVIGATOR :: ReadIMUdata()
	{
//	unsigned char reg = BNO055_ACCEL_DATA_X_LSB_ADDR;

//	memset(&pMyI2Cdatabuffer->Data_Buffer, 0, I2C_RCV_DATA_LEN);
	#ifdef BNO055_9DOF
		MyIMU->Read(BNO055_ACCEL_DATA_X_LSB_ADDR, I2C_RCV_DATA_LEN, &pMyI2Cdatabuffer->Data_Buffer);
	#endif

	#ifdef MPU6050_6DOF
//		MyIMU->Read(DOF6_Reg_Accel_XH, I2C_RCV_DATA_LEN, &pMyI2Cdatabuffer->Data_Buffer);		// no check of DataReady Interrupt Bit
		MyIMU->Read(DOF6_Reg_INT_STATUS, I2C_RCV_DATA_LEN + I2C_RCV_STATUS_LEN, &pMyI2Cdatabuffer->Status_Buffer);	// capture DataReady Bit
	#endif

	#ifdef ITG3200_GYRO
		MyIMU->Read(GYRO_Reg_XOUT_H, I2C_RCV_DATA_LEN, &pMyI2Cdatabuffer->Data_Buffer);		// no check of DataReady Interrupt Bit
//		MyIMU->Read(GYRO_Reg_INT_STATUS, I2C_RCV_DATA_LEN + I2C_RCV_STATUS_LEN, &pMyI2Cdatabuffer->Status_Buffer);	// capture DataReady Bit
	#endif

	if (SampleCount % 100 == 0) ReadThermostat();
	ReadGyro();
//	ReadEuler();
	#ifndef ITG3200_GYRO
		ReadAccel();
	#endif
	ReadWheelEncoders();

	return (fabs(pMyRobotState->Nav_Gyro_X_Raw) + fabs(pMyRobotState->Nav_Gyro_Y_Raw) + fabs(pMyRobotState->Nav_Gyro_Z_Raw)
			+ fabs(pMyRobotState->Nav_Acc_X_Raw) + fabs(pMyRobotState->Nav_Acc_Y_Raw) + fabs(pMyRobotState->Nav_Acc_Z_Raw));
	}




void NAVIGATOR :: ReadSensors()
	{
	DataClockTime = NavTimer.Get();
	SampleCount++;
	ReadIMUdata();
//	/* ReadIMUdata() replaces ReadThermostat(); ReadGyro(); ReadEuler(); and ReadAccel() with a single data fetch
//	*/
	Push_Navigation_Data_Frame(DataClockTime, pMyRobotState->Nav_Gyro_X_Raw, pMyRobotState->Nav_Gyro_Y_Raw, pMyRobotState->Nav_Gyro_Z_Raw,
											 pMyRobotState->Nav_Acc_X_Raw, pMyRobotState->Nav_Acc_Y_Raw, pMyRobotState->Nav_Acc_Z_Raw,
											 pMyRobotState->Nav_L_Wheel, pMyRobotState->Nav_R_Wheel);
	pMyRobotState->dataspeed = (NavTimer.Get() - DataClockTime) * 1000;	// exec time of this method in msec
//	pMyRobotState->dataspeed = ((pMyRobotState->dataspeed * (double) (SampleCount - 1)) + SampleRunTime) / (double) SampleCount;
	pMyRobotState->datarate = int(1 / fmax(0.001, DataClockTime - DataCollectTime));  // avoid divide by zero crash
	DataCollectTime = DataClockTime;
	}

//  schedule this to run independently every 1.0 milliseconds
void NAVIGATOR :: CallReadSensors(void *controller)
	{
	NAVIGATOR *sample_sensors = (NAVIGATOR*) controller;
	sample_sensors->ReadSensors();
	}


void NAVIGATOR :: Zero()
	{
	pMyRobotState->Navigator_Init++;
	if (INS_Calibrating)
		{
		takeMutex(INS_semaphore);
//		Synchronized sync(INS_semaphore);	// we are going to update values referenced by other procedure.
		INS_Calibrating = false;
		SetStartingPosition();
		pMyRobotState->Nav_Timestamp = NavTimer.Get();
		pMyRobotState->Nav_X_Coord = pMyRobotState->Robot_Position_X;
		pMyRobotState->Nav_Y_Coord = pMyRobotState->Robot_Position_Y;
		pMyRobotState->Nav_X_Speed = 0;
		pMyRobotState->Nav_Y_Speed = 0;
		pMyRobotState->Nav_X_Accel = 0;
		pMyRobotState->Nav_Y_Accel = 0;
		pMyRobotState->Robot_Pitch = 0;
		pMyRobotState->Robot_Roll = 0;
		giveMutex(INS_semaphore);
		}
	}


bool NAVIGATOR :: Pop_Navigation_Data_Frame()
	{
	int queue_depth;
//	double noiselevelfilter = 25;
//	CRITICAL_REGION(INS_semaphore)		// we need to reference values that are updated outside of this procedure
		{								// and we don't want those values getting changed while we are referencing them
		takeMutex(INS_semaphore);
		queue_depth = (NAVIGATION_QUEUE_SIZE + Navigation_Data_Queue_Top - Navigation_Data_Queue_Bottom) % NAVIGATION_QUEUE_SIZE;
		giveMutex(INS_semaphore);
		}
//	END_REGION;
	if (queue_depth <= 1) return false;
	if (Navigation_Data_Queue_Bottom++ >= NAVIGATION_QUEUE_SIZE - 1)
		{
		takeMutex(INS_semaphore);
//		Synchronized sync(INS_semaphore);	// we are going to update values referenced by other procedure.
		Navigation_Data_Queue_Bottom = 0;
		giveMutex(INS_semaphore);
		}
	NavElapsedTime = Navigation_Data_Queue[Navigation_Data_Queue_Bottom].timestamp - pMyRobotState->Nav_Timestamp;
	NAVx_gyro = Navigation_Data_Queue[Navigation_Data_Queue_Bottom].gyro_x;
	NAVy_gyro = Navigation_Data_Queue[Navigation_Data_Queue_Bottom].gyro_y;
	NAVz_gyro = Navigation_Data_Queue[Navigation_Data_Queue_Bottom].gyro_z;
	NAVx_accel = (double) Navigation_Data_Queue[Navigation_Data_Queue_Bottom].accel_x;
	NAVy_accel = (double) Navigation_Data_Queue[Navigation_Data_Queue_Bottom].accel_y;
	NAVz_accel = (double) Navigation_Data_Queue[Navigation_Data_Queue_Bottom].accel_z;
	NAVl_wheel = (double) Navigation_Data_Queue[Navigation_Data_Queue_Bottom].wheel_l;
	NAVr_wheel = (double) Navigation_Data_Queue[Navigation_Data_Queue_Bottom].wheel_r;
		{
		takeMutex(INS_semaphore);
//		Synchronized sync(INS_semaphore);	// we are going to update values referenced by other procedure.
		Navigation_Data_Queue_Depth--;
		giveMutex(INS_semaphore);
		}
	return true;
	}

void NAVIGATOR :: Compute_Gyro_Orientation()
	{
	double new_heading = pMyRobotState->Robot_Heading + (NAVz_gyro * NavElapsedTime);
	double new_pitch = pMyRobotState->Robot_Pitch + (NAVx_gyro * NavElapsedTime);
	double new_roll = pMyRobotState->Robot_Roll + (NAVy_gyro * NavElapsedTime);
	/*
	 * Note:  The Roll axis is only allowed to measure +/- 90 degrees from level.
	 * 		Most of our robotics antics will not likely exceed this limit.
	 * 		However, if we do exceed those limits, then we will have re-orient
	 * 		the Heading and Pitch values to bring the Roll axis within
	 * 		those limits, while leaving the sensor in the same spacial orientation.
	 */
//	if (fabs(pMyRobotState->Robot_Roll) > HALF_PI)
//		{
//		double realign_axis = copysign(180, pMyRobotState->Robot_Roll);
//		pMyRobotState->Robot_Roll -= realign_axis;
//		pMyRobotState->Robot_Pitch -= realign_axis;
//		pMyRobotState->Robot_Heading -= realign_axis;
//		}
	if (fabs(NAVy_gyro) > GYRO_RATELIMIT)
		{
		pMyRobotState->Nav_Gyro_Tilt = true;
		}

	new_pitch += new_pitch > PI ? -1 * TWO_PI : new_pitch <= -1 * PI ? TWO_PI : 0;
	if (fabs(NAVx_gyro) > GYRO_RATELIMIT)
		{
		pMyRobotState->Nav_Gyro_Tilt = true;
		}

	new_heading += new_heading >= TWO_PI ? -1 * TWO_PI : new_heading < -0.0 ? TWO_PI : 0;
	if (fabs(NAVz_gyro) > GYRO_RATELIMIT)
		{
		pMyRobotState->Nav_Gyro_Tilt = true;
		}

	pMyRobotState->Robot_Pitch = new_pitch;
	pMyRobotState->Robot_Heading = new_heading;
	pMyRobotState->Robot_Roll = new_roll;
	}


void NAVIGATOR :: UpdatePosition()
	{
	#if defined XYWHEELS
		double x_distance = (cos((GyroHeading + Xdeviation) * DEGREES_TO_RADIANS) * NAVx_wheel) + (sin((GyroHeading + Ydeviation) * DEGREES_TO_RADIANS) * NAVy_wheel);
		double y_distance = (cos((GyroHeading + Ydeviation) * DEGREES_TO_RADIANS) * NAVy_wheel) - (sin((GyroHeading + Xdeviation) * DEGREES_TO_RADIANS) * NAVx_wheel);
		Navigation_Status_Buffer.Xspeed = (x_distance / TripTime);
		Navigation_Status_Buffer.Yspeed = (y_distance / TripTime);
		pMyRobotState->Robot_Position_X += x_distance;
		pMyRobotState->Robot_Position_Y += y_distance;
	#elif defined LRWHEELS
		/*
		 * We have two wheel encoders measuring forward movement of the Left and Right tankdrive motors.
		 * If the robot is not turning, the two encoders should ideally be equal.
		 * If they are not equal, the faster one is presumed to be slipping.  So, we only count the slower one.
		 * If both are slipping, we are going to miscalculate our movement.
		 * If the robot is turning, the difference in the two encoders should ideally account for the turn.
		 * If both wheels are going the same direction and the difference is less than needed to account for turning rate, then the slower wheel is slipping.
		 * If both wheels are going the same direction and the difference is greater than needed to account for turning rate, then the faster wheel is slipping.
		 * If the wheels are going in opposite directions and the difference is less than needed to account for turning rate, we are getting outside help to turn.
		 * If the wheels are going in opposite directions and the difference is greater than needed to account for turning rate, we are lost.
		 *
		 * Essentially, there is no good way to determine what part slippage and/or external forces are having on our robot's movement
		 * beyond what the encoders are telling us directly.  So, we're left with taking a good guess (and hopefully being able to
		 * use other cues along the way to provide positional updates).
		 *
		 * We will assume the slow wheel is not slipping and start with that.  Then, if the robot is actually turning,
		 * we will apply some portion of the wheel differential.  The higher the rate of robot rotation, the less of the
		 * wheel speed differential we will apply.
		 */
		double slow_wheel = (fmin(fabs(NAVl_wheel),fabs(NAVr_wheel)) == fabs(NAVl_wheel)) ? NAVl_wheel : NAVr_wheel;
		double fast_wheel = (slow_wheel == NAVl_wheel) ? NAVr_wheel : NAVl_wheel;

		double cycle_speed = (fast_wheel + slow_wheel) / 2;
		double x_distance = cycle_speed * sin(pMyRobotState->Robot_Heading);
		double y_distance = cycle_speed * cos(pMyRobotState->Robot_Heading);
		pMyRobotState->Robot_Speed = cycle_speed / fmax(0.001, NavElapsedTime);
		pMyRobotState->Robot_Position_X += x_distance;
		pMyRobotState->Robot_Position_Y += y_distance;
	#else
		// Adjust NAVx_accel and NAVy_accel for Pitch and Roll angles to align with the Robot Orientation (assuming zero axis misalignment)
		double RoAx = (NAVx_accel * cos(pMyRobotState->Robot_Roll)) + (NAVz_accel * sin(pMyRobotState->Robot_Roll));
		double RoAy = (NAVy_accel * cos(pMyRobotState->Robot_Pitch)) + (NAVz_accel * sin(pMyRobotState->Robot_Pitch));

		// Next, rotate the RoAx and RoAy values to align with the Field Orientation (assuming zero axis misalignment)
		pMyRobotState->Nav_X_Accel = (RoAx * cos(pMyRobotState->Robot_Heading)) - (RoAy * sin(pMyRobotState->Robot_Heading));
		pMyRobotState->Nav_Y_Accel = (RoAy * cos(pMyRobotState->Robot_Heading)) - (RoAx * sin(pMyRobotState->Robot_Heading));

		// Update the X and Y position based on prior s = (v * t) + (0.5 * a * t * t)
		double X_dist = (NavElapsedTime * pMyRobotState->Nav_X_Speed) + (0.5 * pMyRobotState->Nav_X_Accel * NavElapsedTime * NavElapsedTime);
		double Y_dist = (NavElapsedTime * pMyRobotState->Nav_Y_Speed) + (0.5 * pMyRobotState->Nav_Y_Accel * NavElapsedTime * NavElapsedTime);
		pMyRobotState->Nav_X_Coord += X_dist;
		pMyRobotState->Nav_Y_Coord += Y_dist;

		// Update the X and Y speed based on s = v + (a * t)
		pMyRobotState->Nav_X_Speed += (pMyRobotState->Nav_X_Accel * NavElapsedTime);
		pMyRobotState->Nav_Y_Speed += (pMyRobotState->Nav_Y_Accel * NavElapsedTime);

		// Update the Robot state values
		pMyRobotState->Robot_Position_X = pMyRobotState->Nav_X_Coord;
		pMyRobotState->Robot_Position_Y = pMyRobotState->Nav_Y_Coord;


		pMyRobotState->Robot_Direction = HALF_PI - atan2(y_distance, x_distance);
		pMyRobotState->Robot_Direction += pMyRobotState->Robot_Direction < 0 ? TWO_PI : 0;
		pMyRobotState->Robot_Speed = pow((pow(pMyRobotState->Nav_X_Speed,2) + pow(pMyRobotState->Nav_Y_Speed,2)),0.5);
	#endif

	}


// schedule this to run independently every 10 milliseconds
void NAVIGATOR :: UpdateINS()
	{
	NavClockTime = NavTimer.Get();
	while (Pop_Navigation_Data_Frame())
		{
		if (pMyRobotState->Nav_Timestamp > (5 * SAMPLEPERIOD))	// throw out the first 5 samples (~0.2 seconds)
			{										// gives sensors time to stabilize
			Compute_Gyro_Orientation();
			UpdatePosition();
			}
		pMyRobotState->Nav_Timestamp += NavElapsedTime;
		}
	pMyRobotState->updatespeed = (NavTimer.Get() - NavClockTime) * 1000;
	pMyRobotState->updaterate = int(1 / fmax(0.001, NavClockTime - NavUpdateTime));  // avoid divide by zero crash
	NavUpdateTime = NavClockTime;
	}


//  schedule this to run independently every 10 milliseconds
void NAVIGATOR :: CallUpdateINS(void *controller)
	{
	NAVIGATOR *ins_update = (NAVIGATOR*) controller;
	ins_update->UpdateINS();
	}


