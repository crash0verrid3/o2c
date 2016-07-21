#include <Navigator.h>

NAVIGATOR :: NAVIGATOR() :	INS_semaphore(0)

	{
	initcnt = 0;
	pMyFieldState = 0;
	pReadSensors = new Notifier(NAVIGATOR::CallReadSensors,this);
	pINSupdate = new Notifier(NAVIGATOR::CallUpdateINS,this);
	MyIMU = new I2C(I2C::kOnboard, BNO055_ADDRESS_A);
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
	Sensitivity_Offset = 0;		// we'll use this later to adjust sensitivity as a function of temperature (it does matter)
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
	uint8_t		chipID = 0;

	memset(IMU_rcv_buff, 0, sizeof(IMU_rcv_buff));
	Wait(1);
	if ((! MyIMU->AddressOnly()) && (! MyIMU->Write((uint8_t) BNO055_PAGE_ID_ADDR, 0)))
		{
		for (initcnt=1; (initcnt<90) && (chipID != BNO055_ID); initcnt+=10)	// wait up to 0.9 sec for the IMU Chip to boot up, checking every 1/10 sec
			{
			MyIMU->Read(BNO055_CHIP_ID_ADDR, 1, &chipID);	// read the ChipID
			pMyRobotState->IMU_ChipID = ((int) chipID) & 0xFF;
			Wait(0.1);
			}
		if (chipID == BNO055_ID)							// good news!  The IMU has booted and is responsive, so begin Configuration
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
/*

 */					{
					initcnt++;			// = x4
					Wait(0.01);
//					if (! MyIMU->Write((uint8_t) BNO055_SYS_TRIGGER_ADDR, (uint8_t) (BNO55_TRIGGER_EXTCLK | BNO55_TRIGGER_RSTSYS)))
					if (! MyIMU->Write((uint8_t) BNO055_SYS_TRIGGER_ADDR, (uint8_t) BNO55_TRIGGER_EXTCLK))
						{
						initcnt++;			// = x5
						Wait(2);
						chipID = 0;
						for (; (initcnt<900) && (chipID != BNO055_ID); initcnt+=100)	// wait up to .9 sec for the IMU Chip to boot up, checking every 1/10 sec
							{
							MyIMU->Read(BNO055_CHIP_ID_ADDR, 1, &chipID);	// read the ChipID
							Wait(0.1);
							}
						if (chipID == BNO055_ID)							// good news!  The IMU has booted and is responsive, so begin Configuration
							{
							initcnt++;			// = xx6
							/*
							 * We might need to loop here until the Calibration status is non-zero
							 * We waited 10 seconds and the Calibration Status remained zero
							 */
//							for (int i=0; i<1000 && IMU_rcv_buff[0] == 0; i++)
								{
								MyIMU->Read(BNO055_CALIB_STAT_ADDR, BNO055_STATUS_CNT, IMU_rcv_buff);
								Wait(0.01);										// wait 10 msec for the IMU Chip enter Config mode (Chip Specs says 19ms)
								}
							initcnt++;			// = xx7
//							if ((! MyIMU->Write((uint8_t) BNO055_OPR_MODE_ADDR, (uint8_t) BNO055_OPERATION_MODE_IMU)))
							if ((! MyIMU->Write((uint8_t) BNO055_OPR_MODE_ADDR, (uint8_t) BNO055_OPERATION_MODE_NDOF)))
//									&& (! MyIMU->Write((uint8_t) BNO055_SYS_TRIGGER_ADDR, (uint8_t) BNO55_TRIGGER_EXTCLK)))
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
	pMyRobotState->IMU_ChipID = ((int) chipID) & 0xFF;
	pMyRobotState->IMU_CalStat = ((int) IMU_rcv_buff[0]) & 0xFF;
	pMyRobotState->IMU_SelfTest = ((int) IMU_rcv_buff[1]) & 0xFF;
	pMyRobotState->IMU_IntrStat = ((int) IMU_rcv_buff[2]) & 0xFF;
	pMyRobotState->IMU_ClockStat = ((int) IMU_rcv_buff[3]) & 0xFF;
	pMyRobotState->IMU_SystemStat = ((int) IMU_rcv_buff[4]) & 0xFF;
	pMyRobotState->IMU_SystemError = ((int) IMU_rcv_buff[5]) & 0xFF;
	}


void NAVIGATOR :: SetStartingPosition()
	{
	pMyRobotState->Robot_Position_X = pMyRobotState->Auto_Start_X;
	pMyRobotState->Robot_Position_Y = pMyRobotState->Auto_Start_Y;
	pMyRobotState->Robot_Heading = pMyRobotState->Auto_Start_Heading;
	pMyRobotState->Drivetrain_Heading = pMyRobotState->Robot_Heading;
	pMyRobotState->Robot_Direction = 0;
	pMyRobotState->Robot_Speed = 0;
	pMyRobotState->Nav_Robot_Lost = false;
	pMyRobotState->Nav_Gyro_Tilt = false;
	}

void NAVIGATOR :: Init(RobotStateBuffer *pRobotState, INPUT *pInput)
	{
	pMyRobotState = pRobotState;
	pMyInput = pInput;
	SetStartingPosition();
	NavTimer.Reset();
	initcnt = 0;
	InitMyIMU();
	INS_Calibrating = true;
	Wait(1);				// Wait for new sensor values to stabilize
	ReadThermostat();
//	ReadGyro();
	ReadAccel();
	for (SampleCount=1; (SampleCount < 5) || ((ReadEuler() > 0.0001) && (SampleCount < 20)); SampleCount++) SmartDashboard::PutNumber("EulerStabilizationCount",SampleCount);;	// read gyro until it stabilizes
	for (SampleCount=1; (SampleCount < 5) || ((ReadGyro() > 0.0001) && (SampleCount < 20)); SampleCount++) SmartDashboard::PutNumber("GyroStabilizationCount",SampleCount);;	// read gyro until it stabilizes
	SampleCount = 0;		// Use the first set of values to set center points.
	NavTimer.Start();
	for (int i=1; (i < 200) && (fabs(NavTimer.Get()) < .000001); i++) Wait (.000001);  // wait for timer to start running
	INS_semaphore = initializeMutexNormal();
	pReadSensors->StartPeriodic(SAMPLEPERIOD);	// schedule periodic sampling of Gyro and Accel data
	pINSupdate->StartPeriodic(INSUPDATEPERIOD);	// schedule periodic updates of the Robot position, orientation, speed and direction of travel
	pMyRobotState->Navigator_Init = initcnt;
	}

void NAVIGATOR :: ReadIMUCalStat()
	{
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


void NAVIGATOR :: ReadThermostat()
	{
	int8_t	temp;
	memset(IMU_rcv_buff, 0, sizeof(IMU_rcv_buff));
	MyIMU->Read(BNO055_TEMP_ADDR, 1, IMU_rcv_buff);
	pMyRobotState->Nav_Temp = (temp = (uint8_t) IMU_rcv_buff[0]);
	NAVtemperature = pMyRobotState->Nav_Temp - pMyRobotState->Nav_IMU_Temp_Center;
	/*
	 * Rather than automagically detecting the "Normal" (Center) value of the temperature sensor, we want to
	 * hard-code the Center value that corresponds to zero compensation from the prescribed sensor sensivities.
	 * Per the book, that is 25 degrees Celsius.
	 */
	}

double NAVIGATOR :: ReadEuler()
	{
	int16_t	Euler_Heading, Euler_Roll, Euler_Pitch;
	double	Hdelta, Rdelta, Pdelta;
	double	heading, roll, pitch;

	memset(IMU_rcv_buff, 0, sizeof(IMU_rcv_buff));
	MyIMU->Read(BNO055_EULER_H_LSB_ADDR, GYRO_READ_CNT, IMU_rcv_buff);
	Heading = (Euler_Heading = ((uint16_t) IMU_rcv_buff[1] << 8) | ((uint16_t) IMU_rcv_buff[0])) / GYRO_RADIANS;
	Roll = (Euler_Roll = ((uint16_t) IMU_rcv_buff[3] << 8) | ((uint16_t) IMU_rcv_buff[2])) / GYRO_RADIANS;
	Pitch = (Euler_Pitch = ((uint16_t) IMU_rcv_buff[5] << 8) | ((uint16_t) IMU_rcv_buff[4])) / GYRO_RADIANS;
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
	return fabs(Hdelta);
}

double NAVIGATOR :: ReadGyro()
	{
	int16_t	Xrate, Yrate, Zrate;
	double	x, y, z;

	memset(IMU_rcv_buff, 0, sizeof(IMU_rcv_buff));
	MyIMU->Read(BNO055_GYRO_DATA_X_LSB_ADDR, GYRO_READ_CNT, IMU_rcv_buff);
 	x = ((Xrate = ((uint16_t) IMU_rcv_buff[1] << 8) | ((uint16_t) IMU_rcv_buff[0]))) - pMyRobotState->Nav_Gyro_X_Center;
	y = ((Yrate = ((uint16_t) IMU_rcv_buff[3] << 8) | ((uint16_t) IMU_rcv_buff[2]))) - pMyRobotState->Nav_Gyro_Y_Center;
	z = ((Zrate = ((uint16_t) IMU_rcv_buff[5] << 8) | ((uint16_t) IMU_rcv_buff[4]))) - pMyRobotState->Nav_Gyro_Z_Center;

	// Squelch noise
	x *= (fabs(x / pMyRobotState->Nav_Gyro_X_Sensitivity) < pMyRobotState->Nav_Gyro_Noise_Base) ? pMyRobotState->Nav_Gyro_Squelch : 1;
	y *= (fabs(y / pMyRobotState->Nav_Gyro_Y_Sensitivity) < pMyRobotState->Nav_Gyro_Noise_Base) ? pMyRobotState->Nav_Gyro_Squelch : 1;
	z *= (fabs(z / pMyRobotState->Nav_Gyro_Z_CW_Sensitivity) < pMyRobotState->Nav_Gyro_Noise_Base) ? pMyRobotState->Nav_Gyro_Squelch : 1;

	if (INS_Calibrating)
		{
		if (fmax(fmax(fabs(x),fabs(y)),fabs(z)) > pMyRobotState->Nav_Gyro_Noise_Limit)
			{
			//	The robot got moved during the calibration sequence.
			pMyRobotState->Nav_Gyro_Noisy_Count++;
			}
		// set the center value to the average of all raw values sampled during calibration
		pMyRobotState->Nav_Gyro_X_Center = ((pMyRobotState->Nav_Gyro_X_Center * (double) (SampleCount - 1)) + x) / (double) SampleCount;
		pMyRobotState->Nav_Gyro_Y_Center = ((pMyRobotState->Nav_Gyro_Y_Center * (double) (SampleCount - 1)) + y) / (double) SampleCount;
		pMyRobotState->Nav_Gyro_Z_Center = ((pMyRobotState->Nav_Gyro_Z_Center * (double) (SampleCount - 1)) + z) / (double) SampleCount;
		}

	// Scale for rate sensitivity
	/*
	 * According to the BNO055 manufacturer's guide, it should be 900 = 1 radian per second.
	 * But, I'm finding that value to be very far from reality.  It's coming out more like 804 = 1 radian per second.
	 * That's way out of tolerance.  And the scale is also not linear.
	 */
 	x /= pMyRobotState->Nav_Gyro_X_Sensitivity;
	y /= pMyRobotState->Nav_Gyro_Y_Sensitivity;
	// Also, compensate for non-linear spin sensitivity
	/*
	 * Sometimes gyro mems devices do not produce the same absolute signal level for the same rate of spin in opposite directions.
	 * Also, they do not always produce the same change in signal level for equal differences in spin rate at various spin rates.
	 */
	if (z > 0.0)
		{
		z /= (pMyRobotState->Nav_Gyro_Z_CW_Sensitivity + (z * pMyRobotState->Nav_Gyro_Z_CW_Spin_Effect));
		}
	else
		{
		z /= (pMyRobotState->Nav_Gyro_Z_CCW_Sensitivity + (z * pMyRobotState->Nav_Gyro_Z_CCW_Spin_Effect));
		}

	// Now compensate for the temperature
	// Typical BNO055 gyroscope rate sensivity to temperature deviation is +/-0.03% per degree Celsius
	pMyRobotState->Nav_Gyro_X_Raw = x * (1.0 + (NAVtemperature * GYRO_THERMO_SENSITIVITY));
	pMyRobotState->Nav_Gyro_Y_Raw = y * (1.0 + (NAVtemperature * GYRO_THERMO_SENSITIVITY));
	pMyRobotState->Nav_Gyro_Z_Raw = z * (1.0 + (NAVtemperature * GYRO_THERMO_SENSITIVITY));

	return (fabs(pMyRobotState->Nav_Gyro_X_Raw) + fabs(pMyRobotState->Nav_Gyro_Y_Raw) + fabs(pMyRobotState->Nav_Gyro_Z_Raw));
	}


void NAVIGATOR :: ReadAccel()
	{
	int16_t	XGs, YGs, ZGs;
	double	x, y, z, b;

	memset(IMU_rcv_buff, 0, sizeof(IMU_rcv_buff));
//	MyIMU->Read(BNO055_LINEAR_ACCEL_DATA_X_LSB_ADDR, GYRO_READ_CNT, IMU_rcv_buff);
	MyIMU->Read(BNO055_ACCEL_DATA_X_LSB_ADDR, GYRO_READ_CNT, IMU_rcv_buff);
	x = (XGs = ((uint16_t) IMU_rcv_buff[1] << 8) | ((uint16_t) IMU_rcv_buff[0])) - pMyRobotState->Nav_Acc_X_Center;
	y = (YGs = ((uint16_t) IMU_rcv_buff[3] << 8) | ((uint16_t) IMU_rcv_buff[2])) - pMyRobotState->Nav_Acc_Y_Center;
	z = (ZGs = ((uint16_t) IMU_rcv_buff[5] << 8) | ((uint16_t) IMU_rcv_buff[4])) - pMyRobotState->Nav_Acc_Z_Center;

	// Squelch noise
	b = pMyRobotState->Nav_Acc_Noise_Base * ACCEL_MPS2;
	x *= (fabs(x) < b) ? pMyRobotState->Nav_Acc_Squelch : 1;
	y *= (fabs(y) < b) ? pMyRobotState->Nav_Acc_Squelch : 1;
	z *= (fabs(z) < b) ? pMyRobotState->Nav_Acc_Squelch : 1;

	if (INS_Calibrating)
		{
		if (fmax(fmax(fabs(XGs),fabs(YGs)),fabs(ZGs)) > pMyRobotState->Nav_Acc_Noise_Limit)
			{
			//	The robot got moved during the calibration sequence.
			noisyAccelcnt++;
			}
		// set the center value to the average of all raw values sampled during calibration
		pMyRobotState->Nav_Acc_X_Center = ((pMyRobotState->Nav_Acc_X_Center * (double) (SampleCount - 1)) + x) / (double) SampleCount;
		pMyRobotState->Nav_Acc_Y_Center = ((pMyRobotState->Nav_Acc_Y_Center * (double) (SampleCount - 1)) + y) / (double) SampleCount;
		pMyRobotState->Nav_Acc_Z_Center = ((pMyRobotState->Nav_Acc_Z_Center * (double) (SampleCount - 1)) + z) / (double) SampleCount;
		}

	// Scale for rate sensitivity
 	x /= ACCEL_MPS2;
	y /= ACCEL_MPS2;
	z /= ACCEL_MPS2;

	// Now compensate for the temperature
	// Typical BNO055 gyroscope rate sensivity to temperature deviation is +/-0.03% per degree Celsius
	pMyRobotState->Nav_Acc_X_Raw = x * (1.0 + (NAVtemperature * ACCEL_THERMO_SENSITIVITY));
	pMyRobotState->Nav_Acc_Y_Raw = y * (1.0 + (NAVtemperature * ACCEL_THERMO_SENSITIVITY));
	pMyRobotState->Nav_Acc_Z_Raw = z * (1.0 + (NAVtemperature * ACCEL_THERMO_SENSITIVITY));
	}




void NAVIGATOR :: Push_Navigation_Data_Frame(double timestamp, double x_gyro, double y_gyro, double z_gyro, double x_accel, double y_accel, double z_accel)
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
	int8_t	temp;
	unsigned char reg = BNO055_ACCEL_DATA_X_LSB_ADDR;
	int16_t	Xrate, Yrate, Zrate, Xacc, Yacc, Zacc;
	double	gx, gy, gz, ax, ay, az, ab;

	uint8_t IMU_data_rcv_buff[45];

	memset(IMU_data_rcv_buff, 0, sizeof(IMU_data_rcv_buff));
	// Transaction(&registerAddress, sizeof(registerAddress), buffer, count);
	MyIMU->Transaction(&reg, sizeof(reg), &IMU_data_rcv_buff[0], 6);

	reg = BNO055_ACCEL_DATA_X_LSB_ADDR;
	MyIMU->Transaction(&reg, sizeof(reg), &IMU_data_rcv_buff[12], 6);

	reg = BNO055_ACCEL_DATA_X_LSB_ADDR;
	MyIMU->Transaction(&reg, sizeof(reg), &IMU_data_rcv_buff[44], 1);

	// pick off the temperature value
	pMyRobotState->Nav_Temp = (temp = (uint8_t) IMU_data_rcv_buff[44]);
	NAVtemperature = pMyRobotState->Nav_Temp - pMyRobotState->Nav_IMU_Temp_Center;

	// pick off the gyro rates for each axis
 	gx = ((Xrate = ((uint16_t) IMU_data_rcv_buff[13] << 8) | ((uint16_t) IMU_data_rcv_buff[12]))) - pMyRobotState->Nav_Gyro_X_Center;
	gy = ((Yrate = ((uint16_t) IMU_data_rcv_buff[15] << 8) | ((uint16_t) IMU_data_rcv_buff[14]))) - pMyRobotState->Nav_Gyro_Y_Center;
	gz = ((Zrate = ((uint16_t) IMU_data_rcv_buff[17] << 8) | ((uint16_t) IMU_data_rcv_buff[16]))) - pMyRobotState->Nav_Gyro_Z_Center;

	// Squelch noise
	gx *= (fabs(gx / pMyRobotState->Nav_Gyro_X_Sensitivity) < pMyRobotState->Nav_Gyro_Noise_Base) ? pMyRobotState->Nav_Gyro_Squelch : 1;
	gy *= (fabs(gy / pMyRobotState->Nav_Gyro_Y_Sensitivity) < pMyRobotState->Nav_Gyro_Noise_Base) ? pMyRobotState->Nav_Gyro_Squelch : 1;
	gz *= (fabs(gz / pMyRobotState->Nav_Gyro_Z_CW_Sensitivity) < pMyRobotState->Nav_Gyro_Noise_Base) ? pMyRobotState->Nav_Gyro_Squelch : 1;

	// pick off the accelerometer values for each axis
	ax = (Xacc = ((uint16_t) IMU_data_rcv_buff[1] << 8) | ((uint16_t) IMU_data_rcv_buff[0])) - pMyRobotState->Nav_Acc_X_Center;
	ay = (Yacc = ((uint16_t) IMU_data_rcv_buff[3] << 8) | ((uint16_t) IMU_data_rcv_buff[2])) - pMyRobotState->Nav_Acc_Y_Center;
	az = (Zacc = ((uint16_t) IMU_data_rcv_buff[5] << 8) | ((uint16_t) IMU_data_rcv_buff[4])) - pMyRobotState->Nav_Acc_Z_Center;

	// Squelch noise
	ab = pMyRobotState->Nav_Acc_Noise_Base * ACCEL_MPS2;
	ax *= (fabs(ax) < ab) ? pMyRobotState->Nav_Acc_Squelch : 1;
	ay *= (fabs(ay) < ab) ? pMyRobotState->Nav_Acc_Squelch : 1;
	az *= (fabs(az) < ab) ? pMyRobotState->Nav_Acc_Squelch : 1;

	if (INS_Calibrating)
		{
		if (fmax(fmax(fabs(gx),fabs(gy)),fabs(gz)) > pMyRobotState->Nav_Gyro_Noise_Limit)
			{
			//	The robot got moved during the calibration sequence.
			pMyRobotState->Nav_Gyro_Noisy_Count++;
			}
		// set the center value to the average of all raw values sampled during calibration
		pMyRobotState->Nav_Gyro_X_Center = ((pMyRobotState->Nav_Gyro_X_Center * (double) (SampleCount - 1)) + gx) / (double) SampleCount;
		pMyRobotState->Nav_Gyro_Y_Center = ((pMyRobotState->Nav_Gyro_Y_Center * (double) (SampleCount - 1)) + gy) / (double) SampleCount;
		pMyRobotState->Nav_Gyro_Z_Center = ((pMyRobotState->Nav_Gyro_Z_Center * (double) (SampleCount - 1)) + gz) / (double) SampleCount;

		if (fmax(fmax(fabs(ax),fabs(ay)),fabs(az)) > pMyRobotState->Nav_Acc_Noise_Limit)
			{
			//	The robot got moved during the calibration sequence.
			noisyAccelcnt++;
			}
		// set the center value to the average of all raw values sampled during calibration
		pMyRobotState->Nav_Acc_X_Center = ((pMyRobotState->Nav_Acc_X_Center * (double) (SampleCount - 1)) + ax) / (double) SampleCount;
		pMyRobotState->Nav_Acc_Y_Center = ((pMyRobotState->Nav_Acc_Y_Center * (double) (SampleCount - 1)) + ay) / (double) SampleCount;
		pMyRobotState->Nav_Acc_Z_Center = ((pMyRobotState->Nav_Acc_Z_Center * (double) (SampleCount - 1)) + az) / (double) SampleCount;
		}

	// Scale for rate sensitivity
	/*
	 * According to the BNO055 manufacturer's guide, it should be 900 = 1 radian per second.
	 * But, I'm finding that value to be very far from reality.  It's coming out more like 804 = 1 radian per second.
	 * That's way out of tolerance.  And the scale is also not linear.
	 */
 	gx /= pMyRobotState->Nav_Gyro_X_Sensitivity;
	gy /= pMyRobotState->Nav_Gyro_Y_Sensitivity;
	// Also, compensate for non-linear spin sensitivity
	/*
	 * Sometimes gyro mems devices do not produce the same absolute signal level for the same rate of spin in opposite directions.
	 * Also, they do not always produce the same change in signal level for equal differences in spin rate at various spin rates.
	 */
	if (gz > 0.0)
		{
		gz /= (pMyRobotState->Nav_Gyro_Z_CW_Sensitivity + (gz * pMyRobotState->Nav_Gyro_Z_CW_Spin_Effect));
		}
	else
		{
		gz /= (pMyRobotState->Nav_Gyro_Z_CCW_Sensitivity + (gz * pMyRobotState->Nav_Gyro_Z_CCW_Spin_Effect));
		}

 	ax /= ACCEL_MPS2;
	ay /= ACCEL_MPS2;
	az /= ACCEL_MPS2;

	// Now compensate for the temperature
	// Typical BNO055 gyroscope rate sensivity to temperature deviation is +/-0.03% per degree Celsius
	pMyRobotState->Nav_Gyro_X_Raw = gx * (1.0 + (NAVtemperature * GYRO_THERMO_SENSITIVITY));
	pMyRobotState->Nav_Gyro_Y_Raw = gy * (1.0 + (NAVtemperature * GYRO_THERMO_SENSITIVITY));
	pMyRobotState->Nav_Gyro_Z_Raw = gz * (1.0 + (NAVtemperature * GYRO_THERMO_SENSITIVITY));
	pMyRobotState->Nav_Acc_X_Raw = ax * (1.0 + (NAVtemperature * ACCEL_THERMO_SENSITIVITY));
	pMyRobotState->Nav_Acc_Y_Raw = ay * (1.0 + (NAVtemperature * ACCEL_THERMO_SENSITIVITY));
	pMyRobotState->Nav_Acc_Z_Raw = az * (1.0 + (NAVtemperature * ACCEL_THERMO_SENSITIVITY));


	return (fabs(pMyRobotState->Nav_Gyro_X_Raw) + fabs(pMyRobotState->Nav_Gyro_Y_Raw) + fabs(pMyRobotState->Nav_Gyro_Z_Raw)
			+ fabs(pMyRobotState->Nav_Acc_X_Raw) + fabs(pMyRobotState->Nav_Acc_Y_Raw) + fabs(pMyRobotState->Nav_Acc_Z_Raw));

	}



void NAVIGATOR :: ReadSensors()
	{
	DataClockTime = NavTimer.Get();
//	SampleCount++;
//	ReadIMUdata();
//	/* ReadIMUdata() replaces ReadThermostat(); ReadGyro(); ReadEuler(); and ReadAccel() with a single data fetch
	if (SampleCount++ % 100 == 0) ReadThermostat();
	ReadGyro();
//	ReadEuler();
	ReadAccel();
//	*/
	Push_Navigation_Data_Frame(DataClockTime, pMyRobotState->Nav_Gyro_X_Raw, pMyRobotState->Nav_Gyro_Y_Raw, pMyRobotState->Nav_Gyro_Z_Raw,
											 pMyRobotState->Nav_Acc_X_Raw, pMyRobotState->Nav_Acc_Y_Raw, pMyRobotState->Nav_Acc_Z_Raw);
	pMyRobotState->dataspeed = (NavTimer.Get() - DataClockTime) * 1000;	// exec time of this method in msec
//	pMyRobotState->dataspeed = ((pMyRobotState->dataspeed * (double) (SampleCount - 1)) + SampleRunTime) / (double) SampleCount;
	pMyRobotState->datarate = int(1 / fmax(0.001, DataClockTime - DataCollectTime));  // avoid divide by zero crash
	DataCollectTime = DataClockTime;
	}


//  schedule this to run independently every 0.1 milliseconds
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
	pMyRobotState->Robot_Heading += (NAVz_gyro * NavElapsedTime);
	pMyRobotState->Robot_Pitch += (NAVx_gyro * NavElapsedTime);
	pMyRobotState->Robot_Roll += (NAVy_gyro * NavElapsedTime);
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

	pMyRobotState->Robot_Pitch += pMyRobotState->Robot_Pitch > PI ? -1 * TWO_PI : pMyRobotState->Robot_Pitch <= -1 * PI ? TWO_PI : 0;
	if (fabs(NAVx_gyro) > GYRO_RATELIMIT)
		{
		pMyRobotState->Nav_Gyro_Tilt = true;
		}

	pMyRobotState->Robot_Heading += pMyRobotState->Robot_Heading >= TWO_PI ? -1 * TWO_PI : pMyRobotState->Robot_Heading < -0.0 ? TWO_PI : 0;
	if (fabs(NAVz_gyro) > GYRO_RATELIMIT)
		{
		pMyRobotState->Nav_Gyro_Tilt = true;
		}
	}


void NAVIGATOR :: UpdatePosition()
	{
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


	pMyRobotState->Robot_Direction = 90 - (atan2(Y_dist, X_dist) / DEGREES_TO_RADIANS);
	pMyRobotState->Robot_Direction += pMyRobotState->Robot_Direction < 0 ? 360 : 0;
	pMyRobotState->Robot_Speed = pow((pow(pMyRobotState->Nav_X_Speed,2) + pow(pMyRobotState->Nav_Y_Speed,2)),0.5);
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


