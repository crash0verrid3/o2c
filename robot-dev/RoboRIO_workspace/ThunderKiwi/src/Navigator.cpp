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
	spin_rate = 0;
	sample_counter = 0;
	NavClockTime = 0;
	temperature = 0;
	NavElapsedTime = NAVtemperature = NAVgyro_spin = NAVx_accel = NAVy_accel = 0;

	Navigation_Status_Buffer.Xcoord = Navigation_Status_Buffer.Ycoord
									= Navigation_Status_Buffer.Xspeed
									= Navigation_Status_Buffer.Yspeed
									= Navigation_Status_Buffer.Timestamp = 0;

	MyGyro = new AnalogInput(GYRO_ANALOGCHANNEL);
	cw_sensitivity = GYRO_ANALOG_CW_SENSITIVITY / 1000000000;		// Gyro is not consistent between clockwise turns and counter-clockwise turns
	ccw_sensitivity = GYRO_ANALOG_CCW_SENSITIVITY / 1000000000;	// Up to about 0.5 degrees per revolution difference between them
	GyroNoiseLimit = GYRO_ANALOG_NOISELIMIT;
	GyroCenter = GYRO_ANALOG_RATECENTER / 1000;

	MyThermostat = new AnalogInput(THERMOSTAT_ANALOGCHANNEL);

	GyroSensitivity = (cw_sensitivity + ccw_sensitivity) / 2;
	NavLoopCnt = 0;
	SampleLoopTime = 0;
	SampleElapsedTime = 0;
	SampleRunTime = 0;
	GyroRawRate = 0;
	GyroInitCenter = 0;
	GyroSample = 0;
	GyroHeading = 0;	// actually want to read from SmartDashboard 'cause we might want to start the robot facing otherwise
	RobotDirection = 0;
	GyroTilt = false;
	noisyGyrocnt = 0;
	noisyAccelcnt = 0;
	gyro_accumulator_count = 0;
	dataNotRdyCnt = 0;
	prevcount = 0;
	accumulator = 0;
	prevaccum = 0;
	SpinEffectCW = 0;
	SpinEffectCCW = 0;
	Sensitivity_Offset = 0;		// we'll use this later to adjust sensitivity as a function of temperature (it does matter)

	}


void NAVIGATOR :: InitMyGyro()
	{
	/*
	 * Note: There is a flaw in either the accumulator firmware or in the WPILib accumulator class.
	 * 		If you set any oversamplebits or average bits and then routinely call the ResetAccumulator method,
	 * 		several BAD things happen.  First, the ResetAccumulator method waits until the next oversampled/average
	 * 		data value is ready to be added to the accumulator.  Then, the accumulator is zeroed.  This means that
	 * 		the more oversamplebits and averagebits you set, the longer it takes for the ResetAccumulator method
	 * 		to complete.  Second, when it does complete, it throws out the data that was suppose to get added to
	 * 		the accumulator.
	 *
	 * 		So...if you want the hardware to do the heavy lifting (saving precious CPU cycles for other tasks)
	 * 		you cannot use the ResetAccumulator method between collecting datapoints.  You will actually lose more
	 * 		data than the oversampling and averaging functions were intended to preserve.
	 */
	// Get the gyro configuration parameters from the SmartDashboard.
	int oversamplebits = SmartDashboard::GetNumber("OversampleBits",GYRO_OVERSAMPLEBITS);
	int avgbits = SmartDashboard::GetNumber("AvgBits",GYRO_SAMPLEAVGBITS);
	double samplerate = SmartDashboard::GetNumber("SampleRate",GYRO_SAMPLERATE);

	// Get the gyro tuning parameters from the SmartDashboard.
	GyroCenter = GyroInitCenter = (SmartDashboard::GetNumber("GyroInitCenter",GYRO_ANALOG_RATECENTER) / 1000);
	GyroNoiseLimit = SmartDashboard::GetNumber("GyroNoiseLimit",GYRO_ANALOG_NOISELIMIT);
	// Did I mention that the gyro doesn't give the same
	// voltage shift / dps for clockwise and counter-clockwise.
	// In other words, the voltage shift does not appear to be entirely linear.
	// It's just a teeny, tiny bit off.
	cw_sensitivity = SmartDashboard::GetNumber("GyroSensitivityCW",GYRO_ANALOG_CW_SENSITIVITY) / 1000000000;
	ccw_sensitivity = SmartDashboard::GetNumber("GyroSensitivityCCW",GYRO_ANALOG_CCW_SENSITIVITY) / 1000000000;
	GyroSensitivity = (cw_sensitivity + ccw_sensitivity) / 2;
	// Did I mention that, if the gyro is not placed in the exact center of rotation
	// of the robot, then the above mentioned issue is further exagerated as speed
	// of rotation increases.
	SpinEffectCW = SmartDashboard::GetNumber("SpinEffectCW",GYRO_ANALOG_CW_SPIN_CORRECTION) / 1000000000000;
	SpinEffectCCW = SmartDashboard::GetNumber("SpinEffectCCW",GYRO_ANALOG_CCW_SPIN_CORRECTION) / 1000000000000;

	// Configure and initialize the gyro
	MyGyro->SetOversampleBits(oversamplebits);
	MyGyro->SetAverageBits(avgbits);
	MyGyro->SetSampleRate(samplerate);
	MyGyro->InitAccumulator();
	Wait (1);						// wait for the gyro to run its internal calibration sequence
	MyGyro->ResetAccumulator();		// then clear the garbage from the accumulator
	Wait (1);						// wait for the gyro to run its internal calibration sequence

	// If these parameters are not currently on the SmartDashboard, the values will default
	// to pre-programmed values.  Write out those values so the operator can change them to
	// whatever they want and then restart the RoboRIO User Program to pick up the new values.
	SmartDashboard::PutNumber("OversampleBits",oversamplebits);
	SmartDashboard::PutNumber("AvgBits",avgbits);
	SmartDashboard::PutNumber("SampleRate",samplerate);
	SmartDashboard::PutNumber("GyroInitCenter",GyroCenter * 1000);
	SmartDashboard::PutNumber("GyroNoiseLimit",GyroNoiseLimit);
	SmartDashboard::PutNumber("GyroSensitivityCW",cw_sensitivity * 1000000000);
	SmartDashboard::PutNumber("GyroSensitivityCCW",ccw_sensitivity * 1000000000);
	SmartDashboard::PutNumber("SpinEffectCW",SpinEffectCW * 1000000000000);
	SmartDashboard::PutNumber("SpinEffectCCW",SpinEffectCCW * 1000000000000);
	}



void NAVIGATOR :: Init(RobotStateBuffer *pRobotState)
	{
	initcnt++;
	Wait (1);		// had some problems with race condition initializing SmartDashboard and then
					// calling GetNumber too soon.  Fixed by waiting 1 second.
	SmartDashboard::PutNumber("NavInit", initcnt);
	pMyRobotState = pRobotState;
	pMyRobotState->Navigation_Position_X = 0;
	pMyRobotState->Navigation_Position_Y = 0;
	pMyRobotState->Navigation_Direction = 360;
	pMyRobotState->Navigation_Speed = 0;
	pMyRobotState->Navigation_Robot_Lost = false;
	NavTimer.Reset();
	InitMyGyro();
	NavTimer.Start();
	for (int i=1; (i < 200) && (fabs(NavTimer.Get()) < .000001); i++) Wait (.000001);  // wait for timer to start running

	INS_Calibrating = true;
	INS_semaphore = initializeMutexRecursive();
	//	here we will schedule the Gyro Data Collector interrupt service
	pReadSensors->StartPeriodic(SAMPLEPERIOD);
	//	here we will schedule the Robot Navigation Update interrupt service
	pINSupdate->StartPeriodic(INSUPDATEPERIOD);

	initcnt++;
	SmartDashboard::PutNumber("NavInit", initcnt);
	}




int NAVIGATOR :: ReadGyro()
	{
	gyro_accumulator_count = 1;
	MyGyro->GetAccumulatorOutput(&accumulator, &gyro_accumulator_count);
	gyro_accumulator_count -= prevcount;
	accumulator -= prevaccum;
	if (gyro_accumulator_count == 0)
		{
		// there is no new data available from the gyro
		// there are two ways to handle this
		//	1. don't record any sensor data points
		//		This means that the next sensor values will be counted for this cycle and the next
		//	2. just return the previous value.
		//		I like this option best because, if we get values from the accelerometer, we can
		//		apply them.  But, should we?
		// Actually, this should not be allowed to happen.  We should be configuring the sensors
		// for a samplerate that will guarantee at least one new value everytime we check.
		dataNotRdyCnt++;
		return 0;
		}
	spin_rate = (double) (((double) accumulator) / ((double) gyro_accumulator_count)) - GyroCenter;
	if (INS_Calibrating)
		{
		if (fabs(spin_rate) > GyroNoiseLimit)
			{
			//	The robot got moved during the calibration sequence.
			noisyGyrocnt++;
			}
		GyroCenter += (double) (10 * spin_rate / ((NavClockTime * 1000) + 10.0));		// adjust GyroCenter by 0.1% with each pass
		// I've found it to be easier and more reliable to keep track of the center point
		// in this method rather than centering the values in the accumulator
		//#ifdef MYGYROANALOG
		//			MyGyro->SetAccumulatorCenter(int(GyroCenter));
		//#endif
		//#ifdef MYGYRO
		//	MyGyro->m_center = GyroCenter;
		//#endif
		}
	if (spin_rate > 0)	// going clockwise
		{
		spin_rate = spin_rate * (cw_sensitivity + (spin_rate * SpinEffectCW));		// calculate spin_rate
		}
	else
		{
		spin_rate = spin_rate * (ccw_sensitivity + (spin_rate * SpinEffectCCW));	// calculate spin_rate
		}
	prevcount += gyro_accumulator_count;
	prevaccum += accumulator;
	return 1;	// using the fp accumulator, we will return 1 value per cycle
				// using an I2C gyro with FIFO, we might have multiple values queued up
	}



void NAVIGATOR :: Push_Navigation_Data_Frame(double timestamp, double gyro_spin, int temperature, int x_accel, int y_accel)
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
		Navigation_Data_Queue[Navigation_Data_Queue_Top].temperature = temperature;
		Navigation_Data_Queue[Navigation_Data_Queue_Top].gyro_spin = gyro_spin;
		Navigation_Data_Queue[Navigation_Data_Queue_Top].x_accel = x_accel;
		Navigation_Data_Queue[Navigation_Data_Queue_Top].y_accel = y_accel;
			{
			Synchronized sync(INS_semaphore);	// we are going to update values referenced by other procedure.
			Navigation_Data_Queue_Depth++;
			}
		}
	}


void NAVIGATOR :: ReadSensors()
	{
	NavClockTime = NavTimer.Get();

	if (ReadGyro() >= 0)
		{
//		Push_Navigation_Data_Frame(NavClockTime, spin_rate, temperature, x_axis, y_axis);
		Push_Navigation_Data_Frame(NavClockTime, spin_rate, 0, 0, 0);
		}

	if (sample_counter++ % 10 == 0)
		{
		SampleElapsedTime = NavClockTime;
		}
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
	if (INS_Calibrating)
		{
		Synchronized sync(INS_semaphore);	// we are going to update values referenced by other procedure.
		INS_Calibrating = false;
//		NavTimer.Reset();					// either reset the NavTimer, or update the NavStartTime
		//NavStartTime = NavTimer.Get();	// I chose to reset the NavTimer, which keeps the math very simple
		//	GyroCenter = gc + GYRO_ZEROSHIFT;
		//GyroDrift = 0;
		GyroHeading = 0;			// This should be specified by the autonomous program selection
//		NavLoopCnt = 0;
		// for (i=1; (i < 20) && (NavStartTime = NavTimer.Get()) < .001; i++);

		Navigation_Data_Queue_Bottom = Navigation_Data_Queue_Top;  // Clear the data input queue
		Navigation_Status_Buffer.Xcoord = 0;			// This should be specified by the autonomous program selection
		Navigation_Status_Buffer.Ycoord = 0;			// This should be specified by the autonomous program selection
		Navigation_Status_Buffer.Xspeed = 0;
		Navigation_Status_Buffer.Yspeed = 0;
		Navigation_Status_Buffer.Timestamp = NavTimer.Get();
		Push_Navigation_Data_Frame(Navigation_Status_Buffer.Timestamp, GyroCenter, temperature, 0, 0);
		}
	}


bool NAVIGATOR :: Pop_Navigation_Data_Frame()
	{
	int queue_depth;
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
	NAVtemperature = Navigation_Data_Queue[Navigation_Data_Queue_Bottom].temperature;
	NAVgyro_spin = Navigation_Data_Queue[Navigation_Data_Queue_Bottom].gyro_spin;
	NAVx_accel = Navigation_Data_Queue[Navigation_Data_Queue_Bottom].x_accel;
	NAVy_accel = Navigation_Data_Queue[Navigation_Data_Queue_Bottom].y_accel;
		{
		Synchronized sync(INS_semaphore);	// we are going to update values referenced by other procedure.
		Navigation_Data_Queue_Depth--;
		}
	return true;
	}



void NAVIGATOR :: UpdateHeading()
	{
	// NAVspin is already converted to degrees, so just add it onto the GyroHeading
	GyroHeading += NAVgyro_spin;
	GyroHeading += GyroHeading > 360 ? -360 : GyroHeading < 0 ? 360 : 0;
	pMyRobotState->Navigation_Heading = GyroHeading;
	}


// schedule this to run independently every 10 milliseconds
void NAVIGATOR :: UpdateINS()
	{
	while (Pop_Navigation_Data_Frame())
		{
		UpdateHeading();
		}
	}


//  schedule this to run independently every 10 milliseconds
void NAVIGATOR :: CallUpdateINS(void *controller)
	{
	NAVIGATOR *ins_update = (NAVIGATOR*) controller;
	ins_update->UpdateINS();
	}


