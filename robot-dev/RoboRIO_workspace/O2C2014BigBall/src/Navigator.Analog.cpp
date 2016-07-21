#include "Navigator.Analog.h"


NAVIGATOR :: NAVIGATOR() :	MyGyro(GYRO_ANALOGCHANNEL),
							gyro_semaphore(0), accel_semaphore(0), INS_semaphore(0)
	{
	initcnt = 0;
	Gyro_Calibrating = false;
	Accel_Calibrating = false;
	NavLoopCnt = 0;
	NavElapsedTime = 0;
	GyroRawRate = 0;
	GyroCenter = 0;
	GyroSample = 0;
	//GyroDrift = 0;
	GyroHeading = 0;	// actually want to read from SmartDashboard 'cause we might want to start the robot facing otherwise
	GyroTilt = false;
	RobotHeading = 0;
	noisycnt = 0;
	Gyro_VoltageCenter = GYRO_CENTERVOLTAGE;
	//Calibrated_GyroVoltageCenter = 0;
	GyroOversample = 0;
	OversampleCount = 0;
//	CalibrateGyro = new Notifier(NAVIGATOR::CallCageGyro,this);
//	CalibrateAccel = new Notifier(NAVIGATOR::CallCageAccel,this);
	SampleGyro = new Notifier(NAVIGATOR::CallOversampleGyro,this);
//	SampleAccel = new Notifier(NAVIGATOR::CallOversampleAccel,this);
	INSupdate = new Notifier(NAVIGATOR::CallCalculate,this);
	}


void NAVIGATOR :: InitMyGyro()
	{
//	SmartDashboard::PutNumber("Sensitivity", GYRO_SENSITIVITY * 1000);
	MyGyro.SetOversampleBits(GYRO_OVERSAMPLEBITS);
	MyGyro.SetAverageBits(GYRO_SAMPLEAVGBITS);
//	SmartDashboard::PutNumber("GyroOverSample",MyGyro.GetOversampleBits());		// = GYRO_OVERSAMPLEBITS;
//	SmartDashboard::PutNumber("GyroSampleAvg",MyGyro.GetAverageBits());			// = GYRO_SAMPLEAVGBITS;
//	SmartDashboard::PutNumber("NoiseOverSample",Noise.GetOversampleBits());		// = GYRO_OVERSAMPLEBITS;
//	SmartDashboard::PutNumber("NoiseSampleAvg",Noise.GetAverageBits());			// = GYRO_SAMPLEAVGBITS;
	Wait (1);
	
	//	here we will schedule the Gyro Data Collector interrupt service
	//  and the Gryo cage interrupt service
//	gyro_semaphore = semMCreate(SEM_Q_PRIORITY);
	gyro_semaphore = initializeMutexRecursive();
	SampleGyro->StartPeriodic(SAMPLEPERIOD);
	Gyro_Calibrating = true;
//	Wait (1);
//	CalibrateGyro->StartPeriodic(CALIBRATEPERIOD);
	}


void NAVIGATOR :: InitMyAccelerometer()
	{
	Accel_Calibrating = true;
//	SampleAccel->StartPeriodic(SAMPLEPERIOD);
//	CalibrateGyro->StartPeriodic(CALIBRATEPERIOD);
	}


void NAVIGATOR :: Init()
	{
	initcnt++;
	SmartDashboard::PutNumber("NavInit", initcnt);
	NavTimer.Reset();
	InitMyGyro();
	InitMyAccelerometer();
	NavTimer.Start();
	for (int i=1; (i < 200) && (fabs(NavTimer.Get()) < .000001); i++) Wait (.000001);  // wait for timer to start running
	NavLoopCnt = 0;
	initcnt++;
	SmartDashboard::PutNumber("NavInit", initcnt);
	//	here we will schedule the Robot Navigation Update interrupt service
	INS_semaphore = initializeMutexRecursive();
	INSupdate->StartPeriodic(INSUPDATEPERIOD);
	}


void NAVIGATOR :: OversampleGyro()
	{
	double gvc;
//	CRITICAL_REGION(gyro_semaphore)		// we need to reference values that are updated outside of this procedure
//		{								// and we don't want those values getting changed while we are referencing them
		gvc = Gyro_VoltageCenter;
//		}
//	END_REGION;
	GyroOversample += (MyGyro.GetVoltage() - gvc);
	if (OversampleCount++ == GYRO_OVERSAMPLECNT)
		{
			{
			Synchronized sync(gyro_semaphore);	// we are going to update values referenced by other procedure.
			GyroRawRate = GyroOversample / (double) OversampleCount;
			}
		if (fabs(GyroRawRate) > GYRO_NOISELIMIT)
			{
			//	The robot got moved during the calibration sequence.
//			GyroRawRate = GyroCenter;
			noisycnt++;
			}
		else if (Gyro_Calibrating)
			{
			//GyroCenter = ((GyroCenter * 1000.0) + cgr) / 1001.0;	// adjust GyroCenter by 0.1% with each pass
			Gyro_VoltageCenter += (double) (GyroRawRate / 100.0);					// adjust Gyro_VoltageCenter by 1% with each pass
			//Calibrated_GyroVoltageCenter = Gyro_VoltageCenter;		// stops updating this when match play begins
			}
		GyroOversample = 0;
		OversampleCount = 0;
		SmartDashboard::PutNumber("RawGyro", GyroRawRate);
		SmartDashboard::PutNumber("NoisyGyroCount",noisycnt);
		SmartDashboard::PutNumber("GyroVoltageCenter", Gyro_VoltageCenter * 1000);
		SmartDashboard::PutBoolean("GyroCalibrating",Gyro_Calibrating);
		}
	}

//  schedule this to run independently every 0.1 milliseconds
void NAVIGATOR :: CallOversampleGyro(void *controller)
	{
	NAVIGATOR *sample_gyro = (NAVIGATOR*) controller;
	sample_gyro->OversampleGyro();
	}


void NAVIGATOR :: OversampleAccel()
	{
	}

//  schedule this to run independently every 0.1 milliseconds
void NAVIGATOR :: CallOversampleAccel(void *controller)
	{
	NAVIGATOR *sample_accel = (NAVIGATOR*) controller;
	sample_accel->OversampleAccel();
	}



void NAVIGATOR :: Zero()
	{
//	CalibrateGyro->Stop();				//	Stop the Cage scheduler;
	if (Gyro_Calibrating)
		{
		Synchronized sync(gyro_semaphore);	// we are going to update values referenced by other procedure.
		Gyro_Calibrating = false;
		NavTimer.Reset();					// either reset the NavTimer, or update the NavStartTime
		//NavStartTime = NavTimer.Get();	// I chose to reset the NavTimer, which keeps the math very simple
		//	GyroCenter = gc + GYRO_ZEROSHIFT;
		//GyroDrift = 0;
		GyroHeading = 0;	// actually want to read from SmartDashboard 'cause we might want to start the robot facing otherwise
		NavLoopCnt = 0;
		// for (i=1; (i < 20) && (NavStartTime = NavTimer.Get()) < .001; i++);
		}
//	SmartDashboard::PutNumber("GyroCenter0", GyroCenter);
//	SmartDashboard::PutNumber("GyroVoltageCenter0", Gyro_VoltageCenter);
//		Accel_Calibrating = false;
	}


void NAVIGATOR :: UpdatePosition()
	{
	
	}


void NAVIGATOR :: UpdateHeading()
	{
	double grr, gc;
	CRITICAL_REGION(gyro_semaphore)		// we need to reference values that are updated outside of this procedure
		{								// and we don't want those values getting changed while we are referencing them
		grr = GyroRawRate;
		gc = GyroCenter;
		}
	END_REGION;
	GyroHeading += (double) ((grr - gc) * GYRO_DEGREESPERVOLT);
	GyroHeading += GyroHeading > 360 ? -360 : GyroHeading < 0 ? 360 : 0;
	if (fabs(grr) > GYRO_RATELIMIT)
		{
		GyroTilt = true;
		}

	if (NavLoopCnt % 10 == 0)		// update SmartDashboard every 10 loops
		{
//		SmartDashboard::PutNumber("GyroCenter", gc);
		SmartDashboard::PutNumber("RawGyroRate", grr);
		SmartDashboard::PutNumber("HeadingRate", (grr - gc) * GYRO_DEGREESPERVOLT);
		SmartDashboard::PutNumber("GyroHeading", GyroHeading); // * NavElapsedTime / (double) NavLoopCnt);
		}
	}


// schedule this to run independently every 10 milliseconds
void NAVIGATOR :: Calculate()
	{
	CRITICAL_REGION(gyro_semaphore)		// we need to reference values that are updated outside of this procedure
		{								// and we don't want those values getting changed while we are referencing them
		NavElapsedTime = NavTimer.Get();
		}
	END_REGION;
	NavLoopCnt++;
	UpdateHeading();
	UpdatePosition();
		{
		Synchronized sync(INS_semaphore);	// we are going to update values referenced by other procedure.
//		RobotHeading = GyroHeading * NavLoopCnt / NavElapsedTime;
		RobotHeading = GyroHeading;
		}
	if (NavLoopCnt % 10 == 0)		// update SmartDashboard every 10 loops
		{
		SmartDashboard::PutNumber("RobotHeading", RobotHeading); // * NavElapsedTime / (double) NavLoopCnt);
		}
	}


//  schedule this to run independently every 1 milliseconds
void NAVIGATOR :: CallCalculate(void *controller)
	{
	NAVIGATOR *ins_update = (NAVIGATOR*) controller;
	ins_update->Calculate();
	}


double NAVIGATOR :: GetRobotHeading ()
	{
	CRITICAL_REGION(INS_semaphore)		// we need to reference values that are updated outside of this procedure
		{								// and we don't want those values getting changed while we are referencing them
		return RobotHeading;
		}
	END_REGION;
	}
