#ifndef Navigator_H
#define Navigator_H
//mid-level actions

#include "Definitions.h"
#include "math.h"
#include "WPILib.h"
//#include "HAL/Semaphore.hpp"
#include "Notifier.h"

class Notifier;
class NAVIGATOR
{

public:
	NAVIGATOR();
	void Init();
	void Zero();
	double GetRobotHeading();
	
private:
	AnalogInput MyGyro;

	Notifier *SampleGyro;		// independent loop to sample gyro at defined intervals
	Notifier *SampleAccel;		// independent loop to sample accelerometer at defined intervals
	Notifier *INSupdate;		// independent loop to update Inertial Navigation System (position, orientation, speed, spin) at defined intervals

	Timer NavTimer;

	void InitMyGyro();
	void InitMyAccelerometer();
	void OversampleGyro();
	static void CallOversampleGyro(void *controller);
	void OversampleAccel();
	static void CallOversampleAccel(void *controller);
//	static void CallCageGyro(void *controller);
//	static void CallCageAccel(void *controller);
//	void CageGyro();
//	void CageAccelerometer();
//	void Cage();
	void UpdatePosition();
	void UpdateHeading();
	void Calculate();			// Calculate Position, Heading, Speed, Spin
	static void CallCalculate(void *controller);

	MUTEX_ID gyro_semaphore, accel_semaphore, INS_semaphore;
	
	bool	Gyro_Calibrating, Accel_Calibrating;

	bool GyroTilt;
	int noisycnt;
	double GyroHeading;	// the robot's heading calculated by correcting GyroRawAngle for the drift

	double GyroRawRate;
	double GyroCenter;
	double GyroOversample;
	int OversampleCount;
	double Gyro_VoltageCenter;
	//double Calibrated_GyroVoltageCenter;
	double GyroSample;
	double RobotHeading;

	int initcnt;
	int NavLoopCnt;

//	double NavStartTime;
	double NavElapsedTime;


};


#endif
