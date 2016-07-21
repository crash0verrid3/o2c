#ifndef Navigator_H
#define Navigator_H
//mid-level actions

//#define MYGYROI2C
#define MYGYROANALOG
//#define MYGYRO

#include <Definitions.h>
#include "math.h"
#include "WPILib.h"
//#include "HAL/Semaphore.hpp"
#include "Notifier.h"

class Notifier;

class NAVIGATOR
{

public:
	NAVIGATOR();
	void Init(RobotStateBuffer *pMyRobotState);
	void Zero();

private:
	AnalogInput *MyGyro;
	AnalogInput *MyThermostat;

	Notifier *pReadSensors;	// independent loop to sample gyro and accelerometer at defined intervals
	Notifier *pINSupdate;	// independent loop to update Inertial Navigation System (position, orientation, speed, spin) at defined intervals

	Timer NavTimer;

	static void CallReadSensors(void *controller);
	static void CallUpdateINS(void *controller);
	void InitMyGyro();
	int ReadGyro();
	void ReadSensors();
	void Push_Navigation_Data_Frame(double timestamp, double gyro_spin, int temperature, int x_accel, int y_accel);
	bool Pop_Navigation_Data_Frame();
	void UpdateHeading();
	void UpdateINS();

	struct	Navigation_Data_Frame	// a frame of Navigation data collected from the gyro and accelerometer every 1 msec
		{					// Note: data points are robot oriented, not field oriented
		double	timestamp;		// elapsed clock time in microseconds since the last data point
		double	gyro_spin;		// the number of microdegrees that the Robot turned since the last data point
		int		temperature;	// gyro temp impacts sensitivity
		int		x_accel;		// first X-axis acceleration
		int		y_accel;		// first y-axis acceleration
		} Navigation_Data_Queue[NAVIGATION_QUEUE_SIZE];
	int Navigation_Data_Queue_Top, Navigation_Data_Queue_Bottom;
	int Navigation_Data_Queue_Depth, Navigation_Data_Frame_Size;

	struct	Navigation_Status
		{
		double	Timestamp;
		double	Xcoord;
		double	Ycoord;
		double	Xspeed;
		double	Yspeed;
		}	Navigation_Status_Buffer;

	bool	INS_Calibrating;
	double	spin_rate, NavClockTime;
	uint	gyro_accumulator_count, prevcount;
	int		sample_counter;
	int		temperature;

	int		noisyGyrocnt, noisyAccelcnt;
	int		initcnt;
	int		NavLoopCnt;
	double	NavElapsedTime, NAVgyro_spin;
	int		NAVtemperature, NAVx_accel, NAVy_accel;

	MUTEX_ID INS_semaphore;

	byte gyro_rcv_buff[GYRO_READ_CNT];






	bool GyroTilt;
	double GyroHeading;	// the robot's heading calculated by correcting GyroRawAngle for the drift

	double GyroRawRate;
	double GyroCenter, GyroInitCenter, GyroNoiseLimit;
	double GyroSensitivity, cw_sensitivity, ccw_sensitivity;

	double GyroSample;
	double RobotDirection;



	long long int accumulator, prevaccum;
//	int i;
	int dataNotRdyCnt;

	double SampleLoopTime;
	double SampleRunTime;
	double SpinEffectCW;
	double SpinEffectCCW;
	double Sensitivity_Offset;
	double SampleElapsedTime;

};


#endif
