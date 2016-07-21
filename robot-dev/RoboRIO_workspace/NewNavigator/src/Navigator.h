#ifndef Navigator_H
#define Navigator_H

#include <Definitions.h>
#include "math.h"
#include "WPILib.h"
//#include "HAL/Semaphore.hpp"
#include "Notifier.h"
#include "I2c.h"

class Notifier;

class NAVIGATOR
{

public:
	NAVIGATOR();
	void Init(RobotStateBuffer *pRobotState, INPUT *pInput);
	void SetStartingPosition();
	void ReadIMUCalStat();
	void Zero();
	
private:
	Notifier *pReadSensors;	// independent loop to sample gyro and accelerometer at defined intervals
	Notifier *pINSupdate;	// independent loop to update Inertial Navigation System (position, orientation, speed, spin) at defined intervals

	Timer NavTimer;

	MUTEX_ID INS_semaphore;

	I2C *MyIMU;

	uint8_t IMU_rcv_buff[I2C_RCV_DATA_LEN];

	static void CallReadSensors(void *controller);
	static void CallUpdateINS(void *controller);
	void InitMyIMU();
	int CheckFIFOcnt();
	void ReadThermostat();
	double ReadGyro();
	double ReadEuler();
	void ReadAccel();
	void ReadSensors();
	double ReadIMUdata();
	void Push_Navigation_Data_Frame(double timestamp, double x_gyro, double y_gyro, double z_gyro, double x_accel, double y_accel, double z_accel);
	bool Pop_Navigation_Data_Frame();
	void Compute_Gyro_Orientation();
	void UpdatePosition();
	void UpdateINS();


	struct	Navigation_Data_Frame	// a frame of Navigation data collected from the gyro and accelerometer every SAMPLEPERIOD
		{						// Note: data points are robot oriented, not field oriented
		double	timestamp;		// elapsed clock time in microseconds since the last data point
		double	gyro_x;			// the number of microdegrees that the Robot tipped sideways since the last data point
		double	gyro_y;			// the number of microdegrees that the Robot tipped forward since the last data point
		double	gyro_z;			// the number of microdegrees that the Robot turned since the last data point
		double	accel_x;		// x-axis acceleration
		double	accel_y;		// y-axis acceleration
		double	accel_z;		// z-axis acceleration
		} Navigation_Data_Queue[NAVIGATION_QUEUE_SIZE];
	int Navigation_Data_Queue_Top, Navigation_Data_Queue_Bottom;
	int Navigation_Data_Queue_Depth, Navigation_Data_Frame_Size;

/*	struct	Navigation_Status
		{
		double	Timestamp;
		double	Xcoord;
		double	Ycoord;
		double	Xspeed;
		double	Yspeed;
		double	Xaccel;
		double	Yaccel;
		}	Navigation_Status_Buffer;
 */
	bool	INS_Calibrating;
//	double	x_spin_rate, y_spin_rate, z_spin_rate;
	double	NavClockTime, NavUpdateTime, DataClockTime, DataCollectTime;
//	uint	gyro_accumulator_count, prevcount;
//	int		sample_counter;
//	double	x_wheel, y_wheel, prev_x_wheel, prev_y_wheel;
//	double	x_accel, y_accel, z_accel;
//	double	XaccelCenter, YaccelCenter, ZaccelCenter;
	double	XspeedCenter, YspeedCenter;
//	double	Xsensitivity, Ysensitivity;
	int		temperature;
	int		noisyGyrocnt, noisyAccelcnt;
	int		initcnt;
	int		NavLoopCnt, SampleCount;
	double	NavElapsedTime;
	double	NAVx_gyro, NAVy_gyro, NAVz_gyro;
	double	NAVx_accel, NAVy_accel, NAVz_accel;
//	double	NAVx_wheel, NAVy_wheel;
	int		NAVtemperature;
//	bool	GyroTilt;
	double	GyroHeading;	// the robot's heading calculated by correcting GyroRawAngle for the drift
//	double	Xdeviation, Ydeviation;
	double	XtipAngle, YtipAngle;
	double	GyroCenter_H, GyroCenter_R, GyroCenter_P;
	double	GyroDrift_H, GyroDrift_R, GyroDrift_P;
	double	Heading, Roll, Pitch;
	double	prev_Heading, prev_Roll, prev_Pitch;
//	double	GyroXsensitivity, GyroYsensitivity;
//	double	GyroSensitivity, cw_sensitivity, ccw_sensitivity, thermo_sensitivity;
//	double	AccelInitCenterX, AccelInitCenterY, AccelInitCenterZ;
	double	RobotDirection;
	int		dataNotRdyCnt;
	double	SampleRunTime;
//	double	SpinEffectCW;
//	double	SpinEffectCCW;
	double	Sensitivity_Offset;
	double	SampleElapsedTime;
//	double	FrictionEffect;
//	double	prevx;
//	short	fifo_queue_cnt;
	int		gyro_frame_index;
	int		accel_frame_index;
//	double	gyro_noise_base, gyro_noise_squelch, accel_noise_base, accel_noise_squelch;

	double	H[10] = {0,0,0,0,0,0,0,0,0,0};
	double	C[10] = {0,0,0,0,0,0,0,0,0,0};


};


#endif
