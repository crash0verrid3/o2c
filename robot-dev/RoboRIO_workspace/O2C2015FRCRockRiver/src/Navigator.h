#ifndef Navigator_H
#define Navigator_H
//mid-level actions

#include <Definitions.h>
#include "math.h"
#include "WPILib.h"
#include "HAL/Semaphore.hpp"
#include "Notifier.h"
#ifdef BUILTIN_ACCEL
	#include "BuiltInAccelerometer.h"
#endif
#ifdef ADXL345_ACCEL
	#include "I2c.h"
#endif
#ifdef MPU6050_6DOF
	#include "I2c.h"
#endif
#ifdef XYWHEELS

#endif

class Notifier;

class NAVIGATOR
{

public:
	NAVIGATOR();
	void Init(RobotStateBuffer *pRobotState, INPUT *pInput);
	void SetStartingPosition();
	void Zero();
	
private:
	Notifier *pReadSensors;	// independent loop to sample gyro and accelerometer at defined intervals
	Notifier *pINSupdate;	// independent loop to update Inertial Navigation System (position, orientation, speed, spin) at defined intervals

	Timer NavTimer;

	MUTEX_ID INS_semaphore;

#ifndef MPU6050_6DOF
#ifdef ANALOG_GYRO
	AnalogInput *MyGyro;
	long long int accumulator, prevaccum;
	int oversamplebits;
	AnalogInput *MyThermostat;
#endif

#ifdef MPU3050_GYRO
	I2C *MyGyro;
	uint8_t gyro_rcv_buff[GYRO_READ_CNT];
	// The I2C Gyro has a built-in Thermostat, so the SampleGyro method will supply the temperature reading
#endif

#ifdef ITG3200_GYRO
	I2C *MyGyro;
	uint8_t gyro_rcv_buff[GYRO_READ_CNT];
	// The I2C Gyro has a built-in Thermostat, so the SampleGyro method will supply the temperature reading
#endif

#ifdef WPILIB_GYRO
	Gyro		*MyGyro;
	AnalogInput *MyThermostat;
#endif

#ifdef ADXL345_ACCEL
	I2C *MyAccel;
	uint8_t accel_rcv_buff[ACCEL_READ_CNT];
#endif

#ifdef BUILTIN_ACCEL
	BuiltInAccelerometer *MyAccel;
#endif

#endif	//ifndef MPU6050_6DOF

#ifdef MPU6050_6DOF
	I2C *MyGyro;
	#ifdef MPU6050_USING_FIFO
	uint8_t DOF6_rcv_buff[DOF6_FIFO_SIZE];
	#endif
	#ifndef MPU6050_USING_FIFO
	uint8_t DOF6_rcv_buff[DOF6_READ_DATA_LEN];
	#endif
	// The I2C Gyro has a built-in Thermostat, so the SampleGyro method will supply the temperature reading
#endif

#ifdef XYWHEELS
	Encoder	*Xwheel;
	Encoder *Ywheel;
#endif

	static void CallReadSensors(void *controller);
	static void CallUpdateINS(void *controller);
	void InitMyGyro();
	void InitMyThermostat();
	void InitMyAccelerometer();
	int CheckFIFOcnt();
	void ReadGyro();
	void ReadAccel();
	void ReadThermostat();
	void ReadWheelEncoders();
	void ReadSensors();
	void Push_Navigation_Data_Frame(double timestamp, double x_gyro, double y_gyro, double z_gyro, double x_accel, double y_accel, double z_accel, double x_wheel, double y_wheel);
	bool Pop_Navigation_Data_Frame();
	void UpdatePosition();
	void UpdateHeading();
	void UpdateTipAngle();
	void UpdateINS();


	struct	Navigation_Data_Frame	// a frame of Navigation data collected from the gyro and accelerometer every 1 msec
		{						// Note: data points are robot oriented, not field oriented
		double	timestamp;		// elapsed clock time in microseconds since the last data point
		double	gyro_x;			// the number of microdegrees that the Robot tipped sideways since the last data point
		double	gyro_y;			// the number of microdegrees that the Robot tipped forward since the last data point
		double	gyro_z;			// the number of microdegrees that the Robot turned since the last data point
		double	accel_x;		// x-axis acceleration
		double	accel_y;		// y-axis acceleration
		double	accel_z;		// z-axis acceleration
		double	wheel_x;		// encoder tick count on the omni wheel aligned with x-axis
		double	wheel_y;		// encoder tick count on the omni wheel aligned with x-axis
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
		double	Xaccel;
		double	Yaccel;
		}	Navigation_Status_Buffer;

	bool	INS_Calibrating;
	double	x_spin_rate, y_spin_rate, z_spin_rate, NavClockTime;
	uint	gyro_accumulator_count, prevcount;
	int		sample_counter;
	double	x_wheel, y_wheel, prev_x_wheel, prev_y_wheel;
	double	x_accel, y_accel, z_accel;
	double	XaccelCenter, YaccelCenter, ZaccelCenter, XspeedCenter, YspeedCenter, Xsensitivity, Ysensitivity;
	int		temperature;
	int		noisyGyrocnt, noisyAccelcnt;
	int		initcnt;
	int		NavLoopCnt;
	double	NavElapsedTime, NAVx_spin, NAVy_spin, NAVz_spin, NAVx_accel, NAVy_accel, NAVz_accel, NAVx_wheel, NAVy_wheel;
	int		NAVtemperature;
	bool	GyroTilt;
	double	GyroHeading;	// the robot's heading calculated by correcting GyroRawAngle for the drift
	double	Xdeviation, Ydeviation, XtipAngle, YtipAngle;
	double	GyroCenter_X, GyroCenter_Y, GyroCenter_Z, GyroInitCenter, GyroCenter_T, GyroNoiseLimit;
	double	GyroXsensitivity, GyroYsensitivity;
	double	GyroSensitivity, cw_sensitivity, ccw_sensitivity, thermo_sensitivity;
	double	AccelInitCenterX, AccelInitCenterY, AccelInitCenterZ;
	double	RobotDirection;
	int		dataNotRdyCnt;
	double	SampleRunTime;
	double	SpinEffectCW;
	double	SpinEffectCCW;
	double	Sensitivity_Offset;
	double	SampleElapsedTime;
	double	FrictionEffect;
	double	prevx;
	short	fifo_queue_cnt;
	int		gyro_frame_index;
	int		accel_frame_index;
	double	gyro_noise_base, gyro_noise_squelch, accel_noise_base, accel_noise_squelch;

};


#endif
