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
	void Init(RobotStateBuffer *pRobotState);
	void SetStartingPosition();
	void ReadIMUCalStat();
	void Zero();

private:
	Notifier *pReadSensors;	// independent loop to sample gyro and accelerometer at defined intervals
	Notifier *pINSupdate;	// independent loop to update Inertial Navigation System (position, orientation, speed, spin) at defined intervals

	Timer NavTimer;

	MUTEX_ID INS_semaphore;

	I2C *MyIMU;

	#ifdef XYWHEELS
		Encoder		*pXWheel, *pYWheel;
	#endif
	#ifdef LRWHEELS
		Encoder		*pLWheel, *pRWheel;
	#endif

	static void CallReadSensors(void *controller);
	static void CallUpdateINS(void *controller);
	void InitMyIMU();
	int CheckFIFOcnt();
	void ReadThermostat();
	void ReadGyro();
	void ReadEuler();
	#ifndef ITG3200_GYRO
		void ReadAccel();
	#endif
	void ReadWheelEncoders();
	void ReadSensors();
	double ReadIMUdata();
	void Push_Navigation_Data_Frame(double timestamp, double x_gyro, double y_gyro, double z_gyro, double x_accel, double y_accel, double z_accel, double a_wheel, double b_wheel);
	bool Pop_Navigation_Data_Frame();
	void Compute_Gyro_Orientation();
	void UpdatePosition();
	void UpdateINS();

#ifdef BNO055_9DOF
	#define I2C_RCV_ID_LEN		8
	#define I2C_RCV_DATA_LEN	45
	#define I2C_RCV_STATUS_LEN	6
	#define I2C_RCV_CAL_LEN	22
	struct I2C_RCV_BUFF
		{
		union
			{
			uInt8	ID_Buffer;
			uInt8	CHIP_ID;
			};
		uInt8	ACC_ID;
		uInt8	MAG_ID;
		uInt8	GYR_ID;
		uInt8	SW_REV_ID_LSB;
		uInt8	SW_REV_ID_MSB;
		uInt8	BL_REV_ID;
		uInt8	PAGE_ID;
		union
			{
			uInt8	Data_Buffer;
			uInt8	ACC_DATA_X_LSB;
			};
		uInt8	ACC_DATA_X_MSB;
		uInt8	ACC_DATA_Y_LSB;
		uInt8	ACC_DATA_Y_MSB;
		uInt8	ACC_DATA_Z_LSB;
		uInt8	ACC_DATA_Z_MSB;
		uInt8	MAG_DATA_X_LSB;
		uInt8	MAG_DATA_X_MSB;
		uInt8	MAG_DATA_Y_LSB;
		uInt8	MAG_DATA_Y_MSB;
		uInt8	MAG_DATA_Z_LSB;
		uInt8	MAG_DATA_Z_MSB;
		uInt8	GYR_DATA_X_LSB;
		uInt8	GYR_DATA_X_MSB;
		uInt8	GYR_DATA_Y_LSB;
		uInt8	GYR_DATA_Y_MSB;
		uInt8	GYR_DATA_Z_LSB;
		uInt8	GYR_DATA_Z_MSB;
		uInt8	EUL_DATA_X_LSB;
		uInt8	EUL_DATA_X_MSB;
		uInt8	EUL_DATA_Y_LSB;
		uInt8	EUL_DATA_Y_MSB;
		uInt8	EUL_DATA_Z_LSB;
		uInt8	EUL_DATA_Z_MSB;
		uInt8	QUA_DATA_W_LSB;
		uInt8	QUA_DATA_W_MSB;
		uInt8	QUA_DATA_X_LSB;
		uInt8	QUA_DATA_X_MSB;
		uInt8	QUA_DATA_Y_LSB;
		uInt8	QUA_DATA_Y_MSB;
		uInt8	QUA_DATA_Z_LSB;
		uInt8	QUA_DATA_Z_MSB;
		uInt8	LIA_DATA_X_LSB;
		uInt8	LIA_DATA_X_MSB;
		uInt8	LIA_DATA_Y_LSB;
		uInt8	LIA_DATA_Y_MSB;
		uInt8	LIA_DATA_Z_LSB;
		uInt8	LIA_DATA_Z_MSB;
		uInt8	GRV_DATA_X_LSB;
		uInt8	GRV_DATA_X_MSB;
		uInt8	GRV_DATA_Y_LSB;
		uInt8	GRV_DATA_Y_MSB;
		uInt8	GRV_DATA_Z_LSB;
		uInt8	GRV_DATA_Z_MSB;
		uInt8	TEMP;
		union
			{
			uInt8	Status_Buffer;
			uInt8	CALIB_STAT;
			};
		uInt8	ST_RESULT;
		uInt8	INT_STA;
		uInt8	SYS_CLK_STATUS;
		uInt8	SYS_STATUS;
		uInt8	SYS_ERR;
		uInt8	UNIT_SEL;
		uInt8	OPR_MODE;
		uInt8	PWR_MODE;
		uInt8	SYS_TRIGGER;
		uInt8	TEMP_SOURCE;
		uInt8	AXIS_MAP_CONFIG;
		uInt8	AXIS_MAP_SIGN;
		union
			{
			uInt8	Calibration_Buffer;
			uInt8	ACC_OFFSET_X_LSB;
			};
		uInt8	ACC_OFFSET_X_MSB;
		uInt8	ACC_OFFSET_Y_LSB;
		uInt8	ACC_OFFSET_Y_MSB;
		uInt8	ACC_OFFSET_Z_LSB;
		uInt8	ACC_OFFSET_Z_MSB;
		uInt8	MAG_OFFSET_X_LSB;
		uInt8	MAG_OFFSET_X_MSB;
		uInt8	MAG_OFFSET_Y_LSB;
		uInt8	MAG_OFFSET_Y_MSB;
		uInt8	MAG_OFFSET_Z_LSB;
		uInt8	MAG_OFFSET_Z_MSB;
		uInt8	GYR_OFFSET_X_LSB;
		uInt8	GYR_OFFSET_X_MSB;
		uInt8	GYR_OFFSET_Y_LSB;
		uInt8	GYR_OFFSET_Y_MSB;
		uInt8	GYR_OFFSET_Z_LSB;
		uInt8	GYR_OFFSET_Z_MSB;
		uInt8	ACC_RADIUS_LSB;
		uInt8	ACC_RADIUS_MSB;
		uInt8	MAG_RADIUS_LSB;
		uInt8	MAG_RADIUS_MSB;
		}	*pMyI2Cdatabuffer;
#endif	// BNO055_9DOF


#ifdef MPU6050_6DOF
	#define I2C_RCV_DATA_LEN	14
	#define I2C_RCV_STATUS_LEN	1
	struct I2C_RCV_BUFF
		{
		union
			{
			uInt8	Status_Buffer;
			uInt8	INT_STA;
			};
		union
			{
			uInt8	Data_Buffer;
			uInt8	ACC_DATA_X_MSB;
			};
		uInt8	ACC_DATA_X_LSB;
		uInt8	ACC_DATA_Y_MSB;
		uInt8	ACC_DATA_Y_LSB;
		uInt8	ACC_DATA_Z_MSB;
		uInt8	ACC_DATA_Z_LSB;
		uInt8	TEMP_DATA_MSB;
		uInt8	TEMP_DATA_LSB;
		uInt8	GYR_DATA_X_MSB;
		uInt8	GYR_DATA_X_LSB;
		uInt8	GYR_DATA_Y_MSB;
		uInt8	GYR_DATA_Y_LSB;
		uInt8	GYR_DATA_Z_MSB;
		uInt8	GYR_DATA_Z_LSB;
		uInt8	EXT_SENS_DATA_00;
		uInt8	EXT_SENS_DATA_01;
		uInt8	EXT_SENS_DATA_02;
		uInt8	EXT_SENS_DATA_03;
		uInt8	EXT_SENS_DATA_04;
		uInt8	EXT_SENS_DATA_05;
		uInt8	EXT_SENS_DATA_06;
		uInt8	EXT_SENS_DATA_07;
		uInt8	EXT_SENS_DATA_08;
		uInt8	EXT_SENS_DATA_09;
		uInt8	EXT_SENS_DATA_10;
		uInt8	EXT_SENS_DATA_11;
		uInt8	EXT_SENS_DATA_12;
		uInt8	EXT_SENS_DATA_13;
		uInt8	EXT_SENS_DATA_14;
		uInt8	EXT_SENS_DATA_15;
		uInt8	EXT_SENS_DATA_16;
		uInt8	EXT_SENS_DATA_17;
		uInt8	EXT_SENS_DATA_18;
		uInt8	EXT_SENS_DATA_19;
		uInt8	EXT_SENS_DATA_20;
		uInt8	EXT_SENS_DATA_21;
		uInt8	EXT_SENS_DATA_22;
		uInt8	EXT_SENS_DATA_23;
		}	*pMyI2Cdatabuffer;
#endif	// MPU6050_6DOF

#ifdef ITG3200_GYRO
	#define I2C_RCV_DATA_LEN	8
	#define I2C_RCV_STATUS_LEN	1
		struct I2C_RCV_BUFF
			{
			union
				{
				uInt8	Status_Buffer;
				uInt8	INT_STA;
				};
			union
				{
				uInt8	Data_Buffer;
				uInt8	TEMP_DATA_MSB;
				};
			uInt8	TEMP_DATA_LSB;
			uInt8	GYR_DATA_X_MSB;
			uInt8	GYR_DATA_X_LSB;
			uInt8	GYR_DATA_Y_MSB;
			uInt8	GYR_DATA_Y_LSB;
			uInt8	GYR_DATA_Z_MSB;
			uInt8	GYR_DATA_Z_LSB;
			}	*pMyI2Cdatabuffer;
#endif	// ITG3200_GYRO

#define GYRO_Reg_TEMP_OUT_H	0x1B	//0-7: high 8-bits of 16-bit data
#define GYRO_Reg_TEMP_OUT_L	0x1C	//0-7: high 8-bits of 16-bit data
#define GYRO_Reg_XOUT_H		0x1D
#define GYRO_Reg_XOUT_L		0x1E
#define GYRO_Reg_YOUT_H		0x1F
#define GYRO_Reg_YOUT_L		0x20
#define GYRO_Reg_ZOUT_H		0x21
#define GYRO_Reg_ZOUT_L		0x22



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
	#ifdef XYWHEELS
		double	NAVx_wheel, NAVy_wheel;
		double	prev_x_wheel, prev_y_wheel;
	#endif
	#ifdef LRWHEELS
		double	NAVl_wheel, NAVr_wheel;
		double	prev_l_wheel, prev_r_wheel;
	#endif
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
//	double	Sensitivity_Offset;
	double	SampleElapsedTime;
//	double	FrictionEffect;
//	double	prevx;
//	short	fifo_queue_cnt;
	int		gyro_frame_index;
	int		accel_frame_index;
//	double	gyro_noise_base, gyro_noise_squelch, accel_noise_base, accel_noise_squelch;

	double	H[10] = {0,0,0,0,0,0,0,0,0,0};
	double	C[10] = {0,0,0,0,0,0,0,0,0,0};





	// Inertial Navigation System
	#define I2C_CHANNEL 2				// RoboRIO I2C channel
	#define SAMPLEPERIOD 0.001			// I2C bus WPILib function calls are inefficient and take 0.5msec per call
	#define INSUPDATEPERIOD 0.01		// Update positon, orientation, speed, heading, spin every 10 msec
	#define NAVIGATION_QUEUE_SIZE	200	// we will be long lost before we fill this queue
	//#define I2C_RCV_DATA_LEN		6	// Good news. the 7 byte limit has been lifted as of 2016.  Bad news, we need to reprogram

	struct	Navigation_Data_Frame	// a frame of Navigation data collected from the gyro and accelerometer every SAMPLEPERIOD
		{						// Note: data points are robot oriented, not field oriented
		double	timestamp;		// elapsed clock time in microseconds since the last data point
		double	gyro_x;			// the number of microdegrees that the Robot tipped sideways since the last data point
		double	gyro_y;			// the number of microdegrees that the Robot tipped forward since the last data point
		double	gyro_z;			// the number of microdegrees that the Robot turned since the last data point
		double	accel_x;		// x-axis acceleration
		double	accel_y;		// y-axis acceleration
		double	accel_z;		// z-axis acceleration
		#ifdef XYWHEELS
			double	wheel_x;
			double	wheel_y;
		#endif
		#ifdef LRWHEELS
			double	wheel_l;
			double	wheel_r;
		#endif
		} Navigation_Data_Queue[NAVIGATION_QUEUE_SIZE];
	int Navigation_Data_Queue_Top, Navigation_Data_Queue_Bottom;
	int Navigation_Data_Queue_Depth, Navigation_Data_Frame_Size;


	// From here down are the control register definitions for various Gyros and Accelerometers


	#ifdef BNO055_9DOF
		/*
		The BNO055 is a System in Package (SiP), integrating a triaxial 14-bit accelerometer, a triaxial
		16-bit gyroscope with a range of �2000 degrees per second, a triaxial geomagnetic sensor
		and a 32-bit cortex M0+ microcontroller running Bosch Sensortec sensor fusion software, in
		a single package.

		The default operation mode after power-on is CONFIGMODE.
		When the user changes to another operation mode, the sensors which are required in that
		particular sensor mode are powered, while the sensors whose signals are not required are
		set to suspend mode.

		Sensor fusion modes are meant to calculate measures describing the orientation of the
		device in space. It can be distinguished between non-absolute or relative orientation and
		absolute orientation. Absolute orientation means orientation of the sensor with respect to
		the earth and its magnetic field. In other words, absolute orientation sensor fusion modes
		calculate the direction of the magnetic north pole.
		In non-absolute or relative orientation modes, the heading of the sensor can vary depending
		on how the sensor is placed initially.
		All fusion modes provide the heading of the sensor as quaternion data or in Euler angles
		(roll, pitch and yaw angle). The acceleration sensor is both exposed to the gravity force and to
		accelerations applied to the sensor due to movement. In fusion modes it is possible to
		separate the two acceleration sources, and thus the sensor fusion data provides separately
		linear acceleration (i.e. acceleration that is applied due to movement) and the gravity vector.

		In the IMU mode the relative orientation of the BNO055 in space is calculated from the
		accelerometer and gyroscope data. The calculation is fast (i.e. high output data rate).
		The operating mode can be selected by writing to the OPR_MODE register.
		Fusion Mode IMU [OPR_MODE]: xxxx1000b.

		The fusion outputs of the BNO055 are tightly linked with the sensor configuration settings.
		Due to this fact, the sensor configuration is limited when BNO055 is configured to run in any
		of the fusion operating mode. In any of the sensor modes the configuration settings can be
		updated by writing to the configuration registers.

		Default sensor settings:  Gyro 2000dps/32Hz; Accel 4g/62.5Hz

			Gyro.  All config settings are auto controlled in Fusion mode.
			Accel.  G Range is User selectable in all modes.  Bandwidth and Power settings are auto controlled in Fusion mode.
				2G [ACC_Config]: xxxxxx00b
				4G [ACC_Config]: xxxxxx01b
				8G [ACC_Config]: xxxxxx10b
				16G [ACC_Config]: xxxxxx11b
			Mag.  All config settings are auto controlled in Fusion mode.

		The device mounting position should not limit the data output of the BNO055 device. The
		axis of the device can be re-configured to the new reference axis.

		The measurement units for the various data outputs (regardless of operation mode) can be
		configured by writing to the UNIT_SEL register as described here.
				Data 								Units 			[Reg Addr]: Register Value
				Acceleration, Linear				m/s2			[UNIT_SEL] : xxxxxxx0b
				Acceleration, Gravity vector		mg				[UNIT_SEL] : xxxxxxx1b
				Magnetic Field Strength				Micro Tesla 	NA
				Angular Rate 						Dps 			[UNIT_SEL] : xxxxxx0xb
													Rps 			[UNIT_SEL] : xxxxxx1xb
				Euler Angles 						Degrees 		[UNIT_SEL] : xxxxx0xxb
													Radians 		[UNIT_SEL] : xxxxx1xxb
				Quaternion 							Quaternion units	NA
				Temperature 						�C 				[UNIT_SEL] : xxx0xxxxb
													�F 				[UNIT_SEL] : xxx1xxxxb

		In fusion mode the fusion algorithm output offset compensated acceleration data for each
		axis X/Y/Z, the output data can be read from the appropriate ACC_DATA_<axis>_LSB and
		ACC_DATA_<axis>_MSB registers.

		In fusion mode the fusion algorithm output offset compensated angular velocity (yaw rate)
		data for each axis X/Y/Z, the output data can be read from the appropriate
		GYR_DATA_<axis>_LSB and GYR_DATA_<axis>_MSB registers.

		The fusion algorithm output offset and tilt compensated orientation data in Euler angles
		format for each DOF Heading/Roll/Pitch, the output data can be read from the appropriate
		EUL<dof>_LSB and EUL_<dof>_MSB registers.

		The fusion algorithm output linear acceleration data for each axis x/y/z, the output data can
		be read from the appropriate LIA_DATA_<axis>_LSB and LIA_DATA_<axis>_MSB registers.

		The register map is separated into two logical pages, Page 1 contains sensor specific
		configuration data and Page 0 contains all other configuration parameters and output data.
		At power-on Page 0 is selected, the PAGE_ID register can be used to identify the current
		selected page and change between page 0 and page 1.

		*/


		#define IMU_I2C_ADDR		0x28
		#define IMU_I2C_ALT_ADDR	0x29
		#define BNO055_ID       	0xA0
		#define BNO055_STATUS_CNT	6

		/* Page id register definition */
		#define BNO055_PAGE_ID_ADDR                      0X07

		/* PAGE0 REGISTER DEFINITION START*/
		#define BNO055_CHIP_ID_ADDR                      0x00
		#define BNO055_ACCEL_REV_ID_ADDR                 0x01
		#define BNO055_MAG_REV_ID_ADDR                   0x02
		#define BNO055_GYRO_REV_ID_ADDR                  0x03
		#define BNO055_SW_REV_ID_LSB_ADDR                0x04
		#define BNO055_SW_REV_ID_MSB_ADDR                0x05
		#define BNO055_BL_REV_ID_ADDR                    0X06

		/* Accel data register */
		#define BNO055_ACCEL_DATA_X_LSB_ADDR             0X08
		#define BNO055_ACCEL_DATA_X_MSB_ADDR             0X09
		#define BNO055_ACCEL_DATA_Y_LSB_ADDR             0X0A
		#define BNO055_ACCEL_DATA_Y_MSB_ADDR             0X0B
		#define BNO055_ACCEL_DATA_Z_LSB_ADDR             0X0C
		#define BNO055_ACCEL_DATA_Z_MSB_ADDR             0X0D

		/* Mag data register */
		#define BNO055_MAG_DATA_X_LSB_ADDR               0X0E
		#define BNO055_MAG_DATA_X_MSB_ADDR               0X0F
		#define BNO055_MAG_DATA_Y_LSB_ADDR               0X10
		#define BNO055_MAG_DATA_Y_MSB_ADDR               0X11
		#define BNO055_MAG_DATA_Z_LSB_ADDR               0X12
		#define BNO055_MAG_DATA_Z_MSB_ADDR               0X13

		/* Gyro data registers */
		#define BNO055_GYRO_DATA_X_LSB_ADDR              0X14
		#define BNO055_GYRO_DATA_X_MSB_ADDR              0X15
		#define BNO055_GYRO_DATA_Y_LSB_ADDR              0X16
		#define BNO055_GYRO_DATA_Y_MSB_ADDR              0X17
		#define BNO055_GYRO_DATA_Z_LSB_ADDR              0X18
		#define BNO055_GYRO_DATA_Z_MSB_ADDR              0X19

		/* Euler data registers */
		#define BNO055_EULER_H_LSB_ADDR                  0X1A
		#define BNO055_EULER_H_MSB_ADDR                  0X1B
		#define BNO055_EULER_R_LSB_ADDR                  0X1C
		#define BNO055_EULER_R_MSB_ADDR                  0X1D
		#define BNO055_EULER_P_LSB_ADDR                  0X1E
		#define BNO055_EULER_P_MSB_ADDR                  0X1F

		/* Quaternion data registers */
		#define BNO055_QUATERNION_DATA_W_LSB_ADDR        0X20
		#define BNO055_QUATERNION_DATA_W_MSB_ADDR        0X21
		#define BNO055_QUATERNION_DATA_X_LSB_ADDR        0X22
		#define BNO055_QUATERNION_DATA_X_MSB_ADDR        0X23
		#define BNO055_QUATERNION_DATA_Y_LSB_ADDR        0X24
		#define BNO055_QUATERNION_DATA_Y_MSB_ADDR        0X25
		#define BNO055_QUATERNION_DATA_Z_LSB_ADDR        0X26
		#define BNO055_QUATERNION_DATA_Z_MSB_ADDR        0X27

		/* Linear acceleration data registers */
		#define BNO055_LINEAR_ACCEL_DATA_X_LSB_ADDR      0X28
		#define BNO055_LINEAR_ACCEL_DATA_X_MSB_ADDR      0X29
		#define BNO055_LINEAR_ACCEL_DATA_Y_LSB_ADDR      0X2A
		#define BNO055_LINEAR_ACCEL_DATA_Y_MSB_ADDR      0X2B
		#define BNO055_LINEAR_ACCEL_DATA_Z_LSB_ADDR      0X2C
		#define BNO055_LINEAR_ACCEL_DATA_Z_MSB_ADDR      0X2D

		/* Gravity data registers */
		#define BNO055_GRAVITY_DATA_X_LSB_ADDR           0X2E
		#define BNO055_GRAVITY_DATA_X_MSB_ADDR           0X2F
		#define BNO055_GRAVITY_DATA_Y_LSB_ADDR           0X30
		#define BNO055_GRAVITY_DATA_Y_MSB_ADDR           0X31
		#define BNO055_GRAVITY_DATA_Z_LSB_ADDR           0X32
		#define BNO055_GRAVITY_DATA_Z_MSB_ADDR           0X33

		/* Temperature data register */
		#define BNO055_TEMP_ADDR                         0X34

		/* Status registers */
		#define BNO055_CALIB_STAT_ADDR                   0X35
			#define BNO055_CALIB_STAT_MAG							0X03
			#define BNO055_CALIB_STAT_ACC							0X0C
			#define BNO055_CALIB_STAT_GYRO							0X30
			#define BNO055_CALIB_STAT_SYS							0XC0
		#define BNO055_SELFTEST_RESULT_ADDR              0X36
			#define BNO055_ST_STAT_ACC							0X01
			#define BNO055_ST_STAT_MAG							0X02
			#define BNO055_ST_STAT_GYRO							0X04
			#define BNO055_ST_STAT_MCU							0X08
		#define BNO055_INTR_STAT_ADDR                    0X37
			#define BNO055_INTR_STAT_MAG_AM							0X04
			#define BNO055_INTR_STAT_MAG_HR							0X08
			#define BNO055_INTR_STAT_ACC_HG							0X20
			#define BNO055_INTR_STAT_ACC_AM							0X40
			#define BNO055_INTR_STAT_ACC_NM							0X80
		#define BNO055_SYS_CLK_STAT_ADDR                 0X38
			#define BNO055_SYS_CLK_STAT								0X01
		#define BNO055_SYS_STAT_ADDR                     0X39
			#define BNO055_SYS_STAT									0X0F
			enum BNO055_SYS_STAT_VALUE {BNO055_SYS_STAT_IDLE=0,
										BNO055_SYS_STAT_ERROR=1,
										BNO055_SYS_STAT_INITP=2,
										BNO055_SYS_STAT_INITS=3,
										BNO055_SYS_STAT_EXEST=4,
										BNO055_SYS_STAT_FUSON=5,
										BNO055_SYS_STAT_NOFUS=6};
		#define BNO055_SYS_ERR_ADDR                      0X3A
			#define BNO055_SYS_ERR									0X0F
			enum BNO055_SYS_ERR_VALUE  {BNO055_SYS_ERR_NOERR=0,
			 	 	 	 	 	 	 	BNO055_SYS_ERR_INITP=1,
										BNO055_SYS_ERR_INITS=2,
										BNO055_SYS_ERR_STFAIL=3,
										BNO055_SYS_ERR_REGVAL=4,
										BNO055_SYS_ERR_REGADD=5,
										BNO055_SYS_ERR_REGWRT=6,
										BNO055_SYS_ERR_LOWPWR=7,
										BNO055_SYS_ERR_ACCPWR=8,
										BNO055_SYS_ERR_FUSION=9,
										BNO055_SYS_ERR_CONFIG=10};

		/* Unit selection register */
		#define BNO055_UNIT_SEL_ADDR                     0X3B
			#define BNO055_ORIENTATION_ANDROID						0X80
			#define BNO055_TEMP_FAHRENHEIT							0X10
			#define BNO055_EULER_RADIANS							0X04
			#define BNO055_ANGULAR_RATE_RPS							0X02
			#define BNO055_ACCEL_MG									0X01
			#define BNO055_ORIENTATION_WINDOWS						0X00
			#define BNO055_TEMP_CELSIUS								0X00
			#define BNO055_EULER_DEGREES							0X00
			#define BNO055_ANGULAR_RATE_DPS							0X00
			#define BNO055_ACCEL_MPS2								0X00
		#define BNO055_DATA_SELECT_ADDR                  0X3C

		/* Mode registers */
		#define BNO055_OPR_MODE_ADDR                     0X3D
			#define BNO055_OPERATION_MODE_CONFIG                    0X00
			#define BNO055_OPERATION_MODE_ACCONLY                   0X01
			#define BNO055_OPERATION_MODE_MAGONLY                   0X02
			#define BNO055_OPERATION_MODE_GYRONLY                   0X03
			#define BNO055_OPERATION_MODE_ACCMAG                    0X04
			#define BNO055_OPERATION_MODE_ACCGYRO                   0X05
			#define BNO055_OPERATION_MODE_MAGGYRO                   0X06
			#define BNO055_OPERATION_MODE_AMG                       0X07
			#define BNO055_OPERATION_MODE_IMU                       0X08
			#define BNO055_OPERATION_MODE_COMPASS                   0X09
			#define BNO055_OPERATION_MODE_M4G                       0X0A
			#define BNO055_OPERATION_MODE_NDOF_FMC_OFF              0X0B
			#define BNO055_OPERATION_MODE_NDOF                      0X0C
		#define BNO055_PWR_MODE_ADDR                     0X3E
			#define BNO055_POWER_MODE_NORMAL                        0X00
			#define BNO055_POWER_MODE_LOWPOWER                      0X01
			#define BNO055_POWER_MODE_SUSPEND                       0X02

		#define BNO055_SYS_TRIGGER_ADDR                  0X3F
			#define BNO55_TRIGGER_EXTCLK							0X80
			#define BNO55_TRIGGER_RSTINT							0X40
			#define BNO55_TRIGGER_RSTSYS							0X20
			#define BNO55_TRIGGER_SELFTEST							0X01
		#define BNO055_TEMP_SOURCE_ADDR                  0X40

		/* Axis remap registers */
		#define BNO055_AXIS_MAP_ZYX_ADDR	              0X41
			#define	BNO055_AXIS_MAP_XYZ								0x06
			#define	BNO055_AXIS_MAP_XZY								0x09
			#define	BNO055_AXIS_MAP_YXZ								0x12
			#define	BNO055_AXIS_MAP_YZX								0x14
			#define	BNO055_AXIS_MAP_ZXY								0x21
			#define	BNO055_AXIS_MAP_ZYX								0x24
		#define BNO055_AXIS_SIGN_XYZ_ADDR                0X42
			#define	BNO055_AXIS_SIGN_PPP								0x00
			#define	BNO055_AXIS_SIGN_PPN								0x01
			#define	BNO055_AXIS_SIGN_PNP								0x02
			#define	BNO055_AXIS_SIGN_PNN								0x03
			#define	BNO055_AXIS_SIGN_NPP								0x04
			#define	BNO055_AXIS_SIGN_NPN								0x05
			#define	BNO055_AXIS_SIGN_NNP								0x06
			#define	BNO055_AXIS_SIGN_NNN								0x07

		/* SIC registers */
		#define BNO055_SIC_MATRIX_0_LSB_ADDR             0X43
		#define BNO055_SIC_MATRIX_0_MSB_ADDR             0X44
		#define BNO055_SIC_MATRIX_1_LSB_ADDR             0X45
		#define BNO055_SIC_MATRIX_1_MSB_ADDR             0X46
		#define BNO055_SIC_MATRIX_2_LSB_ADDR             0X47
		#define BNO055_SIC_MATRIX_2_MSB_ADDR             0X48
		#define BNO055_SIC_MATRIX_3_LSB_ADDR             0X49
		#define BNO055_SIC_MATRIX_3_MSB_ADDR             0X4A
		#define BNO055_SIC_MATRIX_4_LSB_ADDR             0X4B
		#define BNO055_SIC_MATRIX_4_MSB_ADDR             0X4C
		#define BNO055_SIC_MATRIX_5_LSB_ADDR             0X4D
		#define BNO055_SIC_MATRIX_5_MSB_ADDR             0X4E
		#define BNO055_SIC_MATRIX_6_LSB_ADDR             0X4F
		#define BNO055_SIC_MATRIX_6_MSB_ADDR             0X50
		#define BNO055_SIC_MATRIX_7_LSB_ADDR             0X51
		#define BNO055_SIC_MATRIX_7_MSB_ADDR             0X52
		#define BNO055_SIC_MATRIX_8_LSB_ADDR             0X53
		#define BNO055_SIC_MATRIX_8_MSB_ADDR             0X54

		/* Accelerometer Offset registers */
		#define BNO055_ACCEL_OFFSET_X_LSB_ADDR           0X55
		#define BNO055_ACCEL_OFFSET_X_MSB_ADDR           0X56
		#define BNO055_ACCEL_OFFSET_Y_LSB_ADDR           0X57
		#define BNO055_ACCEL_OFFSET_Y_MSB_ADDR           0X58
		#define BNO055_ACCEL_OFFSET_Z_LSB_ADDR           0X59
		#define BNO055_ACCEL_OFFSET_Z_MSB_ADDR           0X5A

		/* Magnetometer Offset registers */
		#define BNO055_MAG_OFFSET_X_LSB_ADDR             0X5B
		#define BNO055_MAG_OFFSET_X_MSB_ADDR             0X5C
		#define BNO055_MAG_OFFSET_Y_LSB_ADDR             0X5D
		#define BNO055_MAG_OFFSET_Y_MSB_ADDR             0X5E
		#define BNO055_MAG_OFFSET_Z_LSB_ADDR             0X5F
		#define BNO055_MAG_OFFSET_Z_MSB_ADDR             0X60

		/* Gyroscope Offset register s*/
		#define BNO055_GYRO_OFFSET_X_LSB_ADDR            0X61
		#define BNO055_GYRO_OFFSET_X_MSB_ADDR            0X62
		#define BNO055_GYRO_OFFSET_Y_LSB_ADDR            0X63
		#define BNO055_GYRO_OFFSET_Y_MSB_ADDR            0X64
		#define BNO055_GYRO_OFFSET_Z_LSB_ADDR            0X65
		#define BNO055_GYRO_OFFSET_Z_MSB_ADDR            0X66

		/* Radius registers */
		#define BNO055_ACCEL_RADIUS_LSB_ADDR             0X67
		#define BNO055_ACCEL_RADIUS_MSB_ADDR             0X68
		#define BNO055_MAG_RADIUS_LSB_ADDR               0X69
		#define BNO055_MAG_RADIUS_MSB_ADDR               0X6A

	#endif	// BNO055_9DOF


	// ITG3200_GYRO
	#ifdef ITG3200_GYRO
		#define IMU_I2C_ADDR		0x68
		#define IMU_I2C_ALT_ADDR	0x69
		#define GYRO_Reg_WhoAmI		0x00	//7:I2C_IF_DIS; 1-6: ID; 0:R/W
		#define GYRO_Reg_SMPLRT_DIV	0x15
			#define GYRO_SampDiv1	0x00	//00000100	= new sample every .125 msec (we check every 1 ms)
			#define GYRO_SampDiv2	0x01	//00000100	= new sample every .250 msec (we check every 1 ms)
			#define GYRO_SampDiv3	0x02	//00000100	= new sample every .375 msec (we check every 1 ms)
			#define GYRO_SampDiv4	0x03	//00000100	= new sample every .500 msec (we check every 1 ms)
			#define GYRO_SampDiv5	0x04	//00000100	= new sample every .625 msec (we check every 1 ms)
			#define GYRO_SampDiv6	0x05	//00000101	= new sample every .750 msec (we check every 1 ms)
			#define GYRO_SampDiv7	0x06	//00000110	= new sample every .875 msec (we check every 1 ms)
			#define GYRO_SampDiv8	0x07	//00000111	= new sample every 1.00 msec (we check every 1 ms)
		#define	GYRO_Reg_DLPF_FS	0x16
			#define GYRO_2000dps	0x18	//00011000
			#define GYRO_8kx256Hz	0x00	//00000000
			#define GYRO_1kx188Hz	0x01	//00000001
			#define GYRO_1kx98Hz	0x02	//00000001
			#define GYRO_1kx42Hz	0x03	//00000001
			#define GYRO_1kx20Hz	0x04	//00000001
			#define GYRO_1kx10Hz	0x05	//00000001
			#define GYRO_1kx5Hz		0x06	//00000001
		#define	GYRO_Reg_INT_CFG	0x17
			#define GYRO_DataRdy	0x01	//00000001	= enable data ready bit
			#define GYRO_Latch		0x20	//00100000	= latch the data ready bit until cleared
			#define GYRO_AnyRd2Clr	0x10	//00010000	= clear data ready bit on any register read
		#define GYRO_Reg_INT_STATUS	0x1A
			#define GYRO_DataRdy	0x01	//00000001	= data ready bit mask
		#define GYRO_Reg_TEMP_OUT_H	0x1B	//0-7: high 8-bits of 16-bit data
		#define GYRO_Reg_TEMP_OUT_L	0x1C	//0-7: high 8-bits of 16-bit data
		#define GYRO_Reg_XOUT_H		0x1D
		#define GYRO_Reg_XOUT_L		0x1E
		#define GYRO_Reg_YOUT_H		0x1F
		#define GYRO_Reg_YOUT_L		0x20
		#define GYRO_Reg_ZOUT_H		0x21
		#define GYRO_Reg_ZOUT_L		0x22
		#define GYRO_Reg_PWR_MGM	0x3E
			#define GYRO_H_Reset	0x80	//10000000
			#define GYRO_Sleep		0x40	//01000000
			#define GYRO_Stby_GX	0x20	//00100000
			#define GYRO_Stby_GY	0x10	//00010000
			#define GYRO_Stby_GZ	0x08	//00001000
			#define GYRO_Clock_Intl	0x00	//internal clock source
			#define GYRO_Clock_X	0x01	// X Gyro provides clock source
			#define GYRO_Clock_Y	0x02	// Y Gyro provides clock source
			#define GYRO_Clock_Z	0x03	// Z Gyro provides clock source
	#endif	// ITG3200_GYRO


	#ifdef ANALOG_THERMOSTAT
		#define THERMOSTAT_ANALOGCHANNEL 1
	#endif	// ANALOG_THERMOSTAT


	#ifdef ADXL345_ACCEL
		#define ACCEL_I2C_ADDR		0x53
		#define ACCEL_I2C_ALTADDR	0x1D
		#define ACCEL_OFSX			0x1E	// we won't use the offset registers
		#define ACCEL_OFSY			0x1F	// we will calculate/maintain our own
		#define ACCEL_OFSZ			0x20	// center point value for each axis
		#define ACCEL_BW_RATE		0x2C	// =100Hz on power up; we want 1600Hz (00001110)
			#define ACCEL_RATE_50Hz		0x9
			#define ACCEL_RATE_100Hz	0xA
			#define ACCEL_RATE_200Hz	0xB
			#define ACCEL_RATE_400Hz	0xC
			#define ACCEL_RATE_800Hz	0xD
			#define ACCEL_RATE_1600Hz	0xE
			#define ACCEL_RATE_3200Hz	0xF
		#define ACCEL_PWR_CTL		0x2D	// set to (00001000) to start measuring
			#define ACCEL_PWR_STANDBY	0x0
			#define ACCEL_PWR_MEASURE	0x8
		#define ACCEL_INT_ENA		0x2E
			#define ACCEL_DATA_RDY		0x80
		#define ACCEL_INT_MAP		0x2F
		#define ACCEL_INT_SRC		0x30
			#define ACCEL_DATA_RDY		0x80
		#define ACCEL_DATA_FMT		0x31	// set to (00001011) for Full Range +/- 16g
			#define ACCEL_FULL_RES		0x8
			#define ACCEL_2g			0x0
			#define ACCEL_4g			0x1
			#define ACCEL_8g			0x2
			#define ACCEL_16g			0x3
		#define ACCEL_X0_DATA		0x32	// LSB
		#define ACCEL_X1_DATA		0x33	// MSB sign extended twos compliment
		#define ACCEL_Y0_DATA		0x34	// LSB
		#define ACCEL_Y1_DATA		0x35	// MSB sign extended twos compliment
		#define ACCEL_Z0_DATA		0x36	// LSB
		#define ACCEL_Z1_DATA		0x37	// MSB sign extended twos compliment
		#define ACCEL_FIFO_CTL		0x38	// set to (10000001) for Stream w/min of 1 sample
			#define ACCEL_FIFO_OFF		0x0
			#define ACCEL_FIFO_FULL		0x40
			#define ACCEL_FIFO_STREAM	0x80
			#define ACCEL_FIFO_TRIG		0xC0
			#define ACCEL_FIFO_SAMPLES	0x1
		#define ACCEL_FIFO_STAT		0x39	// &0x3F to give FIFO queue depth
			#define ACCEL_FIFO_QUE_MSK	0x3F
		/*
		 * Note:  Unlike the I2C gyro, the ADXL345 FIFO data is read from the normal
		 * 		data registers and all axes must be read in one multi-byte read, since
		 * 		the data frame is replaced by the next FIFO entry as soon as any
		 * 		data registers are read.
		 */

		#define ACCEL_READ_CNT	6	// 2 bytes per axis
		#define ACCEL_OVERSAMPLECNT 8	// add 8 consecutive samples to achieve a stable value
		#define ACCEL_NOISE_BASE 8.000
		#define ACCEL_NOISE_SQUELCH	0.001	// Any signals below the base threshold are multiplied by the squelch factor
		#define	ACCEL_NOISELIMIT 500.0	// robot was moved during calibration -- start over
		#define ACCEL_XSENSITIVITY 1
		#define ACCEL_YSENSITIVITY 1
		#define ACCEL_XCENTER 1165
		#define ACCEL_YCENTER 790
		#define ACCEL_ZCENTER 61429
		#define ACCEL_XALIGN	0.030
		#define ACCEL_YALIGN	0.030
		#define ACCEL_FRICTION 2.000	// percent of velocity loss due to friction
	#endif	// ADXL345_ACCEL

	#ifdef MPU3050_GYRO
		#define GYRO_I2C_ADDR		0x68
		#define GYRO_I2C_ALT_ADDR	0x69
		#define GYRO_Reg_WhoAmI	0	//7:I2C_IF_DIS; 1-6: ID; 0:R/W
		#define GYRO_Reg_XoffsH	12	//0-7:DC bias
		#define GYRO_Reg_XoffsL	13	//0-7:subtracted from sensor value
		#define GYRO_Reg_YoffsH	14	//0-7:before going into sensor registers
		#define GYRO_Reg_YoffsL	15
		#define GYRO_Reg_ZoffsH	16
		#define GYRO_Reg_ZoffsL	17
		#define GYRO_Reg_FIFOen	18	//7:temp_out; 6:gyro_Xout; 5:gyro_Yout; 4:gyro_Zout
										//3:aux_Xout: 2:aux_Yout; 1:aux_Zout; 0:FIFO_footer
		#define GYRO_Reg_AUXdio	19	//2:1=VDD, 0=VLOGIC; 0-1,3-7:=0
		#define GYRO_Reg_AUXadr	20	//7:clkout_en; 0-6:AuxID
		#define GYRO_Reg_SMPLRT_DIV	21	//0-7:sample rate divider
		#define GYRO_Reg_DLPF_FS	22	//5-7:ext_sync_set; 3-4:FS_sel; 0-2:DLPF_CFG
										//ext sync: 0=none; 1=temp_out; 2=gyro_Xout; 3=gyro_Yout
										//ext sync: 4=gyro_Zout; 5=aux_Xout; 6=aux_Yout; 7=auxZ_out
										//FS_sel: 0=250d/s; 1=500d/s; 2=1000d/s; 3=2000d/s
										//DLPF_CFG: Sample Rate: 0=8kHz; 1-6=1kHz;
										//DLPF_CFG: Low Pass Filter: 0=256Hz; 1=188Hz; 2=98Hz; 3=42Hz
										//DLPF_CFG: Low Pass Filter: 4=20Hz; 5=10Hz; 6=5Hz
		#define GYRO_Reg_INT_CFG	23	//7:ACTL; 6:OPEN; 5:Latch_Int_en; 4:Int_anyRD
										//3:0; 2:MPU_Rdy_en; 1:DMP_done_en; 0:Raw_Rdy_en
		#define GYRO_Reg_AuxBur	24	//0-7: Burst Address
		#define GYRO_Reg_INT_STATUS	26	//2:MPU_Rdy; 1:DMP_done; 0:Raw_Rdy
		#define GYRO_Reg_TEMP_OUT_H	0x1B	//0-7: high 8-bits of 16-bit data
		#define GYRO_Reg_TEMP_OUT_L	0x1C	//0-7: high 8-bits of 16-bit data
		#define GYRO_Reg_XOUT_H		0x1D
		#define GYRO_Reg_XOUT_L		0x1E
		#define GYRO_Reg_YOUT_H		0x1F
		#define GYRO_Reg_YOUT_L		0x20
		#define GYRO_Reg_ZOUT_H		0x21
		#define GYRO_Reg_ZOUT_L		0x22
		#define GYRO_Reg_AuxXH	35
		#define GYRO_Reg_AuxXL	36
		#define GYRO_Reg_AuxYH	37
		#define GYRO_Reg_AuxYL	38
		#define GYRO_Reg_AuxZH	39
		#define GYRO_Reg_AuxZL	40
		#define GYRO_Reg_FIFOcH	58	//0-2: high 2 bits of 10 bit counter
		#define GYRO_Reg_FIFOcL	58	//0-7: low 8 bits of 10 bit counter
		#define GYRO_Reg_FIFOd	60	//0-7: 8-bit data in FIFO queue
		#define GYRO_Reg_User	61	//6:FIFO_en; 5:Aux_IF_en; 3:Aux_IF_rst; 1:FIFO_rst; 0:Gyro_rst
		#define GYRO_Reg_PWR_MGM	62	//7:H_reset; 6:sleep; 5:StbyX; 4:StbyY; 3:StbyZ
										//0-2:Clk_Sel: 0=IntOsc; 1=gyroX; 2=gyroY; 3=gyroZ
										//             4=ext32.768MHZ; 5=ext19.2MHz; 6:rsvd; 7:stop clock

		#define GYRO_ZoutSync	0x80	//10000000
		#define GYRO_2000dps	0x18	//00011000
		#define GYRO_1000dps	0x10	//00010000
		#define GYRO_8kHz		0x00	//00000000
		#define GYRO_1kHz		0x01	//00000001
		#define GYRO_SampleDiv	0x0F	//00010000	= new sample every 2 msec (we check every 2 ms)
		#define GYRO_H_Reset	0x80	//10000000
		#define GYRO_Sleep		0x40	//01000000
		#define GYRO_Stby_GX	0x20	//00100000
		#define GYRO_Stby_GY	0x10	//00010000
		#define GYRO_Stby_GZ	0x08	//00001000
		#define GYRO_Zoffset	0x05	//00000101
		#define GYRO_DataRdy	0x01	//00000001	= enable data ready bit
		#define GYRO_Latch		0x20	//00100000	= latch the data ready bit until cleared
		#define GYRO_AnyRd2Clr	0x10	//00010000	= clear data ready bit on any register read
		#define GYRO_FIFO_Tmp	0x80	//10000000	= put Temp data on FIFO queue
		#define GYRO_FIFO_GX	0x40	//01000000	= put gyro X-axis data on FIFO queue
		#define GYRO_FIFO_GY	0x20	//00100000	= put gyro Y-axis data on FIFO queue
		#define GYRO_FIFO_GZ	0x10	//00010000	= put gyro Z-axis data on FIFO queue
		#define GYRO_FIFO_AX	0x08	//00001000	= put accel X-axis data on FIFO queue
		#define GYRO_FIFO_AY	0x04	//00000100	= put accel Y-axis data on FIFO queue
		#define GYRO_FIFO_AZ	0x02	//00000010	= put accel Z-axis data on FIFO queue
		#define GYRO_FIFO_ft	0x01	//00000001	= put footer place holder on FIFO queue
	#endif	// MPU3050_GYRO

	#ifdef MPU6050_6DOF
		#define IMU_I2C_ADDR		0x68
		#define IMU_I2C_ALT_ADDR	0x69
		#define DOF6_READ_FIFO_CNT	2	// need to acquire fifo queue count
		#define DOF6_FRAME_SIZE		6	// 14 bytes of data per frame includes all sensors on the chip
		#define DOF6_FIFO_SIZE		1024
		#define DOF6_READ_DATA_LEN	15	// read all data values from the onboard sensors
										// it appears that burst reads autoincrement the reg addr even when
										// addressing the FIFO register.  Wah!
										// fifo frame contains AccXH, AccXL, AccYH, AccYL, AccZH, AccZL
										// [skip TempH, TempL], GyroXH, GyroXL, GyroYH, GyroYL, GyroZH, GyroZL
										// repeat frame reads while fifo queue counts > 12
										// Note:  Skipping temp. To add temp, read 14 at a time
		#define DOF6_Reg_SMPLRT_DIV	0x19	//0-7 bits gyro output rate / (1 + SMPLRT_DIV)
			#define DOF6_SampleDiv8	0x07	//00000111	= new sample every 1 msec (we check every 1.5 ms)
		#define DOF6_Reg_CONFIG		0x1A	//3-5:ext_sync_set; 0-2:DLPF_CFG
									//ext sync: 0=none; 1=temp_out; 2=gyro_Xout; 3=gyro_Yout
									//ext sync: 4=gyro_Zout; 5=accel_Xout; 6=accel_Yout; 7=accel_Zout
									//DLPF_CFG: Sample Rate: 0=8kHz; 1-6=1kHz;
									//DLPF_CFG: Low Pass Filter: 0=256Hz; 1=188Hz; 2=98Hz; 3=42Hz
									//DLPF_CFG: Low Pass Filter: 4=20Hz; 5=10Hz; 6=5Hz
			#define DOF6_ZoutSync	0x20	//10000000
			#define DOF6_8kHz		0x00	//00000000
			#define DOF6_1kHz		0x01	//00000001
		#define DOF6_Reg_GCONFIG	0x1B	//3-4:FS_SEL; 5:ZG_ST; 6:YG_ST; 7:XG_ST
			#define DOF6_2000dps	0x18	//00011000
			#define DOF6_1000dps	0x10	//00010000
			#define DOF6_500dps		0x08	//00011000
			#define DOF6_250dps		0x00	//00010000
		#define DOF6_Reg_ACONFIG	0x1C	//3-4:AFS_SEL; 5:ZA_ST; 6:YA_ST; 7:XA_ST
			#define DOF6_2g			0x00
			#define DOF6_4g			0x08
			#define DOF6_8g			0x10
			#define DOF6_16g		0x18
		#define DOF6_Reg_MOT_THR	0x1F	// 0-7: Motion Detection Threshold
		#define DOF6_Reg_FIFO_EN	0x23	//7:temp_out; 6:gyro_Xout; 5:gyro_Yout; 4:gyro_Zout
											//3:accel_out; 2:slv2_out; 1:slv1_out; 0:slv0_out
			#define DOF6_FIFO_Tmp	0x80	//10000000	= put Temp data on FIFO queue
			#define DOF6_FIFO_GX	0x40	//01000000	= put gyro X-axis data on FIFO queue
			#define DOF6_FIFO_GY	0x20	//00100000	= put gyro Y-axis data on FIFO queue
			#define DOF6_FIFO_GZ	0x10	//00010000	= put gyro Z-axis data on FIFO queue
			#define DOF6_FIFO_ACC	0x08	//00001000	= put accel data on FIFO queue
			#define DOF6_FIFO_S2	0x04	//00000100	= put slave 2 data on FIFO queue
			#define DOF6_FIFO_S1	0x02	//00000010	= put slave 1 data on FIFO queue
			#define DOF6_FIFO_S0	0x01	//00000001	= put slave 0 data on FIFO queue
		#define DOF6_Reg_INT_CFG	0x37	//7:Int_level; 6:Int_open; 5:Latch_Int_en; 4:Int_rd_clr
											//3:FS_int_lvl; 2:FS_int_en; 1:i2c_bypass_en; 0:-
			#define DOF6_Latch		0x20	//00100000	= latch the data ready bit until cleared
			#define DOF6_AnyRd2Clr	0x10	//00010000	= clear data ready bit on any register read
		#define DOF6_Reg_INT_ENA	0x38	//7:-; 6:MOT_en; 5:-; 4:FIFO_overflow_en
											//3:i2c_mst_int_en; 2:-; 1:-; 0:Data_Rdy_en
			#define DOF6_DataRdy	0x01	//00000001	= enable data ready bit
		#define DOF6_Reg_INT_STATUS	0x3A	//6:MOT_int; 4:FIFO_Ov_int; 3:i2c_mst_int; 0:Data_Rdy_int
		#define DOF6_Reg_Accel_XH	0x3B
		#define DOF6_Reg_Accel_XL	0x3C
		#define DOF6_Reg_Accel_YH	0x3D
		#define DOF6_Reg_Accel_YL	0x3D
		#define DOF6_Reg_Accel_ZH	0x3F
		#define DOF6_Reg_Accel_ZL	0x40
		#define DOF6_Reg_TEMP_H		0x41	//0-7: high 8-bits of 16-bit data
		#define DOF6_Reg_TEMP_L		0x42	//0-7: low 8-bits of 16-bit data
		#define DOF6_Reg_GYRO_XH	0x43
		#define DOF6_Reg_GYRO_XL	0x44
		#define DOF6_Reg_GYRO_YH	0x45
		#define DOF6_Reg_GYRO_YL	0x46
		#define DOF6_Reg_GYRO_ZH	0x47
		#define DOF6_Reg_GYRO_ZL	0x48
		#define DOF6_Reg_MOT_CTL	0x69	//4-5:accel_pwr_on_delay
			#define DOF6_PON_DELAY	0x30	//delay accel samples for 7ms after power on (limit garbage)
		#define DOF6_Reg_USER_CTL	0x6A	//6:FIFO_en; 5:i2c_mst_en; 4:i2c_if_dis; 2:fifo_reset
											//1:i2c_mst_reset; 0:sig_cond_reset
			#define DOF6_FIFO_EN	0x40	//enable fifo buffer
			#define DOF6_FIFO_RST	0x04	//reset fifo buffer; set FIFO_EN=0, then FIFO_RST=1, then FIFO_EN=1
		#define DOF6_Reg_PWR_MGT1	0x6B	//7:dev_reset; 6:sleep; 5:cycle; 3:temp_dis
											//0-2:Clk_Sel: 0=Int8MHz; 1=gyroX; 2=gyroY; 3=gyroZ
											//             4=ext32.768KHZ; 5=ext19.2KHz; 6:-; 7:stop clock
			#define DOF6_Dev_Reset	0x80	//10000000
			#define DOF6_Sleep		0x40	//01000000
			#define DOF6_PLL_Z		0x03	//00000011
		#define DOF6_Reg_PWR_MGT2	0x6B	//6-7:lp_wake_ctl; 5:stbyXA; 4:stbyYA; 3:StbyZA
											//2:stbyXG; 1:stbyYG; 0:stbyZG
			#define DOF6_Stby_AX	0x20	//00100000
			#define DOF6_Stby_AY	0x10	//00010000
			#define DOF6_Stby_AZ	0x08	//00001000
			#define DOF6_Stby_GX	0x20	//00000100
			#define DOF6_Stby_GY	0x10	//00000010
			#define DOF6_Stby_GZ	0x08	//00000001
		#define DOF6_Reg_FIFO_CNT_H	0x72	//0-2: high 2 bits of 10 bit counter
		#define DOF6_Reg_FIFO_CNT_L	0x73	//0-7: low 8 bits of 10 bit counter
		#define DOF6_Reg_FIFO_DATA	0x74	//0-7: 8-bit data in FIFO queue
		#define DOF6_Reg_WhoAmI		0x75	//7:-; 1-6: ID; 0:R/W	= 0x68
	#endif	// MPU6050_6DOF


};


#endif