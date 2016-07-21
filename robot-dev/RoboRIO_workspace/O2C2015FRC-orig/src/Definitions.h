#ifndef Definitions_H
#define Definitions_H

#include "SmartDashboard/SendableChooser.h"
#include "SmartDashboard/SmartDashboard.h"
#include "math.h"
#include "WPILib.h"


// ROBOT		Select your Robot
#define PRACTICEBOT
//#define REALBOT

//#define DEBUG_LIFT_INIT

	#ifdef REALBOT
		#define ITG3200_GYRO
		//#define ADXL345_ACCEL
		#define BUILTIN_ACCEL
		#define XYWHEELS
	#endif

	#ifdef PRACTICEBOT
		#define MPU6050_6DOF
//		#define BUILTIN_ACCEL
		#define XYWHEELS
	#endif

#define PI					3.1415926535897932384626433832795
#define DEGREES_TO_RADIANS	0.01745329251994329576923690768489

#define AUTONOMOUS_PERIOD		15000	// 15 seconds Autonomous Time Period
#define AUTONOMOUS_MAX_STEPS	100		// Really!  How many steps can you do in 15 seconds anyway?
#define AUTO_DEFAULT_SCENARIO	0
#ifdef REALBOT
#define XPOWERFACTOR	3		// It take 3 times more power to move sideways
#endif
#ifdef PRACTICEBOT
#define XPOWERFACTOR	3		// It take 3 times more power to move sideways
#endif

// Field loop configuration
#define LOOPSPERSEC 50
#define	LOOPPERIOD 0.02
#define	LOOPPERIODMIN 0.0001		// 0.00015 is as fast as the cRio can run


//CONTROLS //should be in Dinput, mode off
//Drive Controls
#define AXIS_DRIVEX 0
#define AXIS_DRIVEY 1
#define AXIS_DRIVER 2
#define AXIS_LIFTER	3
#define BTN_SPRINTCONTROL	11
#define BTN_SPINCONTROL		12
#define BTN_ORIENTMODE 3
#define LIFT_JOY_BTN 1
#define LIFT_UP_BTN	6
#define LIFT_DN_BTN 8


// Optical Sensor for hot goal detection
#define PHOTOCHANNEL 3

// Ultrasonic Sensor to detect ball near forklift (used in two-ball autonomous)
#define ULTRASONICCHANNEL 3
#define ULTRASONICVOLTSPERINCH 0.0098
#define ULTRASONICSETBACK 6.5
#define ULTRASONICOVERSAMPLEBITS 6
#define ULTRASONICAVERAGEBITS 6




#define FLVICTORCHANNEL 0
#define BLVICTORCHANNEL 1
#define FRVICTORCHANNEL 2
#define BRVICTORCHANNEL 3
#define MINIMUM_POWER_TO_MOVE	10		// less than 10% does not overcome friction/inertia
#define INVERT_Y_AXIS_INPUT	1
#define ACTIVESENSE 250		// Spin Control correction factor.  Lower numbers correct more violently.

//DRIVETRAIN state machine
#define ORIENT_BTNHELD 0
#define ORIENT_ROBOT 1
#define ORIENT_FIELD 2
#define SPRINT_INVALID 0
#define SPRINT_HALF 50
#define SPRINT_FULL 100

//Pincer Defines
#define PINCERUM_CHANNEL 8
#define PINCERLM_CHANNEL 7
#define PINCERUM_UP_POWER -100
#define PINCERLM_UP_POWER -100
#define PINCERUM_DN_POWER 55
#define PINCERLM_DN_POWER 55

// Lift Defines

#define LIFTVICTORCHANNEL 9
#define LIFT_DIO_PORT1	4
#define LIFT_DIO_PORT2	5
#define LIFT_REVERSE_DIRECTION false
#define LIFT_MAX_INIT_CYCLES 25
#ifdef DEBUG_LIFT_INIT
#define LIFT_UP_POWER 	-75					// -75%  Negative is up.
#define LIFT_DN_POWER 	60					// +25%
#endif
#ifndef DEBUG_LIFT_INIT
#define LIFT_DN_POWER 	25					// +25%
#define LIFT_UP_POWER 	-75					// -75%  Negative is up.
#endif
#define LIFT_NO_POWER 	0.00
#define LIFT_MINHOLD_POWER 	0.0				//Negative is up.

#define	LIFT_JOY_OFF	0
#define LIFT_JOY_ON		1
#define LIFT_UP			1
#define LIFT_HOLD		0
#define LIFT_DN 		-1
#define LIFT_CLOSE		40
#define LIFT_HOLD_WINDOW 5
#define LIFT_HOLD_POWER_WINDOW 40
#define LIFT_LIMIT_DN		0
#define LIFT_STEP_UP		285
#define LIFT_TOTE_UP		445
#define LIFT_TOTESTEP_UP	550
#define LIFT_LIMIT_UP		600
//#define JOYLIFT // Variable Lift and no State Machine.
//#define LIFT_MAXHOLD_POWER	-0.5


#define WHEELS_LEFT_ARM_FWD_CHANNEL		0
#define WHEELS_LEFT_ARM_REV_CHANNEL		1
#define WHEELS_RIGHT_ARM_FWD_CHANNEL	2
#define WHEELS_RIGHT_ARM_REV_CHANNEL	3
#define TOTEWHEEL_LEFT_CHANNEL			7
#define TOTEWHEEL_RIGHT_CHANNEL			8
#define WHEELARMS_INIT_DUTY_CYCLE	10		// 10% duty cycle to control actuator speed during robot initialization
#define	WHEELARMS_DUTY_CYCLE		50		// 45% duty cycle to control actuator speed
#define WHEELARMS_MAX_CYCLES		15		// 15 on cycles * 20msec/cycle = 0.3sec/duty_cycle = 0.67sec to open/close arms
#define	TOTEWHEEL_LEFT_IN_POWER		100		// pulling totes in can be reasonably quick as it is generally done 1 at a time
#define TOTEWHEEL_RIGHT_IN_POWER	-100
#define TOTEWHEEL_LEFT_OUT_POWER	60		// pushing totes out requires care, as it may be a stack of many totes
#define TOTEWHEEL_RIGHT_OUT_POWER	-60
#define WHEELARMS_WIDE_OPEN			0		// tote wheel arm positions
#define WHEELARMS_WIDE_TOTE			5
#define WHEELARMS_NORMAL			7
#define WHEELARMS_CLOSED			9
#define WHEELARMS_PINCHED			12
#define WHEELARMS_GRIP				15

#define REPORT_FIELD_STATUS
#define REPORT_FIELD_DEBUG
static struct FieldStateBuffer		// holds the state of the Field Control Systems
	{
	int looprate;
	int totalruns;
	int autoruns;
	int teleruns;
	int testruns;
	int disabledruns;
	int enabledruns;
	} *pMyFieldState;

//typedef unsigned char byte;  // make it more Arduino like
typedef uint8_t byte;  // make it more Arduino like
typedef char* string;

enum SpeedControl {Slow=1, Normal=2, Fast=3, Speedy=4};
enum DriveOrientation {Robot=0, Field=1};

static struct RobotStateBuffer	// holds the state of all Robot Subsystems
	{
	int		Robot_Init;
	double	Robot_Heading;				// Measured (by Navigator) orientation of robot relative to field (preset by DS)
	double	Robot_Direction;			// Measured (by Navigator) direction of mevement on the field (field oriented)
	double	Robot_Speed;				// Measured (by Navigator) speed of the Robot
	double	Robot_Position_X;			// Measured (by Navigator) X coordinate of Robot Position (preset by DS)
	double	Robot_Position_Y;			// Measured (by Navigator) Y coordinate of Robot Position (preset by DS)
	double	Robot_Spin_Rate;			// Measured (by Navigator) used to predict stoping heading after spinning

	int		Comms_Init;

	double	Battery_Voltage;			// Measured Battery Voltage (Analog input or CAN bus interrogation of PDP?)

	int		Drivetrain_Init;
	double	Drivetrain_FL_Motor_Power;
	double	Drivetrain_FR_Motor_Power;
	double	Drivetrain_BL_Motor_Power;
	double	Drivetrain_BR_Motor_Power;
	SpeedControl	Drivetrain_Speed_Control;	// motor = pwr * Drivetrain_Speed_Control / 4
	bool	Drivetrain_Spin_Control;	// enabled/disabled by gamepad button input
	double	Drivetrain_Heading;			// if == 0 && Drivetrain_Spin_Control, set to Navigation_Heading;
	DriveOrientation	Drivetrain_Directional_Orientation;

	int		Navigation_Init;
	bool	Navigation_Robot_Lost;		// autorun needs to check this and if set, disable all motors and wait for teleop
	bool	Navigation_GyroTilt;		// gyro_spin exceeded maximum range

	int		ToteLift_Init;
	int		ToteLift_Position;			// Measured position of the ToteLift arms (not clear if this is an encoder or pot)
	int		ToteLift_Target;			// The position we told the ToteLift arms to move to
	double	ToteLift_Speed;				// Measured speed of ToteLift arms movements
	int		ToteLift_Direction;			// Direction of ToteLift movements
	int		ToteLift_ToteCount;			// number of totes being lifted
	int		ToteLift_JoyMode;			// Joy Stick or StateMachine

	int		Pincer_Init;
	double	Pincer_Arm1_Position;
	double	Pincer_Arm1_Speed;
	double	Pincer_Arm2_Position;
	double	Pincer_Arm2_Speed;
	int		Pincer_Grip_Position;
	double	Pincer_Grip_Speed;
	double	Pincer_Grip_Height;
	double	Pincer_Grip_Reach;

	int		ToteWheels_Init;
	int		ToteWheelArm_Left_Position;		// Values: 0 - WHEELARMS_MAX_CYCLES, represent the position of the arms
	int		ToteWheelArm_Right_Position;	//	preset values defined above
	double	ToteWheels_Left_Speed;
	double	ToteWheels_Right_Speed;

	int		Autonomous_Init;
	int		Autonomous_Step;
	int		Autonomous_Scenario;
	} *pMyRobotState;


//Array for holding input
static struct INPUT //holds all the input from the driverstation/dashboard
	{
	//drive inputs
	double DriveX; 				//drive x
	double DriveY; 				//drive y
	double DriveR; 				//drive z/spin
	double ToteLift;			//right joystick (throttle) for manual control of tote lift
//	double PincerUM;
//	double PincerLM;
	double LeftArm;				// second game controller, drive x controls Left ToteWheel Arm position
	double LeftWheel;			// second game controller, drive y controls Left ToteWheel speed & direction of rotation
	double RightArm;			// second game controller, drive z controls Right ToteWheel Arm position
	double RightWheel;			// second game controller, throttle controls Right ToteWheel speed & direction of rotation
	bool OrientModeBtn;
	bool SprintModeBtn;
	bool SpinControlBtn;
	bool ToteUPBtn;
	bool ToteDNBtn;
	bool ToteJoyBtn;
	} *pMyInput;

struct OUTPUT //holds all the output from the robot to the dashboard
{

};



// Inertial Navigation System
#define I2C_CHANNEL 2				// RoboRIO I2C channel
#define SAMPLEPERIOD 0.001			// I2C bus WPILib function calls are inefficient and take 0.5msec per call
#define INSUPDATEPERIOD 0.01		// Update positon, orientation, speed, heading, spin every 10 msec
#define NAVIGATION_QUEUE_SIZE	200	// we will be long lost before we fill this queue
#define I2C_RCV_DATA_LEN		6	// WPILib call to I2C supports a max transfer packet size of 7 bytes (I don't know why)
/*
 * Good news. the 7 byte limit has been lifted as of 2016
 */


// From here down are the control register definitions for various Gyros and Accelerometers

#ifndef MPU6050_6DOF
#ifdef ANALOG_GYRO
	#define GYRO_ANALOGCHANNEL 0
	/*
	 * This value set yields an average of 1.6 data points every 1ms cycle time
	 * High speed turns in both directions vary by +/- 0.1 degree per revolution
	 */
	#define GYRO_OVERSAMPLEBITS 5
	#define GYRO_SAMPLEAVGBITS 0
	#define GYRO_SAMPLERATE 50000
	#define GYRO_NOISE_BASE 1.500
	#define GYRO_NOISE_SQUELCH	0.010	// Any signals below the base threshold are multiplied by the squelch factor
	#define	GYRO_NOISELIMIT 500	// robot was moved during calibration -- start over
	#define GYRO_CW_SENSITIVITY	-62671.000
	#define GYRO_CCW_SENSITIVITY	-64412.000
	#define GYRO_CW_SPIN_CORRECTION	3.0	// Offset to Gyro_Sensitivity to account for data loss during high speed spins
	#define GYRO_CCW_SPIN_CORRECTION	-0.1	// Offset to Gyro_Sensitivity to account for data loss during high speed spins
	#define GYRO_RATECENTER 59226750.000
	#define GYRO_RATELIMIT 200		// 20d/10msec exceeds the gyro rate limit of 2000dps -- flag it
	#define GYRO_THERMO_SENSITIVITY 2
	#define GYRO_THERMO_CENTER 54050;
	#define GYRO_X_SENSITIVITY	500
	#define GYRO_Y_SENSITIVITY	500
#endif

//#define ITG3200_GYRO
#ifdef ITG3200_GYRO
	/*
	 * This value set yields an average of 1.6 data points every 1ms cycle time
	 * High speed turns in both directions vary by +/- 0.1 degree per revolution
	 */
	#define GYRO_THERMO_SENSITIVITY 2
	#define GYRO_THERMO_CENTER 54050;
	#define GYRO_NOISE_BASE 0.0008
	#define GYRO_NOISE_SQUELCH	0.001	// Any signals below the base threshold are multiplied by the squelch factor
	#define	GYRO_NOISELIMIT 500	// robot was moved during calibration -- start over
	#define GYRO_CW_SENSITIVITY			-100128.571
	#define GYRO_CCW_SENSITIVITY		-100128.571
	#define GYRO_CW_SPIN_CORRECTION		0.000	// Offset to Gyro_Sensitivity to account for data loss during high speed spins
	#define GYRO_CCW_SPIN_CORRECTION	-0.000	// Offset to Gyro_Sensitivity to account for data loss during high speed spins
	#define GYRO_X_SENSITIVITY	500
	#define GYRO_Y_SENSITIVITY	500
	#define GYRO_RATECENTER 2550.000
	#define GYRO_RATELIMIT 200		// 20d/10msec exceeds the gyro rate limit of 2000dps -- flag it
	#define GYRO_READ_CNT 6			// need to acquire all three axes due to tipping effect causes accelerometer drift
									// also grab the Low Order Byte of the temperature

	#define GYRO_I2C_ADDR		0x68
	#define GYRO_I2C_ALT_ADDR	0x69
	#define GYRO_Reg_WhoAmI		0x00	//7:I2C_IF_DIS; 1-6: ID; 0:R/W
	#define GYRO_Reg_SMPLRT_DIV	0x15
		#define GYRO_5Samples	0x04	//00000100	= new sample every .625 msec (we check every 1 ms)
		#define GYRO_6Samples	0x05	//00000101	= new sample every .75 msec (we check every 1 ms)
		#define GYRO_7Samples	0x06	//00000110	= new sample every .875 msec (we check every 1 ms)
	#define	GYRO_Reg_DLPF_FS	0x16
		#define GYRO_2000dps	0x18	//00011000
		#define GYRO_8kHz		0x00	//00000000
		#define GYRO_1kHz		0x01	//00000001
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
#endif


#ifdef ANALOG_THERMOSTAT
	#define THERMOSTAT_ANALOGCHANNEL 1
#endif

#ifdef BUILTIN_ACCEL
	#define ACCEL_XSENSITIVITY 2500.000
	#define ACCEL_YSENSITIVITY 2500.000
	#define ACCEL_XALIGN	-4.000
	#define ACCEL_YALIGN	-4.000
	#define ACCEL_XCENTER -0.003
	#define ACCEL_YCENTER -0.049
	#define ACCEL_ZCENTER 1.016
	#define ACCEL_NOISE_BASE 0.020
	#define ACCEL_NOISE_SQUELCH	0.000	// Any signals below the base threshold are multiplied by the squelch factor
	#define	ACCEL_NOISELIMIT 500.0	// robot was moved during calibration -- start over
	#define ACCEL_FRICTION 0.020	// percent of velocity loss due to friction
#endif

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
#endif

#ifdef MPU3050_GYRO
	#define GYRO_RATECENTER 234000
	#define GYRO_SENSITIVITY 192.6	// convert gyro raw signal to degrees
	#define GYRO_NOISE_BASE 0.0005
	#define	GYRO_NOISELIMIT 500.0	// robot was moved during calibration -- start over
	#define GYRO_READ_CNT 2			// need to acquire only the Z axis spin rate

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
#endif
#endif

#ifdef MPU6050_6DOF
//	#define	MPU6050_USING_FIFO

#ifdef PRACTICEBOT
	#define GYRO_THERMO_SENSITIVITY 2
	#define GYRO_THERMO_CENTER 54050
	#define GYRO_NOISE_BASE 0.300
	#define GYRO_NOISE_SQUELCH	0.100	// Any signals below the base threshold are multiplied by the squelch factor
	#define	GYRO_NOISELIMIT 500	// robot was moved during calibration -- start over
	#define GYRO_X_SENSITIVITY	500
	#define GYRO_Y_SENSITIVITY	500
	#define GYRO_CW_SENSITIVITY			60550.000
	#define GYRO_CCW_SENSITIVITY		60550.000
	#define GYRO_CW_SPIN_CORRECTION		0.015	// Offset to Gyro_Sensitivity to account for data loss during high speed spins
	#define GYRO_CCW_SPIN_CORRECTION	0.015	// Offset to Gyro_Sensitivity to account for data loss during high speed spins
	#define GYRO_RATECENTER -10450.000
	#define GYRO_RATELIMIT 100		// 10d/10msec exceeds the gyro rate limit of 1000dps -- flag it
	#define GYRO_READ_CNT 6			// need to acquire all three axes due to tipping effect causes accelerometer drift
									// also grab the Low Order Byte of the temperature
	#define ACCEL_READ_CNT	6	// 2 bytes per axis
	#define ACCEL_NOISE_BASE 250.000
	#define ACCEL_NOISE_SQUELCH	0.001	// Any signals below the base threshold are multiplied by the squelch factor
	#define	ACCEL_NOISELIMIT 500.0	// robot was moved during calibration -- start over
	#define ACCEL_XSENSITIVITY 25
	#define ACCEL_YSENSITIVITY 150
	#define ACCEL_XCENTER 1112.500
	#define ACCEL_YCENTER -860.000
	#define ACCEL_ZCENTER 14690.000
	#define ACCEL_XALIGN	0.000
	#define ACCEL_YALIGN	0.000
	#define ACCEL_FRICTION 0.000	// percent of velocity loss due to friction that is lost in the Noise Cancellation
#endif


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
	#define DOF6_I2C_ADDR		0x68
	#define DOF6_I2C_ALT_ADDR	0x69
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
#endif

#ifdef XYWHEELS

#ifdef REALBOT
	#define	XWHEEL_IN_PER_TICK_FWD	0.0518	// 19.2 encoder ticks equals 1 inch of travel
	#define	XWHEEL_IN_PER_TICK_REV	0.0517	// 19.8 encoder ticks equals 1 inch of travel (wheel slips less in reversed direction)
	#define	YWHEEL_IN_PER_TICK_FWD	0.0507	// 22.1 encoder ticks equals 1 inch of travel
	#define	YWHEEL_IN_PER_TICK_REV	0.05034	// 19.6 encoder ticks equals 1 inch of travel
	#define	XWHEEL_ANGLE_OFFSET		-1			// X wheel is -1 degrees off relative to robot directional reference
	#define	YWHEEL_ANGLE_OFFSET		-1			// Y wheel is -1 degrees off relative to robot directional reference
	#define XWHEEL_REVERSE_DIRECTION false
	#define YWHEEL_REVERSE_DIRECTION false
	#define XWHEEL_DIO_PORT1	0
	#define XWHEEL_DIO_PORT2	1
	#define YWHEEL_DIO_PORT1	2
	#define YWHEEL_DIO_PORT2	3
#endif

#ifdef PRACTICEBOT
	#define	XWHEEL_IN_PER_TICK_FWD	0.0467	// 19.2 encoder ticks equals 1 inch of travel
	#define	XWHEEL_IN_PER_TICK_REV	0.05222	// 19.8 encoder ticks equals 1 inch of travel (wheel slips less in reversed direction)
	#define	YWHEEL_IN_PER_TICK_FWD	0.05215	// 22.1 encoder ticks equals 1 inch of travel
	#define	YWHEEL_IN_PER_TICK_REV	0.05151	// 19.6 encoder ticks equals 1 inch of travel
	#define	XWHEEL_ANGLE_OFFSET		-3.0		// X wheel is -2 degrees off relative to robot directional reference
	#define	YWHEEL_ANGLE_OFFSET		-3.0		// Y wheel is -2 degrees off relative to robot directional reference
	#define XWHEEL_REVERSE_DIRECTION false
	#define YWHEEL_REVERSE_DIRECTION false
	#define XWHEEL_DIO_PORT1	0
	#define XWHEEL_DIO_PORT2	1
	#define YWHEEL_DIO_PORT1	2
	#define YWHEEL_DIO_PORT2	3
#endif

#endif


#endif
