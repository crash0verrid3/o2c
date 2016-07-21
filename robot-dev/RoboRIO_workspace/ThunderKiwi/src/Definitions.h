#ifndef Definitions_H
#define Definitions_H

#include "SmartDashboard/SendableChooser.h"
#include "SmartDashboard/SmartDashboard.h"

typedef unsigned char byte;  // make it more Arduino like


//General Defines
#define PI 3.14159

// FieldDefines
#define LOOPSPERSEC 50
#define	LOOPPERIOD 0.02
#define	LOOPPERIODMIN 0.0001		// 0.00015 is as fast as the cRio can run


//CONTROLS //should be in Dinput, mode off
#define AXIS_DRIVEX 1
#define AXIS_DRIVEY 2
#define AXIS_DRIVER 3
#define BTN_DRIVEMODE 3
#define BTN_DRIVESPRINT 2



//DRIVETRAIN
#define FLVICTORCHANNEL 0
#define BLVICTORCHANNEL 1
#define FRVICTORCHANNEL 2
#define BRVICTORCHANNEL 3

//DRIVETRAIN state machines
#define ORIENT_BTNHELD 0
#define ORIENT_ROBOT 1
#define ORIENT_FIELD 2
#define SPRINT_INVALID 0
#define SPRINT_THIRD	33
#define SPRINT_HALF 50
#define SPRINT_TWOTHIRDS	66
#define SPRINT_FULL 100


typedef unsigned char byte;  // make it more Arduino like

//Array for holding input
struct INPUT //holds all the input from the driverstation/dashboard
{
	//drive inputs
	double DriveX; //drive x
	double DriveY; //drive y
	double DriveR; //drive z/spin
	bool OrientModeBtn;
	bool SprintModeBtn;

};

static struct RobotStateBuffer	// holds the state of all Robot Subsystems
{
	int		Robot_Init;

	int		Comms_Init;

	int		Drivetrain_Init;
	double	Drivetrain_FL_Motor_Power;
	double	Drivetrain_FR_Motor_Power;
	double	Drivetrain_BL_Motor_Power;
	double	Drivetrain_BR_Motor_Power;
	enum	Drivetrain_Speed_Control {Slow, Normal, Fast};	// motor = pwr * (2^Drivetrain_Speed_Control) / 4
	bool	Drivetrain_Spin_Control;	// true when gamepad spin control == 0; false otherwise
	double	Drivetrain_Heading;			// if == 0 && Drivetrain_Spin_Control, set to Navigation_Heading;
	enum	Drivetrain_Directional_Orientation {Robot, Field};

	int		Navigation_Init;
	double	Navigation_Heading;
	int		Navigation_Position_X;
	int		Navigation_Position_Y;
	double	Navigation_Direction;
	double	Navigation_Speed;
	bool	Navigation_Robot_Lost;		// autorun needs to check this and if set, disable all motors and wait for teleop
	bool	Navigation_GyroTilt;		// gyro_spin exceeded maximum range

	int		Tote_Lift_Init;

	int		Claw_Init;

	int		Tote_Wheels_Init;

} *pMyRobotState;

#define NAVIGATION_QUEUE_SIZE	200	// we will be long lost before we fill this queue

// Inertial Navigation System
#define SAMPLEPERIOD 0.001		// sample gyro and accelerometer inputs every 1.0 msec (~1k Hz)
#define INSUPDATEPERIOD 0.01	// Update positon, orientation, speed, heading, spin every 10 msec


#define GYRO_TYPE_ADXL345_ANALOG
#ifdef GYRO_TYPE_ADXL345_ANALOG
/* This value set yields an average of 2 data points per cycle @ 0.6ms cycle time
 * high speed turns in both directions loosing 0.3 degrees per revolution
 */
	#define GYRO_READ_CNT 2			// only acquiring the Z axis data (2 bytes) = spin rate
	#define GYRO_ANALOGCHANNEL 0
	#define THERMOSTAT_ANALOGCHANNEL 1
	#define GYRO_OVERSAMPLEBITS 5
	#define GYRO_SAMPLEAVGBITS 0
	#define GYRO_SAMPLERATE 50000
	#define	GYRO_ANALOG_NOISELIMIT 500	// robot was moved during calibration -- start over
	#define GYRO_ANALOG_CW_SENSITIVITY	5552.1
	#define GYRO_ANALOG_CCW_SENSITIVITY	5583.5
	#define GYRO_ANALOG_RATECENTER 66031550.000
	#define GYRO_ANALOG_CW_SPIN_CORRECTION	3.0	// Offset to Gyro_Sensitivity to account for data loss during high speed spins
	#define GYRO_ANALOG_CCW_SPIN_CORRECTION	-0.1	// Offset to Gyro_Sensitivity to account for data loss during high speed spins
#endif



struct OUTPUT //holds all the output from the robot to the dashboard
{
// Plans for the Future.
};



#endif
