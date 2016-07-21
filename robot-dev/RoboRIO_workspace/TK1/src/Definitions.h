#ifndef Definitions_H
#define Definitions_H

#include "SmartDashboard/SendableChooser.h"
#include "SmartDashboard/SmartDashboard.h"

//General Defines
#define PI 3.14159

// FieldDefines
#define LOOPSPERSEC 50
#define	LOOPPERIOD 0.02
#define	LOOPPERIODMIN 0.0001		// 0.00015 is as fast as the cRio can run


//CONTROLS //should be in Dinput, mode off
#define AXIS_DRIVEX 0
#define AXIS_DRIVEY 1
#define AXIS_DRIVER 2
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
#define SPRINT_THIRD 33
#define SPRINT_HALF 50
#define SPRINT_TWOTHIRDS 66
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

struct OUTPUT //holds all the output from the robot to the dashboard
{
// Plans for the Future.
};



#endif
