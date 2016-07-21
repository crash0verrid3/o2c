#ifndef Definitions_H
#define Definitions_H

#include "SmartDashboard/SendableChooser.h"
#include "SmartDashboard/SmartDashboard.h"

#define LOOPSPERSEC 50

//CONTROLS //should be in Dinput, mode off
#define AXIS_DRIVEX 1
#define AXIS_DRIVEY 2
#define AXIS_DRIVER 3
#define BTN_ACTIVEROT 12
#define BTN_ACTIVEOFF 9
#define BTN_DRIVEMODE 3
#define BTN_DRIVESPRINT 7

#define BTN_CATSHOOT 1 //fire shooter
#define BTN_CATRESET 24 //reset shooter
#define BTN_CATSTATEOFF 10
#define BTN_CATSTATEON 9
//#define BTN_CATKILL 11 //stop the firing sequence, go to idle state
#define BTN_CATMANFIRE 2
#define AXIS_CATMANPOWER 6

#define BTN_CLAWSTART 22
#define BTN_CLAWSCORE 21
#define BTN_CLAWDOWN 24
#define BTN_CLAWCATAPULT 25
#define BTN_CLAWMANDOWN 8
#define BTN_CLAWMANUP 6

//AUTONOMOUS
#define AUTO_MOVETIME 1.75
#define AUTO_STOPTIME 0
#define AUTO_RESETTIME 0
#define AUTO_FIRETIME 1.78
#define AUTO_FIREDONE 10

#define PHOTOMODULE 1
#define PHOTOCHANNEL 3
#define ULTRACLAWMODULE 1
#define ULTRACLAWCHANNEL 3

//DRIVETRAIN
#define LDRIVEVICTORMODULE 1
#define LDRIVEVICTORCHANNEL 3

#define RDRIVEVICTORMODULE 1
#define RDRIVEVICTORCHANNEL 2

#define BDRIVEVICTORMODULE 1
#define BDRIVEVICTORCHANNEL 1

#define ROBOTGYROMODULE 1
#define ROBOTGYROCHANNEL 2

#define ACTIVESENSE 10

//drive state machines
#define MOSEY_ALONG 1
#define CRAZY_IVAN 2

#define ORIENT_FIELD 1
#define ORIENT_ROBOT 2
#define ORIENT_BTNHELD 3


//SHOOTER
#define LAUNCHMOTORMODULE 1
#define LAUNCHMOTORCHANNEL 10

#define LOADMOTORMODULE 1
#define LOADMOTORCHANNEL 9

#define SENCODERACHANNEL 1
#define SENCODERBCHANNEL 2

#define SHOOTERSWITCHMODULE 1
#define SHOOTERSWITCHCHANNEL 6

#define RESET_STOP 30
#define SHOOTER_RESETANG 50
#define SHOOTER_LAUNCHTIME 0.5
#define SHOOTER_RESTANG_SLOWZONE 300
#define CAT_CIRCLE 360

//shooter state machine
#define CATAPULT_IDLE 1
#define CATAPULT_LAUNCHING 2
#define CATAPULT_RESET 3
#define CATAPULT_RESETING 4
#define CATAPULT_MANUAL 5

//CLAW
#define CLAWMOTORMODULE 1
#define CLAWMOTORCHANNEL 6

#define CLAWENCODERMOD1 1
#define CLAWENCODERCHAN1 3

#define CLAWENCODERMOD2 1
#define CLAWENCODERCHAN2 4

#define SLOWZONE 120
#define CLAWPRECISION 5

#define CLAWANGLE_START 0
#define CLAWANGLE_SCORE -60
#define CLAWANGLE_DOWN 0
#define CLAWANGLE_CATAPULT -140

//claw state machine
#define CLAW_START 1
#define CLAW_SCORE 2
#define CLAW_DOWN 3
#define CLAW_CATAPULT 4
#define CLAW_GOINGSTART 5
#define CLAW_GOINGSCORE 6
#define CLAW_GOINGDOWN 7
#define CLAW_GOINGCATAPULT 8

#define CLAW_MANUAL 1
#define CLAW_STATE 2

//Array for holding input
struct INPUT //holds all the input from the driverstation/dashboard
{
	//drive inputs
	double DriveX; //drive x
	double DriveY; //drive y
	double DriveR; //drive z/spin
	bool ActiveRotation;
	bool ActiveOff;
	bool FieldMode;
	bool Sprint;
	
	//shooter inputs
	bool shoot; //firin mah lazor
	bool reset; //chargin mah lazor
	bool catkill; //no more lazor
	bool catmanfire;
	bool catstateon;
	bool catstateoff;
	double catmanpower;
	
	
	//claw inputs
	bool gotostart; //claw to start position
	bool gotoscore; //claw to scoring position
	bool gotodown; //claw to down position
	bool gotocatapult; //claw dump to catapult
	bool clawmandown;
	bool clawmanup;
	
	double RoboGyroVal;
	
	/*
	//Aton Tracking Data;
	int autoruns;
	int atonRunsStart;
	int atonFireDelay;
	double atonForwardPower;
	bool atonFire;
	bool atonReadyToFire;
	bool isAton;
	*/
	
	
	
};

struct OUTPUT //holds all the output from the robot to the dashboard
{
	
};


//make input and output available to any class that includes Definitions.h
//extern INPUT input;
//extern OUTPUT output;

#define PI 3.14159

#endif
