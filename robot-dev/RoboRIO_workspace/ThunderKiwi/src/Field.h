//if it's not defined yet, define it
//it shouldn't be. This is just a formality.
#ifndef Field_H
#define Field_H

//Include stuff that isn't ours
#include "RobotBase.h"
#include "Timer.h"
//#include "Watchdog.h"
#include "WPILib.h"

//here's our stuff
#include "Robot.h"
#include "Definitions.h"

class FIELD : public RobotBase
{
public:
	FIELD(); //constructifcate
	virtual void StartCompetition(); //runs once at the start. loops internally.

private:
    //define variables
	int looprate;
	double lastloop;
	double looptime;

	//for initializations
	int runs;
	int autoruns;
	int teleruns;
	int testruns;
	int disabledruns;
	int enabledruns;

	//has stuff
	Timer FieldTimer;
	ROBOT Robot;

};

#endif
