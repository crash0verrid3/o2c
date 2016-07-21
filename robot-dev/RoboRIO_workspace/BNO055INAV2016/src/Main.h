//if it's not defined yet, define it
//it shouldn't be. This is just a formality.
#ifndef MAIN_H
#define MAIN_H

#include <Definitions.h>
#include "RobotBase.h"
#include "Timer.h"
//#include "WPILib.h"

//#include "Robot.h"

class MAIN : public IterativeRobot
{
public:
	MAIN(); //constructifcate
	virtual void StartCompetition(); //runs once at the start. loops internally.

private:
	double lastloop;
	double looptime;

	Timer FieldTimer;
//	ROBOT Robot;

};

#endif
