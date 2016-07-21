#include "Robot.h"

ROBOT :: ROBOT() : PhotoSensor(PHOTOMODULE, PHOTOCHANNEL), UltraClaw(ULTRACLAWMODULE, ULTRACLAWCHANNEL)
{

}

void ROBOT :: Init()
{
	//inits go here
	Drivetrain.Init();
	Catapult.Init();
	Forklift.Init();
	PrevAngle = 0;
	
}

/*
void ROBOT :: Aton(INPUT* input)
{

	//Check for first time and start forward
	
	if (input->atonReadyToFire == FALSE)  // Prevent the Delay Count increase until Ready To Fire.
	{
		input->atonFireDelay = input->autoruns;
	}
	
	if ( input->autoruns - input->atonRunsStart <= LOOPSPERSEC * 1) // Drive Forward Time in Seconds.
	{
		Drivetrain.AtonKiwiDrive(input, 0, .5, .0);
	}
	else
	{
		Drivetrain.AtonKiwiDrive(input, 0.0, 0.0, 0.0);
		input->atonReadyToFire = TRUE;
		
	}
	
	if ( input->autoruns - input->atonFireDelay > LOOPSPERSEC ) // Wait 1 sec to Fire after Ready.
	{
		input->atonFire = TRUE;
	}
	
	//Catapult.Run(input);
	
	return;
}
*/

void ROBOT :: Drive(INPUT* input)
{
	Drivetrain.KiwiDrive(input);
	//tell drivetrain to drive
}

void ROBOT :: Shooter(INPUT* input)
{
	Catapult.Run(input);
}

void ROBOT :: Claw(INPUT* input)
{
	Forklift.Run(input);
}

void ROBOT :: GyroCore(INPUT* input)
{
	/*
	double rate = Drivetrain.RobotGyro.GetRate();
	rate = 10 * rate;
	rate = int(rate);
	rate = rate / 10;
	input->RoboGyroVal += rate;
	*/
	
	double rawangle = Drivetrain.RobotGyro.GetAngle();
	double change = rawangle - PrevAngle;
	PrevAngle = rawangle;
	change = (int(10 * change)) / 10;
	input->RoboGyroVal += change;
	
}
