#include "Quadapult.h"

QUADAPULT :: QUADAPULT() : ShooterResetSwitch(SHOOTERSWITCHMODULE, SHOOTERSWITCHCHANNEL), Loader(LOADMOTORMODULE, LOADMOTORCHANNEL), Launcher(LAUNCHMOTORMODULE, LAUNCHMOTORCHANNEL)
{
	
}

void QUADAPULT :: Init()
{
	CatapultState = 1;
	Loader.Set((double) 1);
	ShootTimer.Reset();
	ShootTimer.Start();
}

void QUADAPULT :: Run(INPUT* input)
{
	
	if (input->shoot == true)
	{
		Loader.Set(1);
	}
	else
	{
		Loader.Set(0);
	}
	
	if (input->catmanfire == true)
	{
		Launcher.Set(1);
	}
	else
	{
		Launcher.Set(0);
	}
	
	/*
	else
	{
		
	switch (CatapultState)
	{
	case 1: //loaded
		Launcher.Set(0);
		//Loader.Set(0);
		if (input->catmanfire)
		{
			CatapultState = 2;
			ShootTimer.Reset();
		}
	break;
		
	case 2: //firing
		Launcher.Set((double) 1);
		//Loader.Set(0);
		if (ShootTimer.Get() >= 0.75)
		{
			Launcher.Set(0);
			CatapultState = 3;
		}
	break;
	
	case 3: //loading
		Launcher.Set(0);
		Loader.Set(1);
		if (ShooterResetSwitch.Get() or input->catstateoff)
		{
			Launcher.Set(0);
			CatapultState = 1;
		}
	break;
	}
	}
	*/
	SmartDashboard::PutNumber("ShooterState", CatapultState);
}
