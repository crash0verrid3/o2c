#include "O2CCATAPULT.h"

O2CCATAPULT :: O2CCATAPULT() : Sencoder(SENCODERACHANNEL, SENCODERBCHANNEL), LaMotor(LAUNCHMOTORMODULE, LAUNCHMOTORCHANNEL), ShooterResetSwitch(SHOOTERSWITCHMODULE, SHOOTERSWITCHCHANNEL)
{

}

void O2CCATAPULT :: Init()
{
	CatapultState = CATAPULT_RESET; //Starts the Catapult in an idle state to wait for a ball.
	//TimesShot = 0;	
	Sencoder.Reset();
	Sencoder.Start();
	ShootTimer.Reset();
	ShootTimer.Start();
	SmartDashboard::PutNumber("CatEncoderValue", 0);
	//tempencoder = 0;
	SmartDashboard::PutNumber("ShooterEncoder", Sencoder.Get());
}

void O2CCATAPULT :: Run(INPUT* input)
{

	//tempencoder = SmartDashboard::GetNumber("CatEncoderValue");
	if (input->catstateoff)
		{
			CatapultState = CATAPULT_MANUAL;
		}
	switch (CatapultState)
	{
		case CATAPULT_MANUAL:
			if (input->catstateon)
			{
				CatapultState = CATAPULT_RESETING;
				break;
			}
			if (input->catmanfire == TRUE)
				{
					LaMotor.Set(1); // + (double) 1)/ (double) 2);
					SmartDashboard::PutNumber("ShooterPower", 1); //input->catmanpower); // + (double) 1)/ (double) 2);
				}	
				
				
				else 
					{
						LaMotor.Set(0);
					}	
			break;
			
		case CATAPULT_IDLE: //Idle State. means it's up
			//Sencoder.Reset();
			LaMotor.Set(0); //Tells launch motor not to move.
			SmartDashboard::PutNumber("ShooterPower", 0);
			if (input->reset == true) //If shooter button is being pressed...
			{
				CatapultState = CATAPULT_RESETING; //...Switch to the launch state.
			}
		break; //End of Idle State
		
		case CATAPULT_LAUNCHING: //launch State means going back up
			
//			if ( ( -1 * (Sencoder.Get() % CAT_CIRCLE ) <= SHOOTER_LAUNCHANG ) || 
//					( -1 * (Sencoder.Get() % CAT_CIRCLE) >= SHOOTER_RESTANG_SLOWZONE ) ) //(360 * TimesShot) + SHOOTER_LAUNCHANG)

			Sencoder.Reset();  // Rezero the encoder.
			if ( ShootTimer.Get() <= SHOOTER_LAUNCHTIME )
			{
				SmartDashboard::PutNumber("ShooterPower", .45);
				LaMotor.Set(.25);
			}
			else
			{
				SmartDashboard::PutNumber("ShooterPower", 0);
				LaMotor.Set(0);
				//TimesShot ++;
				CatapultState = CATAPULT_IDLE;
			}
			/*if (input->catkill == true)
			{
				SmartDashboard::PutNumber("ShooterPower", 0);
				LaMotor.Set(0);
				//TimesShot ++;
				CatapultState = CATAPULT_IDLE;
			}*/
		break; //End of Launch State.
		
		case CATAPULT_RESET: //Reset State. means going and staying down.
			/*if (-1 * (Sencoder.Get() % CAT_CIRCLE) <= SHOOTER_RESETANG) //(360 * TimesShot) + SHOOTER_RESETANG)
			{
				SmartDashboard::PutNumber("ShooterPower", .7);
				LaMotor.Set(.7);
			}
			else*/
			{
				SmartDashboard::PutNumber("ShooterPower", 0);
				LaMotor.Set(0);
				if (input->shoot == true) //|| (input->isAton == true && input->atonFire == true))
				{
					ShootTimer.Reset();
					CatapultState = CATAPULT_LAUNCHING;
				}
			}
		break; //Ends Reset State.
		
		case CATAPULT_RESETING:
			
			
	/*		 if (-1 * (Sencoder.Get() % CAT_CIRCLE) >= SHOOTER_RESTANG_SLOWZONE) //(360 * TimesShot) + SHOOTER_RESETANG)
						{
							SmartDashboard::PutNumber("ShooterPower", .85);
							LaMotor.Set(.85);
							break;
						}
			 
			else if (-1 * (Sencoder.Get() % CAT_CIRCLE) >= SHOOTER_RESETANG) //(360 * TimesShot) + SHOOTER_RESETANG)
						{
							SmartDashboard::PutNumber("ShooterPower", .95);
							LaMotor.Set(.95);
							break;
						}
*/			
			if (ShooterResetSwitch.Get() == 1) //Go until the switch is triggered
			{
				SmartDashboard::PutNumber("ShooterPower", .45);
				LaMotor.Set(.25);
			}
	
	
			else
			{
				LaMotor.Set(0);
				CatapultState = CATAPULT_RESET;
			}
			break;
			
		
	}
	
	//SmartDashboard::PutNumber("TimesFired", TimesShot);
	SmartDashboard::PutNumber("ShootTimer", ShootTimer.Get());
	SmartDashboard::PutNumber("ShooterSwitch", ShooterResetSwitch.Get());
	SmartDashboard::PutNumber("ShooterEncoder", Sencoder.Get() * -1 % CAT_CIRCLE);
	SmartDashboard::PutNumber("ShooterState", CatapultState);
	
} //Ends O2CCATAPULT_CPP

