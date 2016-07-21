#include "O2CClaw.h"

O2CClaw :: O2CClaw() : ClawMotor(CLAWMOTORMODULE,CLAWMOTORCHANNEL), ClawEncoder(CLAWENCODERMOD1, CLAWENCODERCHAN1, CLAWENCODERMOD2, CLAWENCODERCHAN2)
{
	
}

void O2CClaw :: Init()
{
	State = CLAW_START;
	ClawEncoder.Start();
	ClawEncoder.Reset();
	ClawMode = CLAW_DOWN;
	//SmartDashboard::PutNumber("ClawEncoderValue", 0);
	SmartDashboard::PutNumber("ClawState", State);
	SmartDashboard::PutNumber("ClawEncoder", 0);
	SmartDashboard::PutNumber("ClawPower", 0);
}

void O2CClaw::Run(INPUT* input)
{
	if (false)
	{
	
	//tempencoder = SmartDashboard::GetNumber("ClawEncoderValue");
	switch (State)
	{
	
	
		case CLAW_START: //In start position.
		ClawMotor.Set(0);
		SmartDashboard::PutNumber("ClawPower", 0);
		if(input->gotoscore)//If 2nd button pressed will go to score.
		{
			State = CLAW_GOINGSCORE;
		}
		
		if(input->gotodown)//If 3rd button pressed will go to down.
		{
			State = CLAW_GOINGDOWN;
		}
		
		if(input->gotocatapult) //If 4th button pressed go to catapult.
		{
			State = CLAW_GOINGCATAPULT;
		}
		break;
		
		
		
		
		case CLAW_SCORE: //In score position.
		ClawMotor.Set(0);
		SmartDashboard::PutNumber("ClawPower", 0);
		if(input->gotostart)//If 1st button pressed go to start.
		{
			State = CLAW_GOINGSTART;
		}
		
		if(input->gotodown)//If 3rd button pressed go to down.
		{
			State = CLAW_GOINGDOWN;
		}
		
		if(input->gotocatapult)//If 4th button pressed go to catapult.
		{
			State = CLAW_GOINGCATAPULT;
		}
		
		break;
		
		
		
		
		case CLAW_DOWN: //In down position.
		ClawMotor.Set(0);
		SmartDashboard::PutNumber("ClawPower", 0);
		if(input->gotostart)//If 1st button pressed go to start.
		{
			State = CLAW_GOINGSTART;
		}		
		
		if(input->gotoscore)//If 2nd button pressed go to score.
		{
			State = CLAW_GOINGSCORE;
		}
		
		if(input->gotocatapult)//If 4th button pressed go to catapult.
		{
			State = CLAW_GOINGCATAPULT;
		}
		break;
		
		
		

		case CLAW_CATAPULT: //In catapult position.
		ClawMotor.Set(0);
		SmartDashboard::PutNumber("ClawPower", 0);
		if(input->gotostart)//If 1st button pressed go to start.
		{
			State = CLAW_GOINGSTART;
		}
		
		
		if(input->gotoscore)//If 2nd button pressed go to score.
		{
			State = CLAW_GOINGSCORE;
		}
		
		
		if(input->gotodown)//If 3rd button pressed go the down.
		{
			State = CLAW_GOINGDOWN;
		}
		
		break;
		
		
		
		
		
		
		
		
		
		case CLAW_GOINGSTART: //At the state going to the start position.
		
		difference = CLAWANGLE_START - ClawEncoder.Get(); //The angle the claw needs to be at minus what angle it is at.
		
		direction = PlusMin(difference) * -1; ////Positive goes forward, negative goes backwards.
		difference = fabs (difference);
		
		ClawMotor.Set(min (difference / SLOWZONE, 1) * direction); //Defines when the claw should start to slow down.
		SmartDashboard::PutNumber("ClawPower", min (difference / SLOWZONE, 1) * direction);
		
		if(fabs (ClawEncoder.Get() - CLAWANGLE_START) <= CLAWPRECISION) // When the encoder equals this degree, or is within a degree, the robot will be in start position.
		{
			SmartDashboard::PutNumber("potato", 111);
			State = CLAW_START;//State the claw is going to.
		}
		break;
		
		
		
		case CLAW_GOINGSCORE: //At the state going to the score position.
		
		difference = CLAWANGLE_SCORE - ClawEncoder.Get(); //The angle the claw needs to be at minus what angle it is at.
		
		direction = PlusMin(difference) * -1; //Positive goes forward, negative goes backwards.
		difference = fabs (difference);
		
		ClawMotor.Set(min (difference / SLOWZONE, 1) * direction); //Defines when the claw should start to slow down.
		SmartDashboard::PutNumber("ClawPower", min (difference / SLOWZONE, 1) * direction);
		
		
		if(fabs (ClawEncoder.Get() - CLAWANGLE_SCORE) <= CLAWPRECISION) // When the encoder equals this degree, or is within a degree, the robot will be in score position.
		{
			SmartDashboard::PutNumber("potato", 222);
			State = CLAW_SCORE;//State the claw is going to.
		}
		break;
		
		
		
		case CLAW_GOINGDOWN: //At the state going to the down position.
		
		difference = CLAWANGLE_DOWN - ClawEncoder.Get(); //The angle the claw needs to be at minus what angle it is at.
		
		direction = PlusMin(difference) * -1; //Positive goes forward, negative goes backwards.
		difference = fabs (difference);
		
		ClawMotor.Set(min (difference / SLOWZONE, 1) * direction); //Defines when the claw should start to slow down.
		SmartDashboard::PutNumber("ClawPower", min (difference / SLOWZONE, 1) * direction);
		
		
		if(fabs (ClawEncoder.Get() - CLAWANGLE_DOWN) <= CLAWPRECISION) // When the encoder equals this degree, or is within a degree, the robot will be in down position.
		{
			SmartDashboard::PutNumber("potato", 333);
			State = CLAW_DOWN; //State the claw is going to.
		}
		break;
		
		
		
		case CLAW_GOINGCATAPULT:// At the state going to the catapult position.
		
		difference = CLAWANGLE_CATAPULT - ClawEncoder.Get(); //The angle the claw needs to be at minus what angle it is at.
		
		direction = PlusMin(difference) * -1;//Positive goes forward, negative goes backwards.
		difference = fabs (difference);
		
		ClawMotor.Set(min (difference / SLOWZONE, 1) * direction); //Defines when the claw should start to slow down.
		SmartDashboard::PutNumber("ClawPower", min (difference / SLOWZONE, 1) * direction);
		
		
		if(fabs (ClawEncoder.Get() - CLAWANGLE_CATAPULT) <= CLAWPRECISION) // When the encoder equals this degree, or is within a degree, the robot will be in catapult position.
		{
			SmartDashboard::PutNumber("potato", 444);
			State = CLAW_CATAPULT; //State the claw is going to.
		}
		break;	
		
		default:
		
		break;
	}
	
	SmartDashboard::PutNumber("ClawState", State);
	SmartDashboard::PutNumber("ClawEncoder", ClawEncoder.Get());
	}
	
	
	
	else
	{
		if (input->clawmandown == true)
		{
			ClawMotor.Set(-0.5);
			SmartDashboard::PutNumber("ClawPower", -0.5);
		}
		else if (input->clawmanup == true)
		{
			ClawMotor.Set(0.7);
			SmartDashboard::PutNumber("ClawPower", 0.7);
		}
		else
		{
			ClawMotor.Set(0);
			SmartDashboard::PutNumber("ClawPower", 0);
		}
	}
}

int O2CClaw::PlusMin(double value)
{
	if (value > 0)
	{
		return 1;
	}
	else if (value < 0)
	{
		return -1;
	}
	else
	{
	return 0;
	}
}
