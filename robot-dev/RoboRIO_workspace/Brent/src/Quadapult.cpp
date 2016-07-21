#include "Quadapult.h"

QUADAPULT :: QUADAPULT() : Tension_Detect(TENSION_COUNTER_CHANNEL), Tensioner(TENSIONMOTORCHANNEL), Launcher(LAUNCHMOTORCHANNEL)
{
	catinitcnt = 0;
	CatapultState = CATAPULT_TENSION_CHECK;
	tension_count = 0;
	prev_tension_count = -1;
	prev2_tension_count = -1;
	fire_power = CATAPULT_FIRE_POWER;
	tension_power = CATAPULT_TENSION_POWER;
	tension_assist_power = CATAPULT_TENSION_ASSIST_POWER;
	tension_assist_time = CATAPULT_TENSION_ASSIST_TIME;
}
/*
 * Components:  3 victors feeding 3 miniCIMs driving a quad gearbox to turn the choo-choo and tension the catapult
 * 				1 victor feeding the fourth miniCIM in the quad gearbox to fire the catapult
 * 				1 cam switch activated by the choo-choo CAM
 * 				1 external PWM generator to tension the catapult while the robot is disabled
 * Operation:
 * 		PWM output #9 is wired through the CAM switch and split to the three tensioning victors
 * 		The external PWM generator is wired in parallel with PWM #9 to perform this function while
 * 		the robot is disabled.  Once tensioned, the PWM generator is turned off, so that there is
 * 		no interference between the two signals when the robot is enabled.  When the choo-choo
 * 		rotates to where the CAM engages the switch, the PWM signal is cut off from reaching the
 * 		tensioning victors and is redirected to the Tension_Detect counter.  As long as the
 * 		Tension_Detect counter is increasing, the catapult is ready to fire.
 *
 * 		PWM #10 is wired directly to the firing victor.  It is set to CATAPULT_FIRE_POWER when the
 * 		BTN_CATSHOOT is pressed.  The firing motor only needs to move the
 * 		choo-choo CAM past the CAM switch.  Once the CAM switch disengages, the signal from PWM #9
 * 		is removed from the Tension_Detect counter and directed to the tensioning victors.  This
 * 		starts a complete cycle (including launching).  If the Tension_Detect counter continues
 * 		to increase for more than CATAPULT_MISFIRE pulses, we failed to fire (may need more power).
 *
 * 		At the start of the launch cycle, we start the ShootTimer.  If the ShootTimer exceeds the
 * 		CATAPULT_TENSION_TIMEOUT value, the catapult failed to tension in the allotted time.  This
 * 		is most likely due to low battery power (towards the end of a match).  In order to avoid
 * 		dropping power to the comms and losing access to the robot altogether, we need to stop
 * 		the tensioning cycle and allow the system to recover, before making another attempt to
 * 		tension the catapult.  The next attempt will need to increase the tensioning power and
 * 		also use the lauching motor to assist (more on that later).
 *
 * 		Once the catapult is fully tensioned, the PWM signal is redirected from the tensioning victors
 * 		to the Tension_Detect counter.  If the catapult tensioning operation does not stop quickly
 * 		enough (too much power = too much speed = blows past the CAM switch before the choo-choo
 * 		stops turning) a new launch cycle begins (a misfire).  We want to see at least CATAPULT_MISFIRE
 * 		PWM pulses on the Tension_Detect counter (=0.4 seconds) to assure us that we have stopped the
 * 		choo-choo at the fully tensioned position.  If we detect a misfire, we need to lower the
 * 		catapult tensioning power slightly until the misfiring stops.
 *
 * 		For the sake of efficiency, we want to run the firing motor at low power during the tensioning
 * 		operation.  This allows the tensioning motors to operate without having to overcome the
 * 		resistance of the firing motor through the quad gearbox.  At the start of the launch cycle,
 * 		we set the power on the firing motor to CATAPULT_TENSION_ASSIST_POWER for CATAPULT_TENSION_ASSIST_TIME
 * 		seconds (or until the CAM switch engages, whichever comes first).  If the CAM switch engages before
 * 		CATAPULT_TENSION_ASSIST_TIME seconds, we drop the CATAPULT_TENSION_ASSIST_TIME slightly for the next
 * 		lauch cycle (which might also help avoid misfires)
 *
 * 		Pressing button BTN_CATMANUAL_OPERATION, toggles input->catmanual between manual operation
 * 		and state machine operation.  State machine operation is as described above.  In manual operation
 * 		mode, all four victors are operated by pressing and releasing the BTN_CATSHOOT button.
 */

void QUADAPULT :: Init()
{
	catinitcnt++;
	SmartDashboard::PutNumber("CatapultInit", catinitcnt);
	Tensioner.Set(0);
	Launcher.Set(0);
	ShootTimer.Start();
	Tension_Detect.SetSemiPeriodMode(true);
	Tension_Detect.SetSamplesToAverage(1);
	//Tension_Detect.StartLiveWindowMode();
	//Tension_Detect.Start();
	Tension_Detect.Reset();
	catinitcnt++;
	SmartDashboard::PutNumber("CatapultInit", catinitcnt);
}

void QUADAPULT :: Run(INPUT* input)
{
	if (input->catmanual == true) CatapultState = CATAPULT_MANUAL;
	else if (CatapultState == CATAPULT_MANUAL) CatapultState = CATAPULT_INITIAL;
	//prev_tension_count = Tension_Detect.Get();

	tension_count = Tension_Detect.Get();
	switch (CatapultState)
		{
		case CATAPULT_MANUAL:		// manual operation
			if (input->fire_catapult)
				{
				Tensioner.Set(fire_power);
				Launcher.Set(fire_power);
				}
			else
				{
				Tensioner.Set(0);
				Launcher.Set(0);
				}
			break;
		case CATAPULT_INITIAL:		// initial state
			Tension_Detect.Reset();
			ShootTimer.Reset();
			Tensioner.Set(tension_power);
			Launcher.Set(tension_assist_power);
			CatapultState = CATAPULT_ASSISTED_TENSIONING;
			// break;	// don't wait for next cycle, go immediately to next case
		case CATAPULT_ASSISTED_TENSIONING:		// Assisted tensioning
			if (ShootTimer.Get() > tension_assist_time)
				{
				Launcher.Set(0);
				CatapultState = CATAPULT_UNASSISTED_TENSIONING;
				}
			else if (tension_count > 0)	// CAM switch activated before Catapult State 3
				{						// not sure if a switch bounce could be picked up as a false positive here
				tension_assist_time -= 0.1;	// reduce Catapult State 2 time by 0.1 sec
				Launcher.Set(0);			// and terminate tension assist
				CatapultState = CATAPULT_UNASSISTED_TENSIONING;
				}
			break;
		case CATAPULT_UNASSISTED_TENSIONING:		// Unassisted tensioning
			 if (tension_count > 0)	// CAM switch activated.  Catapult is tensioned
				{
				Tensioner.Set(0);			// and terminate tension assist
				prev_tension_count = -1;	// prevent false misfire detect in next stage
				CatapultState = CATAPULT_TENSION_CHECK;
				}
			 else if (ShootTimer.Get() > CATAPULT_TENSION_TIMEOUT)	// failed to tension (outajuice)
				{
				Tensioner.Set(0);		// allow the system to recover
				tension_power = fmin(1, tension_power+0.1);	// increase the power for the next attempt
				tension_assist_power = fmin(1, tension_assist_power+0.1);	// increase assist power
				tension_assist_time = fmin(CATAPULT_TENSION_TIMEOUT,tension_assist_time+0.1); // increase time
				CatapultState = CATAPULT_INITIAL;		// restart the cycle
				}
			break;
		case CATAPULT_TENSION_CHECK:		// Tension Check for misfire
			if (tension_count > CATAPULT_MISFIRE)	// CAM switch has remained activated.
				{
				CatapultState = CATAPULT_READY;
				}
			else if (tension_count == prev2_tension_count)	// Tension_Detect is not increasing
				{											// we have a misfire (tensioning did not stop)
				Tensioner.Set(0);		// allow the system to recover
				tension_power -= 0.1;	// decrease the power for the next attempt
				CatapultState = CATAPULT_INITIAL;		// restart the cycle
				}
			break;
		case CATAPULT_READY:		// Catapult is tensioned and ready to fire
			if (input->fire_catapult)
				{
				Tension_Detect.Reset();
				CatapultState = CATAPULT_FIRING;
				}
			 break;		// Don't wait another cycle; to straight to Firing state
		case CATAPULT_FIRING:		// Firing
			Tensioner.Set(fire_power);
			Launcher.Set(fire_power);
			tension_count = Tension_Detect.Get();
			if (tension_count > 0)	// CAM switch still activated.  Catapult has not yet fired.
				{
				if (tension_count > CATAPULT_MISFIRE)	// Catapult failed to fire.  Need more power?
					{
					fire_power = fmin(1, fire_power+0.1);	// increase fire_power
					}
				Tension_Detect.Reset();
				}
			else CatapultState = CATAPULT_INITIAL;		// Catapult has fired.  Restart the cycle
			break;
		}
	prev2_tension_count = prev_tension_count;
	prev_tension_count = tension_count;

	SmartDashboard::PutNumber("ShooterState", CatapultState);
	SmartDashboard::PutNumber("ShooterTimer", ShootTimer.Get());
	SmartDashboard::PutNumber("TensionDetect", prev_tension_count);
	SmartDashboard::PutNumber("TensionPower", tension_power);
	SmartDashboard::PutNumber("AsssistPower", tension_assist_power);
	SmartDashboard::PutNumber("AsssistTime", tension_assist_time);
	SmartDashboard::PutNumber("FirePower", fire_power);

	SmartDashboard::PutBoolean("Direction",Tension_Detect.GetDirection());
	SmartDashboard::PutNumber("Index",Tension_Detect.GetFPGAIndex());
	//SmartDashboard::PutNumber("Index",Tension_Detect.GetIndex());
	SmartDashboard::PutNumber("Period",Tension_Detect.GetPeriod());
	SmartDashboard::PutNumber("AvgBits",Tension_Detect.GetSamplesToAverage());
	SmartDashboard::PutBoolean("Stopped",Tension_Detect.GetStopped());


}
