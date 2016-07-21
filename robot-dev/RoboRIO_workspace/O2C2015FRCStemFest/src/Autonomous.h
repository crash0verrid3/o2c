/*
 * Autonomous.h
 *
 *  Created on: Jan 31, 2015
 *      Author: Rod
 */

#ifndef AUTONOMOUS_H
#define AUTONOMOUS_H

#include <Definitions.h>
#include "math.h"
#include "Timer.h"

class AUTONOMOUS
{

public:
	AUTONOMOUS();
	void Init(RobotStateBuffer *pRobotState, INPUT *pInput, RobotStateBuffer *pAutoTarget);
	// pMyRobotState->Autonomous_Scenario tells us which program we will be running
	void SelectScenario();			// Executed on every Disabled Loop to check for an update to the Selected Scenario on the SmartDashboard
	void Start();									// Execute the NoOp with timeout = 15 sec and completed -> Completed_Task[1]
	void RunAuto();									// Execute all autonomous tasks in the defined scenario
	void Stop();									// If we enter TeleOp before the autonomous tasks complete, stop all autonomous actions

private:
	typedef enum {	ToteA =			0,	// take Right tote to the autozone
					ToteB =			1,	// take Center tote to the autozone
					ToteC =			2,	// take Left tote to the autozone
					ToteAB =		3,	// take Right and Center tote to the autozone
					ToteBC =		4,	// take Center and Left tote to the autozone
					ToteABC =		5,	// take Right, Center and Left tote to the autozone
					DoNothingA =	6,	// start in Right side of Landfill and move to Autozone
					DoNothingB =	7,	// start in Center of Landfill and move to Autozone
					DoNothingC =	8,	// start in Left side of Landfill and move to Autozone
					LandA1st =		9,	// take Right tote to the landmark within 5 sec
					LandB1st =		10,	// take Center tote to the landmark within 5 sec
					LandC1st =		11,	// take Left tote to the landmark within 5 sec
					LandA2nd =		12,	// take Right tote to the landmark between 5 and 10 sec
					LandB2nd =		13,	// take Center tote to the landmark between 5 and 10 sec
					LandC2nd =		14,	// take Left tote to the landmark between 5 and 10 sec
					LandAB2nd =		15,	// take Right and Center tote to the landmark between 5 and 10 sec
					LandBC2nd =		16,	// take Center and Left tote to the landmark between 5 and 10 sec
					LandA3rd =		17,	// take Right tote to the landmark after 10 sec
					LandB3rd =		18,	// take Center tote to the landmark after 10 sec
					LandC3rd =		19,	// take Left tote to the landmark after 10 sec
					XLandA1st =		20,	// take Right tote to the landmark within 5 sec (Sideways)
					XLandB1st =		21,	// take Center tote to the landmark within 5 sec (Sideways)
					XLandC1st =		22,	// take Left tote to the landmark within 5 sec (Sideways)
					XLandA2nd =		23,	// take Right tote to the landmark between 5 and 10 sec (Sideways)
					XLandB2nd =		24,	// take Center tote to the landmark between 5 and 10 sec (Sideways)
					XLandC2nd =		25,	// take Left tote to the landmark between 5 and 10 sec (Sideways)
					XLandAB2nd =	26,	// take Right and Center tote to the landmark between 5 and 10 sec (Sideways)
					XLandBC2nd =	27,	// take Center and Left tote to the landmark between 5 and 10 sec (Sideways)
					XLandA3rd =		28,	// take Right tote to the landmark after 10 sec (Sideways)
					XLandB3rd =		29,	// take Center tote to the landmark after 10 sec (Sideways)
					XLandC3rd =		30,	// take Left tote to the landmark after 10 sec (Sideways)
					MockBox =		31,
					Test =			32,
					Demo =			33
		}	AutoScenario;

		/*
		 * ToDo:
		 * 		Add a Label command for branch control
		 * 		Add a Timeout branch parameter to the Wait4 command
		 * 		Add controls to turn on/off spin control, set speed control, and change orientation mode
		 * 		Add controls to query sensors that are not integral components of controlled subsystems
		 * 			for example, an ultrasonic proximity sensor, or a photo-electric sensor.
		 * 		Add control to reset the Robot's Coordinates to the AutoTarget coordinates (Arrive)
		 */
	typedef enum {	NoOp=0,
					Arrive=1,
					SetControls=2,
					Wait4=3,
					MoveTo=4,
					MoveRelative=5,
					TurnTo=6,
					TurnRelative=7,
					ToteWheels=8,
					ToteArms=9,
					ToteLift=10,
					ClawArm = 11,
					Pincer=12
		} AutoAction;

	/*
	 * The following construct lists all the pre-programmed steps for every scenario.
	 * When the SmartDashboard is updated during Disabled operations, the new scenario steps will be loaded into the
	 * pScenario->Steps array from the array below.
	 */

	typedef int		AutoCommand_t[6];

#define LIFTER_FULL_UP	24			// 24 inches off the ground; pneumatic actuator is fully retracted
#define LIFTER_STACK	14			// 14 inches off the ground; clearance to stack on top of another tote
#define LIFTER_PLATFORM	4			// 24 inches off the ground; minimum height to place on the stacking platform
#define LIFTER_FULL_DN	0			// 0 inches off the ground; pneumatic actuator is fully extended

	AutoCommand_t		AutonomousSteps[650] = {
		{ToteA, 19, 92, -117, 270, 0},			// take Right tote to the autozone; 19 steps, xCoord=92, yCoord=-117, Heading=270
			{NoOp, 1, 500, 0, 0, 0},					// wait 1/2 sec to complete initialization
			{Wait4, 1, 500, 0, 0, 0},					// gives pneumatic susbsystems time to initialize
			{ToteLift, 2, 1000, LIFTER_PLATFORM, 0, 0},	// Lift Tote up
			{Wait4, 2, 500, 0, 0, 0},					// gives lifter time to lift the tote off the ground
			{TurnTo, 3, 1000, 0, 50, 0},				// turn up-field @ 50% power
			{Wait4, 3, 250, 0, 0, 0},					// wait 1/4 sec (or until facing up-field) to continue
			{MoveTo, 4, 4000, 92, 0, 80},				// move to autozone at 80% power
			{ToteArms, 5, 500, WHEELARMS_WIDE_OPEN, 0, 0},	// spread tote arms for 1/2 sec (clear area for dropping totes later)
			{Wait4, 4, 4000, 0, 0, 0},					// wait 4 sec to arrive in autozone
			{TurnTo, 6, 1000, 270, 50, 0},				// turn to 270 dg @ 50% power
			{Wait4, 6, 500, 0, 0, 0},					// wait 1/4 sec (or until facing up-field) to continue
			{ToteLift, 7, 1000, LIFTER_FULL_DN, 0, 0},	// put down the tote
			{Wait4, 7, 1000, 0, 0, 0},					// wait up to 1 sec for tote to set down
			{MoveTo, 8, 1000, 120, 0, 75},				// back away from bin and tote
			{ToteArms, 9, 500, WHEELARMS_GRIP, 0, 0},	// pinch tote arms for 1/2 sec (to assist in ejecting totes)
			{ToteWheels, 10, 1000, TOTEWHEEL_LEFT_OUT_POWER, TOTEWHEEL_RIGHT_OUT_POWER, 0},	// 30% to push totes out as we back away to ensure clean release
			{Wait4, 10, 1000, 0, 0, 0},					// wait 1 sec before stopping;  Finish: 7.75 seconds
			{ToteWheels, 11, 500, 0, 0, 0},				// turn off tote wheels
			{ToteArms, 12, 500, WHEELARMS_WIDE_OPEN, 0, 0},	// spread tote arms for 1/2 sec (clear area for dropping totes later)

		{ToteB, 19, 11, -117, 270, 0},			// take Center tote to the autozone
			{NoOp, 1, 500, 0, 0, 0},
			{Wait4, 1, 500, 0, 0, 0},					// gives pneumatic susbsystems time to initialize
			{ToteLift, 2, 500, LIFTER_PLATFORM, 0, 0},	// Lift Tote up
			{Wait4, 2, 500, 0, 0, 0},					// gives lifter time to lift the tote
			{TurnTo, 3, 1000, 0, 50, 0},				// turn up-field @ 50% power
			{Wait4, 3, 250, 0, 0, 0},					// wait 1/4 sec (or until facing up-field) to continue
			{MoveTo, 4, 4000, 11, 0, 80},				// move to autozone at 80% power
			{ToteArms, 5, 500, WHEELARMS_WIDE_OPEN, 0, 0},	// spread tote arms for 1/2 sec (clear area for dropping totes later)
			{Wait4, 4, 4000, 0, 0, 0},					// wait 4 sec to arrive in autozone
			{TurnTo, 6, 1000, 270, 50, 0},				// turn to 270 dg @ 50% power
			{Wait4, 6, 500, 0, 0, 0},					// wait 1/4 sec (or until facing up-field) to continue
			{ToteLift, 7, 1000, LIFTER_FULL_DN, 0, 0},	// put down the tote
			{Wait4, 7, 1000, 0, 0, 0},					// wait up to 1 sec for tote to set down
			{MoveTo, 8, 1000, 29, 0, 75},				// back away from bin and tote
			{ToteArms, 9, 500, WHEELARMS_GRIP, 0, 0},	// pinch tote arms for 1/2 sec (to assist in ejecting totes)
			{ToteWheels, 10, 1000, TOTEWHEEL_LEFT_OUT_POWER, TOTEWHEEL_RIGHT_OUT_POWER, 0},	// 30% to push totes out as we back away to ensure clean release
			{Wait4, 10, 1000, 0, 0, 0},					// wait 1 sec before stopping;  Finish: 7.75 seconds
			{ToteWheels, 11, 500, 0, 0, 0},				// turn off tote wheels
			{ToteArms, 12, 500, WHEELARMS_WIDE_OPEN, 0, 0},	// spread tote arms for 1/2 sec (clear area for dropping totes later)

		{ToteC, 19, -70, -117, 270, 0},			// take Left tote to the autozone
			{NoOp, 1, 500, 0, 0, 0},
			{Wait4, 1, 500, 0, 0, 0},					// gives pneumatic susbsystems time to initialize
			{ToteLift, 2, 500, LIFTER_PLATFORM, 0, 0},	// Lift Tote up
			{Wait4, 2, 500, 0, 0, 0},					// gives lifter time to lift the tote
			{TurnTo, 3, 1000, 0, 50, 0},				// turn up-field @ 50% power
			{Wait4, 3, 250, 0, 0, 0},					// wait 1/4 sec (or until facing up-field) to continue
			{MoveTo, 4, 4000, -70, 0, 80},				// move to autozone at 80% power
			{ToteArms, 5, 500, WHEELARMS_WIDE_OPEN, 0, 0},	// spread tote arms for 1/2 sec (clear area for dropping totes later)
			{Wait4, 4, 4000, 0, 0, 0},					// wait 4 sec to arrive in autozone
			{TurnTo, 6, 1000, 90, 50, 0},				// turn to 090 dg @ 50% power
			{Wait4, 6, 500, 0, 0, 0},					// wait 1/4 sec (or until facing up-field) to continue
			{ToteLift, 7, 1000, LIFTER_FULL_DN, 0, 0},	// put down the tote
			{Wait4, 7, 1000, 0, 0, 0},					// wait up to 1 sec for tote to set down
			{MoveTo, 8, 1000, -89, 0, 75},				// back away from bin and tote
			{ToteArms, 9, 500, WHEELARMS_GRIP, 0, 0},	// pinch tote arms for 1/2 sec (to assist in ejecting totes)
			{ToteWheels, 10, 1000, TOTEWHEEL_LEFT_OUT_POWER, TOTEWHEEL_RIGHT_OUT_POWER, 0},	// 30% to push totes out as we back away to ensure clean release
			{Wait4, 10, 1000, 0, 0, 0},					// wait 1 sec before stopping;  Finish: 7.75 seconds
			{ToteWheels, 11, 500, 0, 0, 0},				// turn off tote wheels
			{ToteArms, 12, 500, WHEELARMS_WIDE_OPEN, 0, 0},	// spread tote arms for 1/2 sec (clear area for dropping totes later)

		{ToteAB, 32, 92, -117, 270, 0},			// take Left and Center totes to the autozone
			{NoOp, 1, 500, 0, 0, 0},
			{Wait4, 1, 500, 0, 0, 0},					// gives pneumatic susbsystems time to initialize
			{ToteLift, 2, 1000, LIFTER_STACK, 0, 0},	// Lift Tote up
			{Wait4, 2, 500, 0, 0, 0},					// gives lifter time to lift the tote
			{MoveTo, 3, 1250, 29, -117, 50},			// move to just short of ToteB
			{Wait4, 3, 1250, 0, 0, 0},					// allow robot to stabilize before moving in to stack totes
			{MoveTo, 4, 1000, 11, -117, 30},			// move to ToteB
			{ToteWheels, 5, 1500, TOTEWHEEL_LEFT_IN_POWER, TOTEWHEEL_RIGHT_IN_POWER, 0},	// 50% to pull totes in
			{ToteArms, 6, 500, WHEELARMS_GRIP, 0, 0},	// pinch tote arms for 1/2 sec (to assist in ingesting tote)
			{Wait4, 4, 500, 0, 0, 0},					// wait 1/2 second before starting to stack totes
			{ToteLift, 7, 1000, LIFTER_FULL_DN, 0, 0},	// stack toteA on top of toteB
			{Wait4, 7, 500, 0, 0, 0},					// wait 1/2 sec for the tote lift to get down enough to stack A on B
			{ToteArms, 8, 500, WHEELARMS_NORMAL, 0, 0},	// spread tote arms for 1/2 sec
			{ToteWheels, 9, 500, 0, 0, 0},				// turn off tote wheels
			{Wait4, 7, 500, 0, 0, 0},					// wait 1/2 sec for the tote lift to get down enough to stack A on B
			{ToteLift, 10, 500, LIFTER_PLATFORM, 0, 0},	// Lift Tote up
			{Wait4, 10, 500, 0, 0, 0},					// gives lifter time to lift the tote
			{TurnTo, 11, 1000, 0, 50, 0},				// turn up-field @ 50% power
			{Wait4, 11, 250, 0, 0, 0},					// wait 1/4 sec (or until facing up-field) to continue
			{MoveTo, 12, 4000, 11, 0, 80},				// move to autozone at 80% power
			{ToteArms, 13, 500, WHEELARMS_WIDE_OPEN, 0, 0},	// spread tote arms for 1/2 sec (clear area for dropping totes later)
			{Wait4, 12, 4000, 0, 0, 0},					// wait 4 sec to arrive in autozone
			{TurnTo, 14, 1000, 270, 50, 0},				// turn to 270 dg @ 50% power
			{Wait4, 14, 500, 0, 0, 0},					// wait 1/4 sec (or until facing up-field) to continue
			{ToteLift, 15, 1000, LIFTER_FULL_DN, 0, 0},	// put down the tote
			{Wait4, 15, 1000, 0, 0, 0},					// wait up to 1 sec for tote to set down
			{MoveTo, 16, 1000, 29, 0, 75},				// back away from bin and tote
			{ToteArms, 17, 500, WHEELARMS_GRIP, 0, 0},	// pinch tote arms for 1/2 sec (to assist in ejecting totes)
			{ToteWheels, 18, 1000, TOTEWHEEL_LEFT_OUT_POWER, TOTEWHEEL_RIGHT_OUT_POWER, 0},	// 30% to push totes out as we back away to ensure clean release
			{Wait4, 18, 1000, 0, 0, 0},					// wait 1 sec before stopping;  Finish: 11 seconds
			{ToteWheels, 19, 500, 0, 0, 0},				// turn off tote wheels
			{ToteArms, 20, 500, WHEELARMS_WIDE_OPEN, 0, 0},	// spread tote arms for 1/2 sec (clear area for dropping totes later)

		{ToteBC, 32, 11, -117, 270, 0},			// take Center and Right tote to the autozone
			{NoOp, 1, 500, 0, 0, 0},
			{Wait4, 1, 500, 0, 0, 0},					// gives pneumatic susbsystems time to initialize
			{ToteLift, 2, 1000, LIFTER_STACK, 0, 0},	// Lift Tote up
			{Wait4, 2, 500, 0, 0, 0},					// gives lifter time to lift the tote
			{MoveTo, 3, 1250, -52, -117, 50},			// move to just short of ToteB
			{Wait4, 3, 1250, 0, 0, 0},					// allow robot to stabilize before moving in to stack totes
			{MoveTo, 4, 1000, -70, -117, 30},			// move to ToteB
			{ToteWheels, 5, 1500, TOTEWHEEL_LEFT_IN_POWER, TOTEWHEEL_RIGHT_IN_POWER, 0},	// 50% to pull totes in
			{ToteArms, 6, 500, WHEELARMS_GRIP, 0, 0},	// pinch tote arms for 1/2 sec (to assist in ingesting tote)
			{Wait4, 4, 500, 0, 0, 0},					// wait 1/2 second before starting to stack totes
			{ToteLift, 7, 1000, LIFTER_FULL_DN, 0, 0},	// stack toteA on top of toteB
			{Wait4, 7, 500, 0, 0, 0},					// wait 1/2 sec for the tote lift to get down enough to stack A on B
			{ToteArms, 8, 500, WHEELARMS_NORMAL, 0, 0},	// spread tote arms for 1/2 sec
			{ToteWheels, 9, 500, 0, 0, 0},				// turn off tote wheels
			{Wait4, 7, 500, 0, 0, 0},					// wait 1/2 sec for the tote lift to get down enough to stack A on B
			{ToteLift, 10, 500, LIFTER_PLATFORM, 0, 0},	// Lift Tote up
			{Wait4, 10, 500, 0, 0, 0},					// gives lifter time to lift the tote
			{TurnTo, 11, 1000, 0, 50, 0},				// turn up-field @ 50% power
			{Wait4, 11, 250, 0, 0, 0},					// wait 1/4 sec (or until facing up-field) to continue
			{MoveTo, 12, 4000, -70, 0, 80},				// move to autozone at 80% power
			{ToteArms, 13, 500, WHEELARMS_WIDE_OPEN, 0, 0},	// spread tote arms for 1/2 sec (clear area for dropping totes later)
			{Wait4, 12, 4000, 0, 0, 0},					// wait 4 sec to arrive in autozone
			{TurnTo, 14, 1000, 90, 50, 0},				// turn to 090 dg @ 50% power
			{Wait4, 14, 500, 0, 0, 0},					// wait 1/4 sec (or until facing up-field) to continue
			{ToteLift, 15, 1000, LIFTER_FULL_DN, 0, 0},	// put down the tote
			{Wait4, 15, 1000, 0, 0, 0},					// wait up to 1 sec for tote to set down
			{MoveTo, 16, 1000, -89, 0, 75},				// back away from bin and tote
			{ToteArms, 17, 500, WHEELARMS_GRIP, 0, 0},	// pinch tote arms for 1/2 sec (to assist in ejecting totes)
			{ToteWheels, 18, 1000, TOTEWHEEL_LEFT_OUT_POWER, TOTEWHEEL_RIGHT_OUT_POWER, 0},	// 30% to push totes out as we back away to ensure clean release
			{Wait4, 18, 1000, 0, 0, 0},					// wait 1 sec before stopping;  Finish: 11 seconds
			{ToteWheels, 19, 500, 0, 0, 0},				// turn off tote wheels
			{ToteArms, 20, 500, WHEELARMS_WIDE_OPEN, 0, 0},	// spread tote arms for 1/2 sec (clear area for dropping totes later)


		{ToteABC, 45, 92, -117, 270, 0},			// take Right tote to the autozone
			{NoOp, 1, 500, 0, 0, 0},
			{Wait4, 1, 500, 0, 0, 0},					// gives pneumatic susbsystems time to initialize
			{ToteLift, 2, 1000, LIFTER_STACK, 0, 0},	// Lift Tote up
			{Wait4, 2, 500, 0, 0, 0},					// gives lifter time to lift the tote
			{MoveTo, 4, 1250, 29, -117, 50},			// move to just short of ToteB
			{Wait4, 4, 1250, 0, 0, 0},					// allow robot to stabilize before moving in to stack totes
			{MoveTo, 5, 1000, 11, -117, 30},			// move to ToteB
			{ToteWheels, 6, 1500, TOTEWHEEL_LEFT_IN_POWER, TOTEWHEEL_RIGHT_IN_POWER, 0},	// 50% to pull totes in
			{ToteArms, 7, 500, WHEELARMS_GRIP, 0, 0},	// pinch tote arms for 1/2 sec (to assist in ingesting tote)
			{Wait4, 5, 500, 0, 0, 0},					// wait 1/2 second before starting to stack totes
			{ToteLift, 8, 1000, LIFTER_FULL_DN, 0, 0},	// stack toteA on top of toteB
			{Wait4, 8, 500, 0, 0, 0},					// wait 1/2 sec for the tote lift to get down enough to stack A on B
			{ToteArms, 9, 500, WHEELARMS_WIDE_TOTE, 0, 0},	// spread tote arms for 1/2 sec
			{ToteWheels, 10, 500, 0, 0, 0},				// turn off tote wheels
			{Wait4, 8, 500, 0, 0, 0},					// wait 1/2 sec for the tote lift to get down enough to stack A on B
			{ToteLift, 11, 1000, LIFTER_STACK, 0, 0},	// Lift Tote up
			{Wait4, 11, 500, 0, 0, 0},					// gives lifter time to lift the tote
			{MoveTo, 12, 1250, -52, -117, 50},			// move to just short of ToteB
			{Wait4, 12, 1250, 0, 0, 0},					// allow robot to stabilize before moving in to stack totes
			{MoveTo, 13, 1000, -70, -117, 30},			// move to ToteB
			{ToteWheels, 14, 1500, TOTEWHEEL_LEFT_IN_POWER, TOTEWHEEL_RIGHT_IN_POWER, 0},	// 50% to pull totes in
			{ToteArms, 15, 500, WHEELARMS_GRIP, 0, 0},	// pinch tote arms for 1/2 sec (to assist in ingesting tote)
			{Wait4, 13, 500, 0, 0, 0},					// wait 1/2 second before starting to stack totes
			{ToteLift, 16, 1000, LIFTER_FULL_DN, 0, 0},	// stack toteA on top of toteB
			{Wait4, 16, 500, 0, 0, 0},					// wait 1/2 sec for the tote lift to get down enough to stack A on B
			{ToteArms, 17, 500, WHEELARMS_NORMAL, 0, 0},	// spread tote arms for 1/2 sec
			{ToteWheels, 18, 500, 0, 0, 0},				// turn off tote wheels
			{Wait4, 16, 500, 0, 0, 0},					// wait 1/2 sec for the tote lift to get down enough to stack A on B
			{ToteLift, 19, 500, LIFTER_PLATFORM, 0, 0},	// Lift Tote up
			{Wait4, 19, 500, 0, 0, 0},					// gives lifter time to lift the tote
			{TurnTo, 20, 1000, 0, 50, 0},				// turn up-field @ 50% power
			{Wait4, 20, 250, 0, 0, 0},					// wait 1/4 sec (or until facing up-field) to continue
			{MoveTo, 21, 4000, -70, 0, 80},				// move to autozone at 80% power
			{ToteArms, 22, 500, WHEELARMS_WIDE_OPEN, 0, 0},	// spread tote arms for 1/2 sec (clear area for dropping totes later)
			{Wait4, 21, 4000, 0, 0, 0},					// wait 4 sec to arrive in autozone
			{TurnTo, 23, 1000, 90, 50, 0},				// turn to 090 dg @ 50% power
			{Wait4, 23, 500, 0, 0, 0},					// wait 1/4 sec (or until facing up-field) to continue
			{ToteLift, 24, 1000, LIFTER_FULL_DN, 0, 0},	// put down the tote
			{Wait4, 24, 1000, 0, 0, 0},					// wait up to 1 sec for tote to set down
			{MoveTo, 25, 1000, -89, 0, 75},				// back away from bin and tote
			{ToteArms, 26, 500, WHEELARMS_GRIP, 0, 0},	// pinch tote arms for 1/2 sec (to assist in ejecting totes)
			{ToteWheels, 27, 1000, TOTEWHEEL_LEFT_OUT_POWER, TOTEWHEEL_RIGHT_OUT_POWER, 0},	// 30% to push totes out as we back away to ensure clean release
			{Wait4, 27, 1000, 0, 0, 0},					// wait 1 sec before stopping;  Finish: 11 seconds
			{ToteWheels, 28, 500, 0, 0, 0},				// turn off tote wheels
			{ToteArms, 29, 500, WHEELARMS_WIDE_OPEN, 0, 0},	// spread tote arms for 1/2 sec (clear area for dropping totes later)


		{DoNothingA, 2, 92, 72, 90, 0},					// start in Right side of Landfill and move to Autozone
			{MoveTo, 4, 5000, 92, 0, 80},				// move to autozone at 67% power
			{Wait4, 4, 5000, 0, 0, 0},

		{DoNothingB, 2, 11, 72, 90, 0},					// start in Center of Landfill and move to Autozone
			{MoveTo, 4, 5000, 11, 0, 80},				// move to autozone at 67% power
			{Wait4, 4, 5000, 0, 0, 0},

		{DoNothingC, 2, -70, 72, 90, 0},				// start in Left side of Landfill and move to Autozone
			{MoveTo, 4, 5000, -70, 0, 80},				// move to autozone at 67% power
			{Wait4, 4, 5000, 0, 0, 0},

		{LandA1st, 19, 92, -117, 270, 0},			// take Right tote to the Landmark within 5 sec
			{NoOp, 1, 500, 0, 0, 0},
			{Wait4, 1, 500, 0, 0, 0},					// gives pneumatic susbsystems time to initialize
			{ToteLift, 2, 500, LIFTER_PLATFORM, 0, 0},	// Lift Tote up
			{Wait4, 2, 250, 0, 0, 0},					// gives lifter time to lift the tote
			{TurnTo, 3, 1000, 330, 50, 0},				// turn up-field @ 50% power
			{Wait4, 3, 250, 0, 0, 0},					// wait 1/4 sec (or until facing up-field) to continue
			{MoveTo, 4, 4000, 0, 0, 80},				// move to autozone at 80% power
			{ToteArms, 5, 500, WHEELARMS_WIDE_OPEN, 0, 0},	// spread tote arms for 1/2 sec (clear area for dropping totes later)
			{Wait4, 4, 2500, 0, 0, 0},					// wait 2.5 sec to arrive in autozone
			{TurnTo, 6, 1000, 270, 50, 0},				// turn to 270 dg @ 50% power
			{Wait4, 6, 500, 0, 0, 0},					// wait 1/4 sec (or until facing up-field) to continue
			{ToteLift, 7, 500, LIFTER_FULL_DN, 0, 0},	// put down the tote
			{Wait4, 7, 500, 0, 0, 0},					// wait up to 1 sec for tote to set down
			{MoveTo, 8, 1000, 120, 40, 75},				// back away from bin and tote (tote delivered in 4.5 sec)
			{ToteArms, 9, 500, WHEELARMS_GRIP, 0, 0},	// pinch tote arms for 1/2 sec (to assist in ejecting totes)
			{ToteWheels, 10, 1000, TOTEWHEEL_LEFT_OUT_POWER, TOTEWHEEL_RIGHT_OUT_POWER, 0},	// 30% to push totes out as we back away to ensure clean release
			{Wait4, 10, 1000, 0, 0, 0},					// wait 1 sec before stopping;  Finish: 5.5 seconds
			{ToteWheels, 11, 500, 0, 0, 0},				// turn off tote wheels
			{ToteArms, 12, 500, WHEELARMS_WIDE_OPEN, 0, 0},	// spread tote arms for 1/2 sec (clear area for dropping totes later)

		{LandB1st, 21, 11, -117, 270, 0},			// take Center tote to the Landmark within 5 sec
			{NoOp, 1, 500, 0, 0, 0},
			{Wait4, 1, 500, 0, 0, 0},					// gives pneumatic susbsystems time to initialize
			{ToteLift, 2, 500, LIFTER_PLATFORM, 0, 0},	// Lift Tote up
			{Wait4, 2, 250, 0, 0, 0},					// gives lifter time to lift the tote
			{TurnTo, 3, 1000, 0, 50, 0},				// turn up-field @ 50% power
			{Wait4, 3, 250, 0, 0, 0},					// wait 1/4 sec (or until facing up-field) to continue
			{MoveTo, 4, 4000, 0, 0, 80},				// move to autozone at 80% power
			{ToteArms, 5, 500, WHEELARMS_WIDE_OPEN, 0, 0},	// spread tote arms for 1/2 sec (clear area for dropping totes later)
			{Wait4, 4, 2500, 0, 0, 0},					// wait 2.5 sec to arrive in autozone
			{TurnTo, 6, 1000, 270, 50, 0},				// turn to 270 dg @ 50% power
			{Wait4, 6, 500, 0, 0, 0},					// wait 1/4 sec (or until facing up-field) to continue
			{ToteLift, 7, 500, LIFTER_FULL_DN, 0, 0},	// put down the tote
			{Wait4, 7, 500, 0, 0, 0},					// wait up to 1 sec for tote to set down
			{MoveTo, 8, 1000, 20, 0, 75},				// back away from bin and tote (tote delivered in 4.5 sec)
			{ToteArms, 9, 500, WHEELARMS_GRIP, 0, 0},	// pinch tote arms for 1/2 sec (to assist in ejecting totes)
			{ToteWheels, 10, 1000, TOTEWHEEL_LEFT_OUT_POWER, TOTEWHEEL_RIGHT_OUT_POWER, 0},	// 30% to push totes out as we back away to ensure clean release
			{Wait4, 10, 1000, 0, 0, 0},					// wait 1 sec before stopping;  Finish: 5.5 seconds
			{ToteWheels, 11, 500, 0, 0, 0},				// turn off tote wheels
			{ToteArms, 12, 500, WHEELARMS_WIDE_OPEN, 0, 0},	// spread tote arms for 1/2 sec (clear area for dropping totes later)
			{MoveTo, 13, 1000, 60, 40, 75},				// get out of the way of the station A bot coming to stack on the landmark
			{Wait4, 13, 1000, 0, 0, 0},					// wait 1 sec before stopping;  Finish: 6.5 seconds

		{LandC1st, 19, -70, -117, 270, 0},			// take Left tote to the Landmark within 5 sec
			{NoOp, 1, 500, 0, 0, 0},
			{Wait4, 1, 500, 0, 0, 0},					// gives pneumatic susbsystems time to initialize
			{ToteLift, 2, 500, LIFTER_PLATFORM, 0, 0},	// Lift Tote up
			{Wait4, 2, 250, 0, 0, 0},					// gives lifter time to lift the tote
			{TurnTo, 3, 1000, 45, 50, 0},				// turn up-field @ 50% power
			{Wait4, 3, 250, 0, 0, 0},					// wait 1/4 sec (or until facing up-field) to continue
			{MoveTo, 4, 4000, 0, 0, 80},				// move to autozone at 80% power
			{ToteArms, 5, 500, WHEELARMS_WIDE_OPEN, 0, 0},	// spread tote arms for 1/2 sec (clear area for dropping totes later)
			{Wait4, 4, 2500, 0, 0, 0},					// wait 2.5 sec to arrive in autozone
			{TurnTo, 6, 1000, 90, 50, 0},				// turn to 270 dg @ 50% power
			{Wait4, 6, 500, 0, 0, 0},					// wait 1/4 sec (or until facing up-field) to continue
			{ToteLift, 7, 500, LIFTER_FULL_DN, 0, 0},	// put down the tote
			{Wait4, 7, 500, 0, 0, 0},					// wait up to 1 sec for tote to set down
			{MoveTo, 8, 1000, -80, 0, 75},				// back away from bin and tote (tote delivered in 4.5 sec)
			{ToteArms, 9, 500, WHEELARMS_GRIP, 0, 0},	// pinch tote arms for 1/2 sec (to assist in ejecting totes)
			{ToteWheels, 10, 1000, TOTEWHEEL_LEFT_OUT_POWER, TOTEWHEEL_RIGHT_OUT_POWER, 0},	// 30% to push totes out as we back away to ensure clean release
			{Wait4, 10, 1000, 0, 0, 0},					// wait 1 sec before stopping;  Finish: 5.5 seconds
			{ToteWheels, 11, 500, 0, 0, 0},				// turn off tote wheels
			{ToteArms, 12, 500, WHEELARMS_WIDE_OPEN, 0, 0},	// spread tote arms for 1/2 sec (clear area for dropping totes later)



			// this is all the farther I got in rewriting the auto scenarios



		{LandA2nd, 22, 92, -117, 270, 0},			// take Right tote to the Landmark between 5 and 10 seconds
			{NoOp, 1, 5000, 0, 0, 0},					// start 5 second timer (give 1st alliance partner time to clear away)
			{ToteLift, 3, 1000, LIFTER_STACK, 0, 0},	// Lift Tote up
			{Wait4, 3, 500, 0, 0, 0},
			{TurnTo, 4, 1000, 340, 50, 0},				// turn toward landmark @ 50% power
			{Wait4, 1, 5000, 0, 0, 0},					// wait for 5 second timer to elapse before moving to landmark
			{MoveTo, 5, 2000, 30, 0, 67},				// move to right of landmark at 67% power
			{ToteArms, 6, 500, WHEELARMS_WIDE_OPEN, WHEELARMS_DUTY_CYCLE, 0},	// spread tote arms for 1/2 sec (prepare to ingest totes)
			{Wait4, 5, 1500, 0, 0, 0},					// wait 1.5 sec (or arrival) before continuing
			{TurnTo, 7, 1000, 270, 50, 0},				// turn back to the left (hopefully aligned with tote on landmark)
			{Wait4, 5, 500, 0, 0, 0},					// wait 0.5 sec to arrive
			{Wait4, 7, 500, 0, 0, 0},					// wait 0.5 sec to orient
			{MoveTo, 8, 1000, 0, 0, 33},				// move to landmark at 33% power
			{ToteWheels, 9, 1500, TOTEWHEEL_LEFT_IN_POWER, TOTEWHEEL_RIGHT_IN_POWER, 0},	// 50% to pull totes in
			{ToteArms, 10, 500, WHEELARMS_GRIP, WHEELARMS_DUTY_CYCLE, 0},	// pinch tote arms for 1/2 sec (to assist in ingesting tote)
			{Wait4, 8, 500, 0, 0, 0},					// wait 1/2 second before starting to stack totes
			{ToteLift, 11, 1000, LIFTER_FULL_DN, 0, 0},	// stack tote on landmark
			{Wait4, 11, 1000, 0, 0, 0},					// give the totes 1 sec to get on the ground
			{MoveTo, 12, 1000, 40, 0, 33},				// back away from landmark.  elapsed time: 10 sec
			{ToteWheels, 13, 1000, TOTEWHEEL_LEFT_OUT_POWER, TOTEWHEEL_RIGHT_OUT_POWER, 0},	// 30% to push totes out as we back away to ensure clean release
			{Wait4, 12, 1000, 0, 0, 0},					// wait up to 1 sec to clear landmark area
			{TurnTo, 14, 1000, 90, 50, 0},				// turn around
			{Wait4, 14, 1000, 0, 0, 0},					// wait 1 sec to turn around

		{LandB2nd, 22, 11, -117, 270, 0},			// take Center tote to the Landmark between 5 and 10 seconds
			{NoOp, 1, 5000, 0, 0, 0},					// start 5 second timer (give 1st alliance partner time to clear away)
			{ToteLift, 3, 1000, LIFTER_STACK, 0, 0},	// Lift Tote up
			{Wait4, 3, 500, 0, 0, 0},
			{TurnTo, 4, 1000, 30, 50, 0},				// turn toward landmark @ 50% power
			{Wait4, 1, 5000, 0, 0, 0},					// wait for 5 second timer to elapse before moving to landmark
			{MoveTo, 5, 2000, 30, 0, 67},				// move to right of landmark at 67% power
			{ToteArms, 6, 500, WHEELARMS_WIDE_OPEN, WHEELARMS_DUTY_CYCLE, 0},	// spread tote arms for 1/2 sec (prepare to ingest totes)
			{Wait4, 5, 1500, 0, 0, 0},					// wait 1.5 sec (or arrival) before continuing
			{TurnTo, 7, 1000, 270, 50, 0},				// turn back to the left (hopefully aligned with tote on landmark)
			{Wait4, 5, 500, 0, 0, 0},					// wait 0.5 sec to arrive
			{Wait4, 7, 500, 0, 0, 0},					// wait 0.5 sec to orient
			{MoveTo, 8, 1000, 0, 0, 33},				// move to landmark at 33% power
			{ToteWheels, 9, 1500, TOTEWHEEL_LEFT_IN_POWER, TOTEWHEEL_RIGHT_IN_POWER, 0},	// 50% to pull totes in
			{ToteArms, 10, 500, WHEELARMS_GRIP, WHEELARMS_DUTY_CYCLE, 0},	// pinch tote arms for 1/2 sec (to assist in ingesting tote)
			{Wait4, 8, 500, 0, 0, 0},					// wait 1/2 second before starting to stack totes
			{ToteLift, 11, 1000, LIFTER_FULL_DN, 0, 0},	// stack tote on landmark
			{Wait4, 11, 1000, 0, 0, 0},					// give the totes 1 sec to get on the ground
			{MoveTo, 12, 1000, 40, 0, 33},				// back away from landmark.  elapsed time: 10 sec
			{ToteWheels, 13, 1000, TOTEWHEEL_LEFT_OUT_POWER, TOTEWHEEL_RIGHT_OUT_POWER, 0},	// 30% to push totes out as we back away to ensure clean release
			{Wait4, 12, 1000, 0, 0, 0},					// wait up to 1 sec to clear landmark area
			{TurnTo, 14, 1000, 90, 50, 0},				// turn around
			{Wait4, 14, 1000, 0, 0, 0},					// wait 1 sec to turn around

		{LandC2nd, 22, -70, -117, 270, 0},			// take Leftt tote to the Landmark between 5 and 10 seconds
			{NoOp, 1, 5000, 0, 0, 0},					// start 5 second timer (give 1st alliance partner time to clear away)
			{ToteLift, 3, 1000, LIFTER_STACK, 0, 0},	// Lift Tote up
			{Wait4, 3, 500, 0, 0, 0},
			{TurnTo, 4, 1000, 30, 50, 0},				// turn toward landmark @ 50% power
			{Wait4, 1, 5000, 0, 0, 0},					// wait for 5 second timer to elapse before moving to landmark
			{MoveTo, 5, 2000, -30, 0, 67},				// move to right of landmark at 67% power
			{ToteArms, 6, 500, WHEELARMS_WIDE_OPEN, WHEELARMS_DUTY_CYCLE, 0},	// spread tote arms for 1/2 sec (prepare to ingest totes)
			{Wait4, 5, 1500, 0, 0, 0},					// wait 1.5 sec (or arrival) before continuing
			{TurnTo, 7, 1000, 90, 50, 0},				// turn to the right (hopefully aligned with tote on landmark)
			{Wait4, 5, 500, 0, 0, 0},					// wait 0.5 sec to arrive
			{Wait4, 7, 500, 0, 0, 0},					// wait 0.5 sec to orient
			{MoveTo, 8, 1000, 0, 0, 33},				// move to landmark at 33% power
			{ToteWheels, 9, 1500, TOTEWHEEL_LEFT_IN_POWER, TOTEWHEEL_RIGHT_IN_POWER, 0},	// 50% to pull totes in
			{ToteArms, 10, 500, WHEELARMS_GRIP, WHEELARMS_DUTY_CYCLE, 0},	// pinch tote arms for 1/2 sec (to assist in ingesting tote)
			{Wait4, 8, 500, 0, 0, 0},					// wait 1/2 second before starting to stack totes
			{ToteLift, 11, 1000, LIFTER_FULL_DN, 0, 0},	// stack tote on landmark
			{Wait4, 11, 1000, 0, 0, 0},					// give the totes 1 sec to get on the ground
			{MoveTo, 12, 1000, -80, 0, 33},				// back away from landmark.  elapsed time: 10 sec
			{ToteWheels, 13, 1000, TOTEWHEEL_LEFT_OUT_POWER, TOTEWHEEL_RIGHT_OUT_POWER, 0},	// 30% to push totes out as we back away to ensure clean release
			{Wait4, 12, 1000, 0, 0, 0},					// wait up to 1 sec to clear landmark area
			{TurnTo, 14, 1000, 90, 50, 0},				// turn around
			{Wait4, 14, 1000, 0, 0, 0},					// wait 1 sec to turn around

		{LandAB2nd, 34, 92, -117, 270, 0},			// take Right and Center totes to the Landmark between 5 and 10 seconds
			{NoOp, 1, 5000, 0, 0, 0},					// start 5 second timer (give 1st alliance partner time to clear away)
			{ToteLift, 3, 1000, LIFTER_STACK, 0, 0},	// Lift Tote up
			{ToteArms, 4, 500, WHEELARMS_WIDE_OPEN, WHEELARMS_DUTY_CYCLE, 0},	// spread tote arms for 1/2 sec
			{Wait4, 3, 500, 0, 0, 0},
			{MoveTo, 5, 1000, 29, -117, 50},			// move to just short of ToteB
			{Wait4, 5, 1000, 0, 0, 0},					// allow robot to stabilize before moving in to stack totes
			{MoveTo, 6, 1000, 11, -117, 50},			// move to ToteB
			{ToteWheels, 7, 1500, TOTEWHEEL_LEFT_IN_POWER, TOTEWHEEL_RIGHT_IN_POWER, 0},	// 50% to pull totes in
			{ToteArms, 8, 500, WHEELARMS_GRIP, WHEELARMS_DUTY_CYCLE, 0},	// pinch tote arms for 1/2 sec (to assist in ingesting tote)
			{Wait4, 6, 500, 0, 0, 0},					// wait 1/2 second before starting to stack totes
			{ToteLift, 9, 1000, LIFTER_FULL_DN, 0, 0},	// stack toteA on top of toteB
			{Wait4, 9, 500, 0, 0, 0},					// wait 1/2 sec for the tote lift to get down enough to stack A on B
			{ToteArms, 10, 500, WHEELARMS_WIDE_OPEN, WHEELARMS_DUTY_CYCLE, 0},	// spread tote arms for 1/2 sec
			{Wait4, 9, 500, 0, 0, 0},					// wait 1/2 sec more for the tote lift to get all the way down
			{ToteLift, 11, 500, LIFTER_STACK, 0, 0},	// Lift Totes A & B up
			{Wait4, 11, 500, 0, 0, 0},					// wait 1/2 second before starting to move
			{TurnTo, 12, 1000, 30, 50, 0},				// turn toward landmark @ 50% power
			{Wait4, 1, 5000, 0, 0, 0},					// wait for 5 second timer to elapse before moving to landmark
			{MoveTo, 13, 2000, 30, 0, 67},				// move to right of landmark at 67% power
			{Wait4, 13, 1500, 0, 0, 0},					// wait 1.5 sec (or arrival) before continuing
			{TurnTo, 14, 1000, 270, 50, 0},				// turn back to the left (hopefully aligned with tote on landmark)
			{Wait4, 13, 500, 0, 0, 0},					// wait 0.5 sec to arrive
			{Wait4, 14, 500, 0, 0, 0},					// wait 0.5 sec to orient
			{MoveTo, 15, 1000, 0, 0, 33},				// move to landmark at 33% power
			{ToteWheels, 16, 1500, TOTEWHEEL_LEFT_IN_POWER, TOTEWHEEL_RIGHT_IN_POWER, 0},	// 50% to pull totes in
			{ToteArms, 17, 500, WHEELARMS_GRIP, WHEELARMS_DUTY_CYCLE, 0},	// pinch tote arms for 1/2 sec (to assist in ingesting tote)
			{Wait4, 15, 500, 0, 0, 0},					// wait 1/2 second before starting to stack totes
			{ToteLift, 18, 1000, LIFTER_FULL_DN, 0, 0},	// stack tote on landmark
			{Wait4, 18, 1000, 0, 0, 0},					// give the totes 1 sec to get on the ground
			{MoveTo, 19, 1000, 40, 0, 33},				// back away from landmark.  elapsed time: 10 sec
			{ToteWheels, 20, 1000, TOTEWHEEL_LEFT_OUT_POWER, TOTEWHEEL_RIGHT_OUT_POWER, 0},	// 30% to push totes out as we back away to ensure clean release
			{Wait4, 19, 1000, 0, 0, 0},					// wait up to 1 sec to clear landmark area
			{TurnTo, 21, 1000, 90, 50, 0},				// turn around
			{Wait4, 21, 1000, 0, 0, 0},					// wait 1 sec to turn around

		{LandBC2nd, 34, 11, -117, 270, 0},			// take Center and Left totes to the Landmark between 5 and 10 seconds
			{NoOp, 1, 5000, 0, 0, 0},					// start 5 second timer (give 1st alliance partner time to clear away)
			{ToteLift, 3, 1000, LIFTER_STACK, 0, 0},	// Lift Tote up
			{ToteArms, 4, 500, WHEELARMS_WIDE_OPEN, WHEELARMS_DUTY_CYCLE, 0},	// spread tote arms for 1/2 sec
			{Wait4, 3, 500, 0, 0, 0},
			{MoveTo, 5, 1000, -52, -117, 50},			// move to just short of ToteB
			{Wait4, 5, 1000, 0, 0, 0},					// allow robot to stabilize before moving in to stack totes
			{MoveTo, 6, 1000, -70, -117, 50},			// move to ToteB
			{ToteWheels, 7, 1500, TOTEWHEEL_LEFT_IN_POWER, TOTEWHEEL_RIGHT_IN_POWER, 0},	// 50% to pull totes in
			{ToteArms, 8, 500, WHEELARMS_GRIP, WHEELARMS_DUTY_CYCLE, 0},	// pinch tote arms for 1/2 sec (to assist in ingesting tote)
			{Wait4, 6, 500, 0, 0, 0},					// wait 1/2 second before starting to stack totes
			{ToteLift, 9, 1000, LIFTER_FULL_DN, 0, 0},	// stack toteA on top of toteB
			{Wait4, 9, 500, 0, 0, 0},					// wait 1/2 sec for the tote lift to get down enough to stack A on B
			{ToteArms, 10, 500, WHEELARMS_WIDE_OPEN, WHEELARMS_DUTY_CYCLE, 0},	// spread tote arms for 1/2 sec
			{Wait4, 9, 500, 0, 0, 0},					// wait 1/2 sec more for the tote lift to get all the way down
			{ToteLift, 11, 500, LIFTER_STACK, 0, 0},	// Lift Totes A & B up
			{Wait4, 11, 500, 0, 0, 0},					// wait 1/2 second before starting to move
			{TurnTo, 12, 1000, 30, 50, 0},				// turn toward landmark @ 50% power
			{Wait4, 1, 5000, 0, 0, 0},					// wait for 5 second timer to elapse before moving to landmark
			{MoveTo, 13, 2000, -30, 0, 67},				// move to left of landmark at 67% power
			{Wait4, 13, 1500, 0, 0, 0},					// wait 1.5 sec (or arrival) before continuing
			{TurnTo, 14, 1000, 90, 50, 0},				// turn to the right (hopefully aligned with tote on landmark)
			{Wait4, 13, 500, 0, 0, 0},					// wait 0.5 sec to arrive
			{Wait4, 14, 500, 0, 0, 0},					// wait 0.5 sec to orient
			{MoveTo, 15, 1000, 0, 0, 33},				// move to landmark at 33% power
			{ToteWheels, 16, 1500, TOTEWHEEL_LEFT_IN_POWER, TOTEWHEEL_RIGHT_IN_POWER, 0},	// 50% to pull totes in
			{ToteArms, 17, 500, WHEELARMS_GRIP, WHEELARMS_DUTY_CYCLE, 0},	// pinch tote arms for 1/2 sec (to assist in ingesting tote)
			{Wait4, 15, 500, 0, 0, 0},					// wait 1/2 second before starting to stack totes
			{ToteLift, 18, 1000, LIFTER_FULL_DN, 0, 0},	// stack tote on landmark
			{Wait4, 18, 1000, 0, 0, 0},					// give the totes 1 sec to get on the ground
			{MoveTo, 19, 1000, -80, 0, 33},				// back away from landmark.  elapsed time: 10 sec
			{ToteWheels, 20, 1000, TOTEWHEEL_LEFT_OUT_POWER, TOTEWHEEL_RIGHT_OUT_POWER, 0},	// 30% to push totes out as we back away to ensure clean release
			{Wait4, 19, 1000, 0, 0, 0},					// wait up to 1 sec to clear landmark area
			{TurnTo, 21, 1000, 270, 50, 0},				// turn around
			{Wait4, 21, 1000, 0, 0, 0},					// wait 1 sec to turn around

		{LandA3rd, 27, 92, -117, 270, 0},			// take Right tote to the Landmark after 10 seconds
			{NoOp, 1, 10000, 0, 0, 0},					// start 10 second timer (give 1st alliance partner time to clear away)
			{ToteLift, 3, 1000, LIFTER_STACK, 0, 0},	// Lift Tote up
			{Wait4, 3, 500, 0, 0, 0},
			{TurnTo, 4, 1000, 0, 50, 0},				// turn toward landmark @ 50% power
			{MoveTo, 5, 2000, 92, 0, 67},				// move to right of landmark at 67% power
			{ToteArms, 6, 500, WHEELARMS_WIDE_OPEN, WHEELARMS_DUTY_CYCLE, 0},	// spread tote arms for 1/2 sec (prepare to ingest totes)
			{Wait4, 5, 1500, 0, 0, 0},					// wait 1.5 sec (or arrival) before continuing
			{TurnTo, 7, 1000, 90, 50, 0},				// turn around
			{Wait4, 7, 1000, 0, 0, 0},					// wait 1 sec to turn around
			{ToteLift, 8, 1000, LIFTER_FULL_DN, 0, 0},	// stack tote on landmark
			{Wait4, 8, 500, 0, 0, 0},					// give the totes 1/2 sec head start to get on the ground
			{ToteLift, 10, 500, LIFTER_STACK, 0, 0},	// Lift Tote back up
			{Wait4, 10, 500, 0, 0, 0},
			{TurnTo, 11, 1000, 270, 50, 0},				// turn toward landmark @ 50% power
			{Wait4, 11, 1000, 0, 0, 0},
			{Wait4, 1, 10000, 0, 0, 0},					// wait for 10 second timer to elapse before moving to landmark
			{MoveTo, 12, 1000, 30, 0, 67},				// move to right of landmark at 67% power
			{Wait4, 12, 1000, 0, 0, 0},					// wait 1 sec (or arrival) before continuing
			{MoveTo, 13, 1000, 0, 0, 33},				// move to landmark at 33% power
			{ToteWheels, 14, 1500, TOTEWHEEL_LEFT_IN_POWER, TOTEWHEEL_RIGHT_IN_POWER, 0},	// 50% to pull totes in
			{ToteArms, 15, 500, WHEELARMS_GRIP, WHEELARMS_DUTY_CYCLE, 0},	// pinch tote arms for 1/2 sec (to assist in ingesting tote)
			{Wait4, 13, 500, 0, 0, 0},					// wait 1/2 second before starting to stack totes
			{ToteLift, 16, 1000, LIFTER_FULL_DN, 0, 0},	// stack tote on landmark
			{Wait4, 16, 1000, 0, 0, 0},					// give the totes 1 sec to get on the ground
			{MoveTo, 17, 1000, 40, 0, 33},				// back away from landmark
			{ToteWheels, 18, 1000, TOTEWHEEL_LEFT_OUT_POWER, TOTEWHEEL_RIGHT_OUT_POWER, 0},	// 30% to push totes out as we back away to ensure clean release
			{Wait4, 17, 1000, 0, 0, 0},					// wait up to 1 sec to clear landmark area.  Elapsed Time: 13.5 sec

		{LandB3rd, 27, 11, -117, 270, 0},			// take Center tote to the Landmark after 10 seconds
			{NoOp, 1, 10000, 0, 0, 0},					// start 10 second timer (give 1st alliance partner time to clear away)
			{ToteLift, 3, 1000, LIFTER_STACK, 0, 0},	// Lift Tote up
			{Wait4, 3, 500, 0, 0, 0},
			{TurnTo, 4, 1000, 30, 50, 0},				// turn toward landmark @ 50% power
			{MoveTo, 5, 2000, 40, 0, 67},				// move to right of landmark at 67% power
			{ToteArms, 6, 500, WHEELARMS_WIDE_OPEN, WHEELARMS_DUTY_CYCLE, 0},	// spread tote arms for 1/2 sec (prepare to ingest totes)
			{Wait4, 5, 1500, 0, 0, 0},					// wait 1.5 sec (or arrival) before continuing
			{TurnTo, 7, 1000, 90, 50, 0},				// turn around
			{Wait4, 7, 1000, 0, 0, 0},					// wait 1 sec to turn around
			{ToteLift, 8, 1000, LIFTER_FULL_DN, 0, 0},	// stack tote on landmark
			{Wait4, 8, 500, 0, 0, 0},					// give the totes 1/2 sec head start to get on the ground
			{ToteLift, 10, 500, LIFTER_STACK, 0, 0},	// Lift Tote back up
			{Wait4, 10, 500, 0, 0, 0},
			{TurnTo, 11, 1000, 270, 50, 0},				// turn toward landmark @ 50% power
			{Wait4, 11, 1000, 0, 0, 0},
			{Wait4, 1, 10000, 0, 0, 0},					// wait for 10 second timer to elapse before moving to landmark
			{MoveTo, 12, 1000, 30, 0, 33},				// move to right of landmark at 67% power
			{Wait4, 12, 1000, 0, 0, 0},					// wait 1 sec (or arrival) before continuing
			{MoveTo, 13, 1000, 0, 0, 33},				// move to landmark at 33% power
			{ToteWheels, 14, 1500, TOTEWHEEL_LEFT_IN_POWER, TOTEWHEEL_RIGHT_IN_POWER, 0},	// 50% to pull totes in
			{ToteArms, 15, 500, WHEELARMS_GRIP, WHEELARMS_DUTY_CYCLE, 0},	// pinch tote arms for 1/2 sec (to assist in ingesting tote)
			{Wait4, 13, 500, 0, 0, 0},					// wait 1/2 second before starting to stack totes
			{ToteLift, 16, 1000, LIFTER_FULL_DN, 0, 0},	// stack tote on landmark
			{Wait4, 16, 1000, 0, 0, 0},					// give the totes 1 sec to get on the ground
			{MoveTo, 17, 1000, 40, 0, 33},				// back away from landmark
			{ToteWheels, 18, 1000, TOTEWHEEL_LEFT_OUT_POWER, TOTEWHEEL_RIGHT_OUT_POWER, 0},	// 30% to push totes out as we back away to ensure clean release
			{Wait4, 17, 1000, 0, 0, 0},					// wait up to 1 sec to clear landmark area.  Elapsed Time: 13.5 sec

		{LandC3rd, 27, 92, -117, 270, 0},			// take Left tote to the Landmark after 10 seconds
			{NoOp, 1, 10000, 0, 0, 0},					// start 10 second timer (give 1st alliance partner time to clear away)
			{ToteLift, 3, 1000, LIFTER_STACK, 0, 0},	// Lift Tote up
			{Wait4, 3, 500, 0, 0, 0},
			{TurnTo, 4, 1000, 30, 50, 0},				// turn toward landmark @ 50% power
			{MoveTo, 5, 2000, -40, 0, 67},				// move to right of landmark at 67% power
			{ToteArms, 6, 500, WHEELARMS_WIDE_OPEN, WHEELARMS_DUTY_CYCLE, 0},	// spread tote arms for 1/2 sec (prepare to ingest totes)
			{Wait4, 5, 1500, 0, 0, 0},					// wait 1.5 sec (or arrival) before continuing
			{TurnTo, 7, 1000, 270, 50, 0},				// turn around
			{Wait4, 7, 1000, 0, 0, 0},					// wait 1 sec to turn around
			{ToteLift, 8, 1000, LIFTER_FULL_DN, 0, 0},	// stack tote on landmark
			{Wait4, 8, 500, 0, 0, 0},					// give the totes 1/2 sec head start to get on the ground
			{ToteLift, 10, 500, LIFTER_STACK, 0, 0},	// Lift Tote back up
			{Wait4, 10, 500, 0, 0, 0},
			{TurnTo, 11, 1000, 90, 50, 0},				// turn toward landmark @ 50% power
			{Wait4, 11, 1000, 0, 0, 0},
			{Wait4, 1, 10000, 0, 0, 0},					// wait for 10 second timer to elapse before moving to landmark
			{MoveTo, 12, 1000, -30, 0, 33},				// move to left of landmark at 33% power
			{Wait4, 12, 1000, 0, 0, 0},					// wait 1 sec (or arrival) before continuing
			{MoveTo, 13, 1000, 0, 0, 33},				// move to landmark at 33% power
			{ToteWheels, 14, 1500, TOTEWHEEL_LEFT_IN_POWER, TOTEWHEEL_RIGHT_IN_POWER, 0},	// 50% to pull totes in
			{ToteArms, 15, 500, WHEELARMS_GRIP, WHEELARMS_DUTY_CYCLE, 0},	// pinch tote arms for 1/2 sec (to assist in ingesting tote)
			{Wait4, 13, 500, 0, 0, 0},					// wait 1/2 second before starting to stack totes
			{ToteLift, 16, 1000, LIFTER_FULL_DN, 0, 0},	// stack tote on landmark
			{Wait4, 16, 1000, 0, 0, 0},					// give the totes 1 sec to get on the ground
			{MoveTo, 17, 1000, -40, 0, 33},				// back away from landmark
			{ToteWheels, 18, 1000, TOTEWHEEL_LEFT_OUT_POWER, TOTEWHEEL_RIGHT_OUT_POWER, 0},	// 30% to push totes out as we back away to ensure clean release
			{Wait4, 17, 1000, 0, 0, 0},					// wait up to 1 sec to clear landmark area.  Elapsed Time: 13.5 sec

		{XLandA1st, 20, 92, -117, 270, 0},			// take Right tote to the Landmark within 5 sec and orient longitudinally
			{ToteLift, 2, 1000, LIFTER_STACK, 0, 0},	// Lift Tote up
			{Wait4, 2, 500, 0, 0, 0},
			{TurnTo, 3, 1000, 330, 50, 0},				// turn toward landmark @ 50% power
			{Wait4, 3, 250, 0, 0, 0},					// wait 1/4 sec (or until facing up-field) to continue
			{MoveTo, 4, 2000, 0, 0, 67},				// move to landmark at 67% power
			{ToteArms, 5, 500, WHEELARMS_WIDE_OPEN, WHEELARMS_DUTY_CYCLE, 0},	// spread tote arms for 1/2 sec (clear area for dropping totes later)
			{Wait4, 4, 1500, 0, 0, 0},					// wait 1.5 sec (or arrival) before continuing
			{TurnTo, 6, 1000, 180, 50, 0},				// turn left to face due South
			{Wait4, 4, 500, 0, 0, 0},					// wait 0.5 sec to arrive
			{Wait4, 6, 500, 0, 0, 0},					// wait 0.5 sec to orient
			{ToteLift, 7, 1000, LIFTER_FULL_DN, 0, 0},	// put down the tote
			{Wait4, 7, 1000, 0, 0, 0},					// give the totes a 1 sec to get on the ground
			{ToteArms, 8, 500, WHEELARMS_GRIP, WHEELARMS_DUTY_CYCLE, 0},	// pinch tote arms for 1/2 sec (to assist in ejecting totes)
			{MoveTo, 9, 1000, 0, 40, 33},				// back away from landmark.  elapsed time: 5.25 sec
			{ToteWheels, 10, 1000, TOTEWHEEL_LEFT_OUT_POWER, TOTEWHEEL_RIGHT_OUT_POWER, 0},	// 30% to push totes out as we back away to ensure clean release
			{Wait4, 9, 1000, 0, 0, 0},					// wait up to 1 sec to clear landmark area
			{TurnTo, 10, 1000, 90, 50, 0},				// turn East
			{Wait4, 10, 500, 0, 0, 0},					// wait 1 sec to turn around
			{MoveTo, 11, 1000, 110, 0, 50},				// go to safe place to drop bin
			{Wait4, 11, 500, 0, 0, 0},					// wait 1 sec to turn around

		{XLandB1st, 20, 11, -117, 270, 0},			// take Center tote to the Landmark within 5 sec
			{ToteLift, 2, 1000, LIFTER_STACK, 0, 0},	// Lift Tote up
			{Wait4, 2, 500, 0, 0, 0},
			{TurnTo, 3, 1000, 0, 50, 0},				// turn toward landmark @ 50% power
			{Wait4, 3, 250, 0, 0, 0},					// wait 1/4 sec (or until facing up-field) to continue
			{MoveTo, 4, 2000, 0, 0, 67},				// move to landmark at 67% power
			{ToteArms, 5, 500, WHEELARMS_WIDE_OPEN, WHEELARMS_DUTY_CYCLE, 0},	// spread tote arms for 1/2 sec (clear area for dropping totes later)
			{Wait4, 4, 1500, 0, 0, 0},					// wait 1.5 sec (or arrival) before continuing
			{TurnTo, 6, 1000, 180, 50, 0},				// turn to face due South
			{Wait4, 4, 500, 0, 0, 0},					// wait 0.5 sec to arrive
			{Wait4, 6, 500, 0, 0, 0},					// wait 0.5 sec to orient
			{ToteLift, 7, 1000, LIFTER_FULL_DN, 0, 0},	// put down the tote
			{Wait4, 7, 1000, 0, 0, 0},					// give the totes a 1 sec to get on the ground
			{ToteArms, 8, 500, WHEELARMS_GRIP, WHEELARMS_DUTY_CYCLE, 0},	// pinch tote arms for 1/2 sec (to assist in ejecting totes)
			{MoveTo, 9, 1000, 0, 40, 33},				// back away from landmark.  elapsed time: 5.25 sec
			{ToteWheels, 10, 1000, TOTEWHEEL_LEFT_OUT_POWER, TOTEWHEEL_RIGHT_OUT_POWER, 0},	// 30% to push totes out as we back away to ensure clean release
			{Wait4, 9, 1000, 0, 0, 0},					// wait up to 1 sec to clear landmark area
			{TurnTo, 10, 1000, 90, 50, 0},				// turn East
			{Wait4, 10, 500, 0, 0, 0},					// wait 1 sec to turn around
			{MoveTo, 11, 1000, 40, 12, 50},				// go to safe place to drop bin
			{Wait4, 11, 500, 0, 0, 0},					// wait 1 sec to turn around

		{XLandC1st, 20, -70, -117, 270, 0},			// take Left tote to the Landmark within 5 sec
			{ToteLift, 2, 1000, LIFTER_STACK, 0, 0},	// Lift Tote up
			{Wait4, 2, 500, 0, 0, 0},
			{TurnTo, 3, 1000, 45, 50, 0},				// turn toward landmark @ 50% power
			{Wait4, 3, 250, 0, 0, 0},					// wait 1/4 sec (or until facing up-field) to continue
			{MoveTo, 4, 2000, 0, 0, 67},				// move to landmark at 67% power
			{ToteArms, 5, 500, WHEELARMS_WIDE_OPEN, WHEELARMS_DUTY_CYCLE, 0},	// spread tote arms for 1/2 sec (clear area for dropping totes later)
			{Wait4, 4, 1500, 0, 0, 0},					// wait 1.5 sec (or arrival) before continuing
			{TurnTo, 6, 1000, 180, 50, 0},				// turn to face South
			{Wait4, 4, 500, 0, 0, 0},					// wait 0.5 sec to arrive
			{Wait4, 6, 500, 0, 0, 0},					// wait 0.5 sec to orient
			{ToteLift, 7, 1000, LIFTER_FULL_DN, 0, 0},	// put down the tote
			{Wait4, 7, 1000, 0, 0, 0},					// give the totes a 1 sec to get on the ground
			{ToteArms, 8, 500, WHEELARMS_GRIP, WHEELARMS_DUTY_CYCLE, 0},	// pinch tote arms for 1/2 sec (to assist in ejecting totes)
			{MoveTo, 9, 1000, 0, 40, 33},				// back away from landmark.  elapsed time: 5.25 sec
			{ToteWheels, 10, 1000, TOTEWHEEL_LEFT_OUT_POWER, TOTEWHEEL_RIGHT_OUT_POWER, 0},	// 30% to push totes out as we back away to ensure clean release
			{Wait4, 9, 1000, 0, 0, 0},					// wait up to 1 sec to clear landmark area
			{TurnTo, 10, 1000, 270, 50, 0},				// turn around
			{Wait4, 10, 1000, 0, 0, 0},					// wait 1 sec to turn around
			{MoveTo, 11, 1000, -100, 0, 50},			// go to safe place to drop bin
			{Wait4, 11, 500, 0, 0, 0},					// wait 1 sec to turn around

			/*
					XLandA2nd =		20,	// take Right tote to the landmark between 5 and 10 sec (Sideways)
					XLandB2nd =		21,	// take Center tote to the landmark between 5 and 10 sec (Sideways)
					XLandC2nd =		22,	// take Left tote to the landmark between 5 and 10 sec (Sideways)
					XLandAB2nd =	23,	// take Right and Center tote to the landmark between 5 and 10 sec (Sideways)
					XLandBC2nd =	24,	// take Center and Left tote to the landmark between 5 and 10 sec (Sideways)
					XLandA3rd =		25,	// take Right tote to the landmark after 10 sec (Sideways)
					XLandB3rd =		26,	// take Center tote to the landmark after 10 sec (Sideways)
					XLandC3rd =		27,	// take Left tote to the landmark after 10 sec (Sideways)
			 */


		{Demo, 82, -72, -120, 0, 0},
			{MoveTo, 50, 2500, 0, -24, 50},
			{Wait4, 50, 500, 0, 0, 0},				// wait up to 1/2 sec before proceeding
			{TurnTo, 51, 500, 45, 30, 0},
			{Wait4, 50, 2500, 0, 0, 0},				// wait up to 2.5 sec before proceeding
			{MoveTo, 1, 500, 16, -16, 50},
			{Wait4, 1, 500, 0, 0, 0},				// wait up to 1/2 sec before proceeding
			{MoveTo, 3, 100, 24, 0, 50},
			{TurnTo, 4, 500, 345, 50, 0},
			{MoveTo, 5, 400, 16, 16, 50},
			{Wait4, 5, 500, 0, 0, 0},				// wait up to 1/2 sec before proceeding
			{MoveTo, 6, 100, 0, 24, 50},
			{TurnTo, 7, 500, 225, 50, 0},
			{MoveTo, 8, 400, -16, 16, 50},
			{Wait4, 8, 500, 0, 0, 0},				// wait up to 1/2 sec before proceeding
			{MoveTo, 9, 100, -24, 0, 50},
			{TurnTo, 10, 500, 135, 50, 0},
			{MoveTo, 11, 400, -16, -16, 50},
			{Wait4, 11, 500, 0, 0, 0},				// wait up to 1/2 sec before proceeding
			{MoveTo, 12, 2500, 67, -110, 50},
			{Wait4, 12, 500, 0, 0, 0},				// wait up to 1/2 sec before proceeding
			{TurnTo, 13, 500, 180, 50, 0},
			{Wait4, 13, 2500, 0, 0, 0},				// wait up to 1/2 sec before proceeding
			{MoveTo, 14, 300, 72, -120, 25},
			{Wait4, 14, 300, 0, 0, 0},				// wait up to .3 sec before proceeding
			{MoveTo, 15, 2500, -16, -16, 50},
			{Wait4, 15, 2500, 0, 0, 0},				// wait up to 2+1/2 sec before proceeding
			{MoveTo, 16, 250, -24, 0, 50},
			{Wait4, 16, 250, 0, 0, 0},				// wait up to 1/4 sec before proceeding
			{MoveTo, 17, 250, -16, 16, 50},
			{Wait4, 17, 250, 0, 0, 0},				// wait up to 1/4 sec before proceeding
			{MoveTo, 18, 250, 0, 24, 50},
			{Wait4, 18, 250, 0, 0, 0},				// wait up to 1/4 sec before proceeding
			{MoveTo, 19, 250, 16, 16, 50},
			{Wait4, 119, 250, 0, 0, 0},				// wait up to 1/4 sec before proceeding
			{MoveTo, 20, 250, 24, 0, 50},
			{Wait4, 20, 250, 0, 0, 0},				// wait up to 1/4 sec before proceeding
			{MoveTo, 21, 250, 16, -16, 50},
			{Wait4, 21, 250, 0, 0, 0},				// wait up to 1/4 sec before proceeding
			{MoveTo, 22, 2500, -67, -110, 50},
			{Wait4, 22, 2500, 0, 0, 0},				// wait up to 2+1/2 sec before proceeding
			{MoveTo, 23, 300, -72, -120, 25},
			{Wait4, 23, 300, 0, 0, 0},				// wait up to .3 sec before proceeding
			{MoveTo, 24, 2500, -3, -30, 50},
			{Wait4, 24, 500, 0, 0, 0},				// wait up to 1/2 sec before proceeding
			{TurnTo, 25, 1000, 0, 50, 0},
			{Wait4, 24, 2000, 0, 0, 0},				// wait up to 2 sec before proceeding
			{Wait4, 25, 1000, 0, 0, 0},				// wait up to 1 sec before proceeding
			{MoveTo, 26, 250, 16, -16, 50},
			{TurnTo, 27, 250, 345, 50, 0},
			{Wait4, 26, 250, 0, 0, 0},				// wait up to 1/4 sec before proceeding
			{MoveTo, 28, 250, 24, 0, 50},
			{TurnTo, 29, 250, 270, 50, 0},
			{Wait4, 28, 250, 0, 0, 0},				// wait up to 1/4 sec before proceeding
			{MoveTo, 30, 250, 16, 16, 50},
			{TurnTo, 31, 250, 225, 50, 0},
			{Wait4, 30, 250, 0, 0, 0},				// wait up to 1/4 sec before proceeding
			{MoveTo, 32, 250, 0, 24, 50},
			{TurnTo, 33, 250, 180, 50, 0},
			{Wait4, 32, 250, 0, 0, 0},				// wait up to 1/4 sec before proceeding
			{MoveTo, 34, 250, -16, 16, 50},
			{TurnTo, 35, 250, 135, 50, 0},
			{Wait4, 34, 250, 0, 0, 0},				// wait up to 1/4 sec before proceeding
			{MoveTo, 36, 250, -24, 0, 50},
			{TurnTo, 37, 250, 90, 50, 0},
			{Wait4, 36, 250, 0, 0, 0},				// wait up to 1/4 sec before proceeding
			{MoveTo, 38, 250, -16, -16, 50},
			{TurnTo, 39, 250, 45, 50, 0},
			{Wait4, 38, 250, 0, 0, 0},				// wait up to 1/4 sec before proceeding
			{MoveTo, 40, 2500, 67, -110, 50},
			{Wait4, 40, 500, 0, 0, 0},				// wait up to 1/2 sec before proceeding
			{TurnTo, 41, 250, 0, 50, 0},
			{Wait4, 40, 2000, 0, 0, 0},				// wait up to 2 sec before proceeding
			{MoveTo, 42, 300, 72, -120, 20},
			{Wait4, 42, 300, 0, 0, 0},				// wait up to .3 sec before proceeding
			{MoveTo, 43, 2500, 16, 16, 50},
			{Wait4, 43, 2500, 0, 0, 0},				// wait up to 2.5 sec before proceeding
			{MoveTo, 44, 250, -16, 16, 50},
			{Wait4, 44, 250, 0, 0, 0},				// wait up to 1/4 sec before proceeding
			{MoveTo, 45, 2500, -67, -110, 50},
			{Wait4, 45, 2500, 0, 0, 0},				// wait up to 2.5 sec before proceeding
			{MoveTo, 46, 300, -72, -120, 20},
			{Wait4, 46, 300, 0, 0, 0},				// wait up to .3 sec before proceeding
		{MockBox, 12, 0, 0, 0, 0},
			{MoveRelative, 1, 1500, 0, 200, 20},	// move straight ahead 200 inches @ 40% power for up to 1.5 sec
			{Wait4, 1, 1500, 0, 0, 0},				// wait up to 1.5 sec before turning
			{TurnRelative, 2, 2000, -90, 25, 0},		// turn 90 degree to the left @ 50% power for up to 2 sec
			{Wait4, 2, 2000, 0, 0, 0},				// wait up to 2 sec before proceding
			{MoveRelative, 3, 1000, -50, 0, 20},	// move straight ahead 50 inches @ 40% power for up to 1 sec
			{Wait4, 3, 1000, 0, 0, 0},				// wait up to 1 sec before proceding
			{TurnRelative, 4, 2000, 90, 25, 0},		// turn 90 degree to the right @ 50% power for up to 2 sec
			{Wait4, 2, 2000, 0, 0, 0},				// wait up to 2 sec before proceding
			{MoveRelative, 5, 1000, 0, -50, 20},	// move backward 50 inches @ 40% power for up to 1 sec
			{Wait4, 5, 1000, 0, 0, 0},				// wait up to 1 sec before proceding
			{MoveRelative, 6, 2000, 50, -150, 20},	// move right 50 inches and back 150 inches @20% power to return to start
			{Wait4, 6, 2000, 0, 0, 0},				// wait up to 2 sec to return to start.  done.
		{MockBox, 24, 0, 0, 0, 0},
			{MoveRelative, 1, 2000, 72, 0, 75},
			{Wait4, 1, 2000, 0, 0, 0},				// wait up to 2 sec before proceeding
			{Arrive, 2, 20, 0, 0, 0},				// with no X/Y wheels, the bot's coordinates remain (0,0), so lie to it
			{MoveRelative, 3, 2000, 0, 72, 75},
			{Wait4, 3, 2000, 0, 0, 0},				// wait up to 2 sec before proceeding
			{Arrive, 4, 20, 0, 0, 0},				// with no X/Y wheels, the bot's coordinates remain (0,0), so lie to it
			{MoveRelative, 5, 2000, -72, 0, 75},
			{Wait4, 5, 2000, 0, 0, 0},				// wait up to 2 sec before proceeding
			{Arrive, 6, 20, 0, 0, 0},				// with no X/Y wheels, the bot's coordinates remain (0,0), so lie to it
			{MoveRelative, 7, 2000, 0, -72, 75},
			{Wait4, 7, 2000, 0, 0, 0},				// wait up to 2 sec before proceeding
			{Arrive, 8, 20, 0, 0, 0},				// with no X/Y wheels, the bot's coordinates remain (0,0), so lie to it
			{MoveRelative, 9, 2000, 72, 72, 90},
			{Wait4, 9, 2000, 0, 0, 0},				// wait up to 2 sec before proceeding
			{Arrive, 10, 20, 0, 0, 0},				// with no X/Y wheels, the bot's coordinates remain (0,0), so lie to it
			{MoveRelative, 11, 2000, -72, 0, 75},
			{Wait4, 11, 2000, 0, 0, 0},				// wait up to 2 sec before proceeding
			{Arrive, 12, 20, 0, 0, 0},				// with no X/Y wheels, the bot's coordinates remain (0,0), so lie to it
			{MoveRelative, 13, 2000, 72, -72, 90},
			{Wait4, 13, 2000, 0, 0, 0},				// wait up to 2 sec before proceeding
			{Arrive, 14, 20, 0, 0, 0},				// with no X/Y wheels, the bot's coordinates remain (0,0), so lie to it
			{MoveRelative, 15, 2000, -72, 0, 75},
			{Wait4, 15, 2000, 0, 0, 0},				// wait up to 2 sec before proceeding
			{Arrive, 16, 20, 0, 0, 0},				// with no X/Y wheels, the bot's coordinates remain (0,0), so lie to it
		};

	Timer	AutonomousTimer;
//	RobotStateBuffer AutoTarget;		// Each autonomous command will
	RobotStateBuffer	*pMyAutoTarget;

	SpeedControl		save_Speed_Control;	// motor = pwr * Drivetrain_Speed_Control / 4
	bool				save_Spin_Control;	// enabled/disabled by gamepad button input
	DriveOrientation	save_Directional_Orientation;

	typedef enum {notstarted=0, started=1, completed=2, timedout=3} CompletionStates;
	CompletionStates	CompletedTasks[AUTONOMOUS_MAX_STEPS]; // each step that has following depencies must specify a CompletedTask index and set the bool when done

	int step, autostep_index, timeout;
	int	AutoTime, ElapsedTime;
	float distance, prev_distance;
	AutoAction	action;

	struct AutoStep_t {
		AutoAction	action;						// this really isn't needed (I might throw this out to save memory space).
		void		(AUTONOMOUS::*function)();	// this is a pointer to the function needed to execute the command
		int			TaskIndex;					// this is the Index to the associated completion flag
		int			Timeout;					// this will be initialized at the start of the task and count down with each iteration
		union  {
			struct	{
					} NoOp;
			struct	{
					} Arrive;
			struct	{
					int		SpeedControl;
					int		SpinControl;
					int		Orientation;
					}SetControls;
			struct	{
					} Wait4;
			struct	{
					int		Xcoordinate;		// expressed in inches from a known origin point on the field
					int		Ycoordinate;
					int		Power;				// expressed as a % of power to the motor controller
					} MoveTo;
			struct	{
					int		Xdistance;			// number of inches to move from the current position along the X axis
					int		Ydistance;			// number of inches to move from the current position along the Y axis
					int		Power;
					} MoveRelative;
			struct	{
					int		Heading;			// compass bearing in degrees
					int		Power;
					} TurnTo;
			struct	{
					int		Degrees;			// angular change in degrees
					int		Power;
					} TurnRelative;
			struct	{
					int		LeftPower;			// percent power to motor controller
					int		RightPower;
					} ToteWheels;
			struct	{
					int		WheelPosition;		// this is a position based on the number of solenoid valve opening cycles
					int		DutyCycle;			// this is the percent of cycles that the solenoid valve is open
					} ToteArms;
			struct	{
					int		Height;				// height of lift arms in inches above the ground
					int		JoyMode;			// deprecated (set to 0)
					int		Power;				// deprecated (set to 0)
					} ToteLift;
/*
 * The rest of this structure is deprecated, since we abandoned the claw/pincer subsystem
			struct	{
					int		Height;				// height of pincers in inches above the ground
					int		Reach;				// distance of pincers in inches out in front of the robot
					int		Power;
					} ClawArm;
			struct	{
					int		Position;			// this will be a direct measure of the analog value from the Pot * 1000
					int		Power;
					} Pincer;
*/
			} parameters;
		};


	struct Scenario_t
		{
		AutoScenario	ScenarioName;
//		string			Description;		// deprecated
		int				NumberOfSteps;
		int				Starting_X_Position;
		int				Starting_Y_Position;
		int				Starting_Heading;
		AutoStep_t		Steps[AUTONOMOUS_MAX_STEPS];
		} *pScenario;


	int SelectedScenario;

	void ExecNoOp();
	void ExecArrive();
	void ExecSetControls();
	void ExecWait4();
	void MoveToPosition();
	void ExecMoveTo();
	void ExecMoveRelative();
	void TurnToHeading();
	void ExecTurnTo();
	void ExecTurnRelative();
	void ExecToteWheels();
	void ExecToteArms();
	void ExecToteLift();
//	void ExecClawArm();
//	void ExecPincer();

	/* all methods above (except the Wait4 method) execute and move on immediately to the next task
	 * If there are dependencies, they are satisfied by issuing a Wait4 command
	 * On each iteration, the pMyRobotState buffer is checked to see if any actuator/motor movements are needed in order
	 * achieve desired targets.  When targets are achieved, the actuators/motors are turned off and the completed boolean is set.
	 * Actuator/motor movements are initiated by updating the INPUT buffer values.
	 *
	 * Note:	Completed_Tasks[0] is used for non-synchronized operations.
	 * 			To wait for an elapsed time, execute the NoOp with a specified timeout value, followed by a Wait4.
	 * 			To wait for a game time, at the start of the game, execute the NoOp with the timeout set to the desired game time.
	 */

};

#endif /* O2C2015FRC_SRC_AUTONOMOUS_H_ */
