/*
 * AutoPilot.h
 *
 *  Created on: Jan 31, 2015
 *      Author: Rod
 */

#ifndef AUTOPILOT_H
#define AUTOPILOT_H

#include "Definitions.h"
#include "DriveTrain.h"

class AUTOPILOT : public DRIVETRAIN
{

//	friend class DRIVETRAIN;

public:
	AUTOPILOT();
	void Init(RobotStateBuffer *pRobotState, RobotStateBuffer *pTargetState, INPUT *pInput);
	void SelectScenario();			// Executed on every Disabled Loop to check for an update to the Selected Scenario on the SmartDashboard
	void Start();					// Execute the NoOp with timeout = 15 sec and completed -> Completed_Task[1]
	void RunAuto();					// Execute all autonomous tasks in the defined scenario
	void Stop();					// If we enter TeleOp before the autonomous tasks complete, stop all autonomous actions

private:

	#define POSITION_COUNT	5
	#define GOAL_COUNT		5
	#define DEFENSE_COUNT	10
	SendableChooser *pStartChooser, *pGoalChooser, *pDefenseChooser[POSITION_COUNT];
	std::string DefenseName[DEFENSE_COUNT] = {"LowBar","A:Portcullis","A:ChevalDeFrise(Left)","A:ChevalDeFrise(Right)","B:Moat","B:Ramparts","C:Drawbridge","C:SallyPort","D:RockWall","D:RoughTerrain"};
	std::string StartPosition[POSITION_COUNT] = {"1","2","3","4","5"};
	std::string Goal[GOAL_COUNT] = {"LeftLower", "LeftUpper","Middle","RightHigh","RightLower"};
	std::string StartDefault = "3";
	std::string GoalDefault = "Middle";
	std::string DefenseDefault[POSITION_COUNT] = {"LowBar","A:Portcullis","B:Ramparts","C:SallyPort","D:RockWall"};
	std::string DefenseSelected[POSITION_COUNT], StartPostionSelected, GoalSelected;

	typedef enum {	OuterWorks1 =		0,	// specifies the starting position for defense 1
					OuterWorks2 =		1,	// specifies the starting position for defense 2
					OuterWorks3 =		2,	// specifies the starting position for defense 3
					OuterWorks4 =		3,	// specifies the starting position for defense 4
					OuterWorks5 =		4,	// specifies the starting position for defense 5
		} StartPositions;

	typedef enum {	LeftLower =			0,	// score ball in the high left goal
					LeftUpper =			1,	// score ball in the lower left goal
					Middle =			2,	// score ball in the high middle goal
					RightHigh =			3,	// score ball in the high right goal
					RightLower =		4,	// score ball in the lower right goal
		} Goals;

	typedef enum {	LowBar =			0,	// cross the Low Bar defense
					Portcullis =		1,	// cross the Portcullis defense
					ChevalDeFriseL =	2,	// cross the Cheval De Frise defense (Left side)
					ChevalDeFriseR =	3,	// cross the Cheval De Frise defense (Right side)
					Moat =				4,	// cross the Moat defense
					Ramparts =			5,	// cross the Ramparts defense
					DrawBridge =		6,	// cross the Draw Bridge defense
					SallyPort =			7,	// cross the Sally Port Door defense
					RockWall =			8,	// cross the Rock Wall defense
					RoughTerrain =		9,	// cross the Rough Terrain defense
					Demo =				10
		} Defenses;

		/*
		 * ToDo:
		 * 		Add a Label command for branch control
		 * 		Add a Timeout branch parameter to the Wait4 command
		 * 		Add controls to turn on/off spin control, set speed control, and change orientation mode
		 * 		Add controls to query sensors that are not integral components of controlled subsystems
		 * 			for example, an ultrasonic proximity sensor, or a photo-electric sensor.
		 * 		Add control to reset the Robot's Coordinates to the AutoTarget coordinates (Arrive)
		 */
	typedef enum {	SetControls = 0,
					CountedLoop = 1,
					Branch = 2,
					Arrive = 3,
					Wait4 = 4,
					MoveTo = 5,
					MoveRelative = 6,
					TurnTo = 7,
					TurnRelative = 8,
					Shooter = 9,
					Scooper = 10,
					Climber = 11
		} AutoAction;

	typedef enum {	Xcoord=0,
	 				Ycoord=1,
					Heading=2,
					Roll=3,
					Pitch=4,
					LeftPhoto=5,
					RightPhoto=6,
					light=7,
					Range=8,
					ShooterAngle=9,
					ClimberAngle=10,
					ClimberReach=11
		} Conditionals;

	typedef enum {	UN=0,
					LT=1,
					LE=2,
					EQ=3,
					GE=4,
					GT=5
		} Comparisons;

	/*
	 * The following construct lists all the pre-programmed steps for every scenario.
	 * When the SmartDashboard is updated during Disabled operations, the new scenario steps will be loaded into the
	 * pScenario->Steps array from the array below.
	 */

	typedef int		AutoCommand_t[7];
	/*
	 * Header records
	 * 		Element		Purpose
	 * 			0		-1				-1 signifies a header record
	 * 			1		name			an arbitrary label that identifies the program scenario
	 * 			2		Starting X		X coordinate of the starting position
	 * 			3		Starting Y		Y coordinate of the starting position
	 * 			4		Start Heading	Robot Heading at the starting position
	 * 			5		not used
	 * 			6		not used
	 *
	 *
	 * Program Steps
	 * 		Element		Purpose
	 * 			0		Line Number		Use arbitrary line numbers, but should be in numerical order and help you count steps for proper array size
	 * 			1		Command			The command to execute at this step
	 * 			2		Timeout			The maximum time to allow the command to complete
	 * 			3		Parameter 1		Each Command requires various parameters
	 * 			4		Parameter 2		Each Command requires various parameters
	 * 			5		Parameter 3		Each Command requires various parameters
	 * 			6		Parameter 4		Each Command requires various parameters
	 */

	AutoCommand_t		StartSteps[5] = {				// There are no steps, only starting positions and headings
			{-1, OuterWorks1, -135, 27, 000, 0, 0},			// Start facing the Low Bar defense (Defense1)
			{-1, OuterWorks2,  -82, 27, 000, 0, 0},			// Start facing the Defense2 defense
			{-1, OuterWorks3,  -29, 27, 000, 0, 0},			// Start facing the Defense3 defense
			{-1, OuterWorks4,   24, 27, 000, 0, 0},			// Start facing the Defense4 defense
			{-1, OuterWorks5,   77, 27, 000, 0, 0},			// Start facing the Defense5 defense
		};

	AutoCommand_t		DefenseSteps[236] = {
		{-1, LowBar, 0, 0, 000, 0, 0},						// Cross the Low Bar defense
			{10, SetControls, 100, Speedy, true, 0, 0},			//	Set for Normal speed with spin control enabled
			{20, Wait4, 500, 20, 0, 0, 0},						//	Wait 0.5 sec for scooper encoder reset
			{30, Shooter, 500, 0, BOULDER_HOLD, 0, 0},			//	Set shooter to Hold the Boulder
			{40, Scooper, 1500, 20, -50, 0, 0},					//	lower the shooting angle to 15 degrees to pass over the ramp and under the low bar
			{50, Wait4, 500, 40, 0, 0, 0},						//	wait 0.5 sec for scooper reposition before moving forward
			{60, MoveRelative, 2000, 0, 55, 75, 0},				//	Move Forward 55 inches at 75% power to the front edge of the ramp
			{70, Wait4, 2000, 60, 0, 0, 0},						//	wait 2.0 sec to get to the leading edge of the ramp
			{80, MoveRelative, 100, 0, 4, 50, 0},				//	Continue moving forward 4 inches at a time
			{90, Wait4, 100, 80, 0, 0, 0},						//	wait 0.5 sec to start climbing the ramp
			{100, Branch, 500, Pitch, GE, 35, 120},				//	until we start climbing up the ramp (pitch >= 2 degree) (2 deg * DEGREES_TO_RADIANS * 1000 = 35)
			{110, CountedLoop, 0, 1, 1, 5, 80},					//	or until we have moved forward an additional 24 inches (whichever occurs first)
			{120, MoveRelative, 1500, 0, 24, 75, 0},			//	Now move forward 24 inches to the start of the downward incline
			{130, Wait4, 1500, 120, 0, 0, 0},					//	wait 2.0 sec to get to the leading edge of the downward ramp
			{140, MoveRelative, 100, 0, 4, 50, 0},				//	Continue moving forward 4 inches at a time
			{150, Wait4, 100, 140, 0, 0, 0},					//	wait 0.5 sec to start descending the ramp
			{160, Branch, 500, Pitch, LE, -35, 180},			//	until we start descending the ramp (pitch <= -2 degree) (2 deg * DEGREES_TO_RADIANS * 1000 = 35)
			{170, CountedLoop, 0, 1, 1, 5, 140},				//	or until we have moved forward an additional 24 inches (whichever occurs first)
			{180, MoveRelative, 1000, 0, 12, 50, 0},			//	Now move forward 12 inches to the bottom of the downward incline
			{190, Wait4, 1000, 180, 0, 0, 0},					//	wait 1.0 sec to get down the ramp
			{200, MoveRelative, 100, 0, 4, 50, 0},				//	Continue moving forward 4 inches at a time
			{210, Wait4, 100, 200, 0, 0, 0},					//	wait 0.5 sec to finish descending the ramp
			{220, Branch, 500, Pitch, GE, -8, 240},				//	until we are back on flat ground (pitch >= -0.5 degree)
			{230, CountedLoop, 0, 1, 1, 3, 200},				//	or until we have moved forward an additional 16 inches (whichever occurs first)
			{240, Arrive, 100, 1, 0, 122, 0},					//	Reset Robot Coordinates to the Courtyard side of the defense 122 inches up field

		{-1, Portcullis, 0, 0, 000, 0, 0},					//	Cross the Portcullis defense
			{1010, SetControls, 100, Speedy, true, 0, 0},		//	Set for Normal speed with spin control enabled
			{1020, Wait4, 500, 1020, 0, 0, 0},					//	Wait 0.5 sec for scooper encoder reset
			{1030, Shooter, 500, 0, BOULDER_HOLD, 0, 0},		//	Set shooter to Hold the Boulder
			{1040, Scooper, 1500, 20, -50, 0, 0},				//	lower the shooting angle to 5 degrees to start lifting the bot as the shooter hits the ramp and yet pass under the portcullis
			{1050, Wait4, 500, 1040, 0, 0, 0},					//	wait 0.5 sec for scooper reposition before moving forward
			{1060, MoveRelative, 2000, 0, 55, 100, 0},			//	Move Forward 55 inches at 100% power to the front edge of the ramp (approach with caution to not ram the portcullis)
			{1070, Wait4, 2000, 1060, 0, 0, 0},					//	wait 2.0 sec to get to the leading edge of the ramp
			{1080, MoveRelative, 500, 0, 4, 50, 0},				//	Continue moving forward 4 inches at a time
			{1090, Wait4, 500, 1080, 0, 0, 0},					//	wait 0.5 sec to start climbing the ramp
			{1100, Branch, 500, Pitch, GE, 35, 1120},			//	until we start climbing up the ramp (pitch >= 2 degree) (2 deg * DEGREES_TO_RADIANS * 1000 = 35)
			{1110, CountedLoop, 0, 0, 1, 2, 1080},				//	or until we have moved forward an additional 12 inches (whichever occurs first)
			{1120, Scooper, 1500, 25, 50, 0, 0},				//	raise the scooper to 25 degrees to start lifting the portcullis
			{1130, MoveRelative, 500, 0, 6, 75, 0},				//	Continue moving forward 6 inches at a time while raising the portcullis
			{1140, Wait4, 500, 1130, 0, 0, 0},					//	wait 0.5 sec for scooper reposition before moving forward
			{1150, Scooper, 1500, 40, 50, 0, 0},				//	raise the scooper to 40 degrees to continue lifting the portcullis
			{1160, MoveRelative, 500, 0, 6, 75, 0},				//	Continue moving forward 6 inches at a time while raising the portcullis
			{1170, Wait4, 500, 1160, 0, 0, 0},					//	wait 0.5 sec for scooper reposition before moving forward
			{1180, Scooper, 1500, 60, 50, 0, 0},				//	raise the scooper to 60 degrees to continue lifting the portcullis
			{1190, MoveRelative, 500, 0, 6, 75, 0},				//	Continue moving forward 6 inches at a time while raising the portcullis
			{1200, Wait4, 500, 1190, 0, 0, 0},					//	wait 0.5 sec for scooper reposition before moving forward
			{1210, Scooper, 1500, SCOOPER_FULL_ELEVATION, 50, 0, 0},//	raise the scooper all the way up
			{1220, MoveRelative, 2000, 0, 18, 100, 0},			//	Now move forward 24 inches to the start of the downward incline
			{1230, Wait4, 2000, 1220, 0, 0, 0},					//	wait 2.0 sec to get to the edge of the downward incline
			{1240, MoveRelative, 500, 0, 4, 50, 0},				//	Continue moving forward 4 inches at a time
			{1250, Wait4, 500, 1240, 0, 0, 0},					//	wait 0.5 sec
			{1260, Branch, 500, Pitch, LE, -35, 1280},			//	until we start descending the ramp (pitch <= -2 degree) (2 deg * DEGREES_TO_RADIANS * 1000 = 35)
			{1270, CountedLoop, 0, 0, 1, 5, 1240},				//	or until we have moved forward an additional 24 inches (whichever occurs first)
			{1280, MoveRelative, 1000, 0, 12, 50, 0},			//	Now move forward 12 inches to the bottom of the downward incline
			{1290, Wait4, 1000, 1280, 0, 0, 0},					//	wait 1.0 sec to reach the bottom
			{1300, MoveRelative, 500, 0, 4, 50, 0},				//	Continue moving forward 4 inches at a time
			{1310, Wait4, 500, 1300, 0, 0, 0},					//	wait 0.5 sec
			{1320, Branch, 500, Pitch, GE, -8, 1340},			//	until we are back on flat ground (pitch >= -0.5 degree)
			{1330, CountedLoop, 0, 0, 1, 3, 1300},				//	or until we have moved forward an additional 16 inches (whichever occurs first)
			{1340, Arrive, 0, 1, 0, 122, 0},					//	Reset Robot Coordinates to the Courtyard side of the defense 122 inches up field

		{-1, ChevalDeFriseL, 0, 0, 000, 0, 0},				//	Cross the ChevalDeFrise (Left side) defense
			{2010, SetControls, 100, Speedy, true, 0, 0},		//	Set for Normal speed with spin control enabled
			{2020, Wait4, 500, 2020, 0, 0, 0},					//	Wait 0.5 sec for scooper encoder reset
			{2030, Shooter, 500, 0, BOULDER_HOLD, 0, 0},		//	Set shooter to Hold the Boulder
			{2040, Scooper, 1500, 90, -50, 0, 0},				//	lower the shooting angle to 20 degrees to pass over the counter tilted platforms
			{2050, Wait4, 500, 2040, 0, 0, 0},					//	wait 0.5 sec for scooper reposition before moving forward
			{2060, MoveRelative, 2000, 0, 51, 100, 0},			//	Move Forward 51 inches at 100% power to the front edge of the ramp
			{2070, Wait4, 2000, 2060, 0, 0, 0},					//	wait 2.0 sec to get to the leading edge of the ramp
			{2080, Scooper, 500, 20, -50, 0, 0},				//	lower the scooper to 10 degrees to pull down the counter tilted platforms
			{2090, MoveRelative, 500, 0, 4, 50, 0},				//	Continue moving forward 4 inches at a time
			{2100, Wait4, 500, 2090, 0, 0, 0},					//	wait 0.5 sec to start climbing the ramp
			{2110, Branch, 500, Pitch, GE, 35, 2130},			//	until we start climbing up the ramp (pitch >= 2 degree) (2 deg * DEGREES_TO_RADIANS * 1000 = 35)
			{2120, CountedLoop, 0, 0, 1, 2, 2090},				//	or until we have moved forward an additional 12 inches (whichever occurs first)
			{2130, MoveRelative, 2000, 0, 24, 100, 0},			//	Now gun it forward 24 inches to the top and tip forward
			{2140, Wait4, 1000, 2130, 0, 0, 0},					//	wait 1.0 sec to get over the hump
			{2150, MoveRelative, 500, 0, 4, 100, 0},			//	Continue to gun it forward 4 inches at a time
			{2160, Branch, 500, Pitch, LE, -35, 2180},			//	until we pitch down (pitch <= -2 degree) (2 deg * DEGREES_TO_RADIANS * 1000 = 35)
			{2170, CountedLoop, 0, 0, 1, 5, 2150},				//	or until we have moved forward an additional 24 inches (whichever occurs first)
			{2180, MoveRelative, 500, 0, 4, 25, 0},				//	Continue moving forward 4 inches at a time
			{2190, Wait4, 500, 2180, 0, 0, 0},					//	wait 0.5 sec to finish descending the ramp
			{2200, Branch, 500, Pitch, GE, -8, 2220},			//	until we are back on flat ground (pitch >= -0.5 degree)
			{2210, CountedLoop, 0, 0, 1, 3, 2180},				//	or until we have moved forward an additional 16 inches (whichever occurs first)
			{2220, Arrive, 0, 1, -6, 122, 0},					//	Reset Robot Coordinates to the Courtyard side of the defense 122 inches up field and 6 inches left

		{-1, ChevalDeFriseR, 0, 0, 000, 0, 0},				//	Cross the ChevalDeFrise (Right side) defense
			{3010, SetControls, 100, Speedy, true, 0, 0},		//	Set for Normal speed with spin control enabled
			{3020, Wait4, 500, 3020, 0, 0, 0},					//	Wait 0.5 sec for scooper encoder reset
			{3030, Shooter, 500, 0, BOULDER_HOLD, 0, 0},		//	Set shooter to Hold the Boulder
			{3040, Scooper, 1500, 90, -50, 0, 0},				//	lower the shooting angle to 20 degrees to pass over the counter tilted platforms
			{3050, Wait4, 500, 3040, 0, 0, 0},					//	wait 0.5 sec for scooper reposition before moving forward
			{3060, MoveRelative, 2000, 0, 51, 100, 0},			//	Move Forward 51 inches at 100% power to the front edge of the ramp
			{3070, Wait4, 2000, 3060, 0, 0, 0},					//	wait 2.0 sec to get to the leading edge of the ramp
			{3080, Scooper, 500, 20, -50, 0, 0},				//	lower the scooper to 10 degrees to pull down the counter tilted platforms
			{3090, MoveRelative, 500, 0, 4, 50, 0},				//	Continue moving forward 4 inches at a time
			{3100, Wait4, 500, 3090, 0, 0, 0},					//	wait 0.5 sec to start climbing the ramp
			{3110, Branch, 500, Pitch, GE, 35, 3130},			//	until we start climbing up the ramp (pitch >= 2 degree) (2 deg * DEGREES_TO_RADIANS * 1000 = 35)
			{3120, CountedLoop, 0, 0, 1, 2, 3090},				//	or until we have moved forward an additional 12 inches (whichever occurs first)
			{3130, MoveRelative, 2000, 0, 24, 100, 0},			//	Now gun it forward 24 inches to the top and tip forward
			{3140, Wait4, 1000, 3130, 0, 0, 0},					//	wait 1.0 sec to get over the hump
			{3150, MoveRelative, 500, 0, 4, 100, 0},			//	Continue to gun it forward 4 inches at a time
			{3160, Branch, 500, Pitch, LE, -35, 3180},			//	until we pitch down (pitch <= -2 degree) (2 deg * DEGREES_TO_RADIANS * 1000 = 35)
			{3170, CountedLoop, 0, 0, 1, 5, 3150},				//	or until we have moved forward an additional 24 inches (whichever occurs first)
			{3180, MoveRelative, 500, 0, 4, 25, 0},				//	Continue moving forward 4 inches at a time
			{3190, Wait4, 500, 3180, 0, 0, 0},					//	wait 0.5 sec to finish descending the ramp
			{3200, Branch, 500, Pitch, GE, -8, 3220},			//	until we are back on flat ground (pitch >= -0.5 degree)
			{3210, CountedLoop, 0, 0, 1, 3, 3180},				//	or until we have moved forward an additional 16 inches (whichever occurs first)
			{3220, Arrive, 0, 1, 6, 122, 0},					//	Reset Robot Coordinates to the Courtyard side of the defense 122 inches up field and 6 inches right

		{-1, Moat, 0, 0, 000, 0, 0},						//	Cross the Moat defense
			{4010, SetControls, 100, Speedy, true, 0, 0},		//	Set for Normal speed with spin control enabled
			{4020, Wait4, 500, 4020, 0, 0, 0},					//	Wait 0.5 sec for scooper encoder reset
			{4030, Shooter, 500, 0, BOULDER_HOLD, 0, 0},		//	Set shooter to Hold the Boulder
			{4040, Scooper, 1500, 90, -50, 0, 0},				//	lower the shooting angle to 15 degrees to pass over the moat wall
			{4050, Wait4, 500, 4040, 0, 0, 0},					//	wait 0.5 sec for scooper reposition before moving forward
			{4060, MoveRelative, 2000, 0, 63, 100, 0},			//	Move Forward 63 inches at 100% power to start climbing up the ramp
			{4070, Wait4, 2000, 4060, 0, 0, 0},					//	wait 2.0 sec to get to the leading edge of the ramp
			{4080, MoveRelative, 500, 0, 4, 50, 0},				//	Continue moving forward 4 inches at a time
			{4090, Wait4, 500, 4080, 0, 0, 0},					//	wait 0.5 sec to start climbing the ramp
			{4100, Branch, 500, Pitch, GE, 35, 4120},			//	until we start climbing up the ramp (pitch >= 2 degree) (2 deg * DEGREES_TO_RADIANS * 1000 = 35)
			{4110, CountedLoop, 0, 0, 1, 5, 4080},				//	or until we have moved forward an additional 24 inches (whichever occurs first)
			{4120, MoveRelative, 500, 0, 4, 50, 0},				//	Continue moving forward 4 inches at a time
			{4130, Wait4, 500, 4120, 0, 0, 0},					//	wait 0.5 sec to start climbing the ramp
			{4140, Branch, 500, Pitch, LE, -35, 4160},			//	until we start descending the first moat wall (pitch <= -2 degree) (2 deg * DEGREES_TO_RADIANS * 1000 = 35))
			{4150, CountedLoop, 0, 0, 1, 5, 4120},				//	or until we have moved forward an additional 24 inches (whichever occurs first)
			{4160, MoveRelative, 500, 0, 4, 50, 0},				//	Continue moving forward 4 inches at a time
			{4170, Wait4, 500, 4160, 0, 0, 0},					//	wait 0.5 sec to start climbing the ramp
			{4180, Branch, 500, Pitch, GE, 35, 4200},			//	until we start climbing up the second moat wall (pitch >= 2 degree) (2 deg * DEGREES_TO_RADIANS * 1000 = 35)
			{4190, CountedLoop, 0, 0, 1, 5, 4160},				//	or until we have moved forward an additional 24 inches (whichever occurs first)
			{4200, MoveRelative, 500, 0, 4, 50, 0},				//	Continue moving forward 4 inches at a time
			{4210, Wait4, 500, 4200, 0, 0, 0},					//	wait 0.5 sec to start climbing the ramp
			{4220, Branch, 500, Pitch, LE, -35, 4240},			//	until we start descending the second moat wall (pitch <= -2 degree) (2 deg * DEGREES_TO_RADIANS * 1000 = 35))
			{4230, CountedLoop, 0, 0, 1, 5, 4200},				//	or until we have moved forward an additional 24 inches (whichever occurs first)
			{4240, MoveRelative, 1000, 0, 12, 50, 0},			//	Now move forward 12 inches to the bottom of the downward incline
			{4250, Wait4, 1000, 4240, 0, 0, 0},					//	wait 1.0 sec to get down the ramp
			{4260, MoveRelative, 500, 0, 4, 50, 0},				//	Continue moving forward 4 inches at a time
			{4270, Wait4, 500, 4260, 0, 0, 0},					//	wait 0.5 sec to finish descending the ramp
			{4280, Branch, 500, Pitch, GE, -8, 4300},			//	until we are back on flat ground (pitch >= -0.5 degree)
			{4290, CountedLoop, 0, 0, 1, 5, 4260},				//	or until we have moved forward an additional 24 inches (whichever occurs first)
			{4300, Arrive, 0, 1, 0, 122, 0},					//	Reset Robot Coordinates to the Courtyard side of the defense 122 inches up field

		{-1, Ramparts, 0, 0, 000, 0, 0},					//	Cross the Ramparts defense
			{5010, SetControls, 100, Speedy, true, 0, 0},		//	Set for Normal speed with spin control enabled
			{5020, Wait4, 500, 5020, 0, 0, 0},					//	Wait 0.5 sec for scooper encoder reset
			{5030, Shooter, 500, 0, BOULDER_HOLD, 0, 0},		//	Set shooter to Hold the Boulder
			{5040, Scooper, 1500, 90, -50, 0, 0},				//	lower the shooting angle to 20 degrees to pass over the ramparts
			{5050, Wait4, 500, 5040, 0, 0, 0},					//	wait 0.5 sec for scooper reposition before moving forward
			{5060, MoveRelative, 3000, 0, 95, 100, 0},			//	Move Forward 95 inches at 100% power to the leading edge of the down ramp
			{5070, Wait4, 3000, 5060, 0, 0, 0},					//	wait 3.0 sec to start descending the ramp
			{5080, MoveRelative, 500, 0, 4, 50, 0},				//	Continue moving forward 4 inches at a time
			{5085, TurnRelative, 500, 5, 50, 0, 0},				//  Ramparts cause robot to veer left, so turn right as you pass over
			{5090, Wait4, 500, 5080, 0, 0, 0},					//	wait 0.5 sec to start descending the ramp
			{5100, Branch, 500, Pitch, LE, -35, 5120},			//	until we start descending the ramp (pitch <= -2 degree) (2 deg * DEGREES_TO_RADIANS * 1000 = 35)
			{5110, CountedLoop, 0, 0, 1, 5, 5080},				//	or until we have moved forward an additional 24 inches (whichever occurs first)
			{5120, MoveRelative, 1000, 0, 12, 50, 0},			//	Now move forward 12 inches to the bottom of the downward incline
			{5130, Wait4, 1000, 5120, 0, 0, 0},					//	wait 1.0 sec to get down the ramp
			{5140, MoveRelative, 500, 0, 4, 50, 0},				//	Continue moving forward 4 inches at a time
			{5150, Wait4, 500, 5140, 0, 0, 0},					//	wait 0.5 sec to finish descending the ramp
			{5160, Branch, 500, Pitch, GE, -8, 5180},			//	until we are back on flat ground (pitch >= -0.5 degree)
			{5170, CountedLoop, 0, 0, 1, 3, 5140},				//	or until we have moved forward an additional 16 inches (whichever occurs first)
			{5180, Arrive, 0, 1, 0, 122, 0},					//	Reset Robot Coordinates to the Courtyard side of the defense 122 inches up field

		{-1, DrawBridge, 0, 0, 000, 0, 0},					//	Cross the DrawBridge defense
			{6010, SetControls, 100, Speedy, true, 0, 0},		//	Set for Normal speed with spin control enabled
			{6020, Wait4, 500, 6020, 0, 0, 0},					//	Wait 0.5 sec for scooper encoder reset
			{6030, Shooter, 500, 0, BOULDER_HOLD, 0, 0},		//	Set shooter to Hold the Boulder
			{6040, Scooper, 1500, 90, -50, 0, 0},				//	lower the shooting angle to 20 degrees to pass over the ramparts
			{6050, Wait4, 500, 6040, 0, 0, 0},					//	wait 0.5 sec for scooper reposition before moving forward
			{6060, MoveRelative, 3000, 0, 95, 100, 0},			//	Move Forward 95 inches at 100% power to the leading edge of the down ramp
			{6070, Wait4, 3000, 6060, 0, 0, 0},					//	wait 3.0 sec to start descending the ramp
			{6080, MoveRelative, 500, 0, 4, 50, 0},				//	Continue moving forward 4 inches at a time
			{6090, Wait4, 500, 6080, 0, 0, 0},					//	wait 0.5 sec to start descending the ramp
			{6100, Branch, 500, Pitch, LE, -35, 6120},			//	until we start descending the ramp (pitch <= -2 degree) (2 deg * DEGREES_TO_RADIANS * 1000 = 35)
			{6110, CountedLoop, 0, 0, 1, 5, 6080},				//	or until we have moved forward an additional 24 inches (whichever occurs first)
			{6120, MoveRelative, 1000, 0, 12, 50, 0},			//	Now move forward 12 inches to the bottom of the downward incline
			{6130, Wait4, 1000, 6120, 0, 0, 0},					//	wait 1.0 sec to get down the ramp
			{6140, MoveRelative, 500, 0, 4, 50, 0},				//	Continue moving forward 4 inches at a time
			{6150, Wait4, 500, 6140, 0, 0, 0},					//	wait 0.5 sec to finish descending the ramp
			{6160, Branch, 500, Pitch, GE, -8, 6180},			//	until we are back on flat ground (pitch >= -0.5 degree)
			{6170, CountedLoop, 0, 0, 1, 3, 6140},				//	or until we have moved forward an additional 16 inches (whichever occurs first)
			{6180, Arrive, 0, 1, 0, 122, 0},					//	Reset Robot Coordinates to the Courtyard side of the defense 122 inches up field

		{-1, SallyPort, 0, 0, 000, 0, 0},					//	Cross the SallyPort defense
			{7010, SetControls, 100, Speedy, true, 0, 0},		//	Set for Normal speed with spin control enabled
			{7020, Wait4, 500, 7020, 0, 0, 0},					//	Wait 0.5 sec for scooper encoder reset
			{7030, Shooter, 500, 0, BOULDER_HOLD, 0, 0},		//	Set shooter to Hold the Boulder
			{7040, Scooper, 1500, 90, -50, 0, 0},				//	lower the shooting angle to 20 degrees to pass over the ramparts
			{7050, Wait4, 500, 7040, 0, 0, 0},					//	wait 0.5 sec for scooper reposition before moving forward
			{7060, MoveRelative, 3000, 0, 95, 100, 0},			//	Move Forward 95 inches at 100% power to the leading edge of the down ramp
			{7070, Wait4, 3000, 7060, 0, 0, 0},					//	wait 3.0 sec to start descending the ramp
			{7080, MoveRelative, 500, 0, 4, 50, 0},				//	Continue moving forward 4 inches at a time
			{7090, Wait4, 500, 7080, 0, 0, 0},					//	wait 0.5 sec to start descending the ramp
			{7100, Branch, 500, Pitch, LE, -35, 7120},			//	until we start descending the ramp (pitch <= -2 degree) (2 deg * DEGREES_TO_RADIANS * 1000 = 35)
			{7110, CountedLoop, 0, 0, 1, 5, 7080},				//	or until we have moved forward an additional 24 inches (whichever occurs first)
			{7120, MoveRelative, 1000, 0, 12, 50, 0},			//	Now move forward 12 inches to the bottom of the downward incline
			{7130, Wait4, 1000, 7120, 0, 0, 0},					//	wait 1.0 sec to get down the ramp
			{7140, MoveRelative, 500, 0, 4, 50, 0},				//	Continue moving forward 4 inches at a time
			{7150, Wait4, 500, 7140, 0, 0, 0},					//	wait 0.5 sec to finish descending the ramp
			{7160, Branch, 500, Pitch, GE, -8, 7180},			//	until we are back on flat ground (pitch >= -0.5 degree)
			{7170, CountedLoop, 0, 0, 1, 3, 7140},				//	or until we have moved forward an additional 16 inches (whichever occurs first)
			{7180, Arrive, 0, 1, 0, 122, 0},					//	Reset Robot Coordinates to the Courtyard side of the defense 122 inches up field

		{-1, RockWall, 0, 0, 000, 0, 0},					//	Cross the RockWall defense
			{8010, SetControls, 100, Speedy, true, 0, 0},		//	Set for Normal speed with spin control enabled
			{8020, Wait4, 500, 8020, 0, 0, 0},					//	Wait 0.5 sec for scooper encoder reset
			{8030, Shooter, 500, 0, BOULDER_HOLD, 0, 0},		//	Set shooter to Hold the Boulder
			{8040, Scooper, 1500, 90, -50, 0, 0},				//	lower the shooting angle to 20 degrees to pass over the ramparts
			{8050, Wait4, 500, 8040, 0, 0, 0},					//	wait 0.5 sec for scooper reposition before moving forward
			{8060, MoveRelative, 3000, 0, 95, 100, 0},			//	Move Forward 95 inches at 100% power to the leading edge of the down ramp
			{8070, Wait4, 3000, 8060, 0, 0, 0},					//	wait 3.0 sec to start descending the ramp
			{8080, MoveRelative, 500, 0, 4, 50, 0},				//	Continue moving forward 4 inches at a time
			{8085, Scooper, 500, 40, -10, 0, 0},
			{8090, Wait4, 500, 8080, 0, 0, 0},					//	wait 0.5 sec to start descending the ramp
			{8100, Branch, 500, Pitch, LE, -35, 8120},			//	until we start descending the ramp (pitch <= -2 degree) (2 deg * DEGREES_TO_RADIANS * 1000 = 35)
			{8110, CountedLoop, 0, 0, 1, 5, 8080},				//	or until we have moved forward an additional 24 inches (whichever occurs first)
			{8120, MoveRelative, 1000, 0, 12, 50, 0},			//	Now move forward 12 inches to the bottom of the downward incline
			{8130, Wait4, 1000, 8120, 0, 0, 0},					//	wait 1.0 sec to get down the ramp
			{8140, MoveRelative, 500, 0, 4, 50, 0},				//	Continue moving forward 4 inches at a time
			{8150, Wait4, 500, 8140, 0, 0, 0},					//	wait 0.5 sec to finish descending the ramp
			{8160, Branch, 500, Pitch, GE, -8, 8180},			//	until we are back on flat ground (pitch >= -0.5 degree)
			{8170, CountedLoop, 0, 0, 1, 3, 8140},				//	or until we have moved forward an additional 16 inches (whichever occurs first)
			{8180, Arrive, 0, 1, 0, 122, 0},					//	Reset Robot Coordinates to the Courtyard side of the defense 122 inches up field

		{-1, RoughTerrain, 0, 0, 000, 0, 0},				//	Cross the RoughTerrain defense
			{9010, SetControls, 100, Speedy, true, 0, 0},		//	Set for Normal speed with spin control enabled
			{9020, Wait4, 500, 9020, 0, 0, 0},					//	Wait 0.5 sec for scooper encoder reset
			{9030, Shooter, 500, 0, BOULDER_HOLD, 0, 0},		//	Set shooter to Hold the Boulder
			{9040, Scooper, 1500, 90, -50, 0, 0},				//	lower the shooting angle to 20 degrees to pass over the ramparts
			{9050, Wait4, 500, 9040, 0, 0, 0},					//	wait 0.5 sec for scooper reposition before moving forward
			{9060, MoveRelative, 3000, 0, 95, 100, 0},			//	Move Forward 95 inches at 100% power to the leading edge of the down ramp
			{9070, Wait4, 3000, 9060, 0, 0, 0},					//	wait 3.0 sec to start descending the ramp
			{9080, MoveRelative, 500, 0, 4, 50, 0},				//	Continue moving forward 4 inches at a time
			{9090, Wait4, 500, 9080, 0, 0, 0},					//	wait 0.5 sec to start descending the ramp
			{9100, Branch, 500, Pitch, LE, -35, 9120},			//	until we start descending the ramp (pitch <= -2 degree) (2 deg * DEGREES_TO_RADIANS * 1000 = 35)
			{9110, CountedLoop, 0, 0, 1, 5, 9080},				//	or until we have moved forward an additional 24 inches (whichever occurs first)
			{9120, MoveRelative, 1000, 0, 12, 50, 0},			//	Now move forward 12 inches to the bottom of the downward incline
			{9130, Wait4, 1000, 9120, 0, 0, 0},					//	wait 1.0 sec to get down the ramp
			{9140, MoveRelative, 500, 0, 4, 50, 0},				//	Continue moving forward 4 inches at a time
			{9150, Wait4, 500, 9140, 0, 0, 0},					//	wait 0.5 sec to finish descending the ramp
			{9160, Branch, 500, Pitch, GE, -8, 9180},			//	until we are back on flat ground (pitch >= -0.5 degree)
			{9170, CountedLoop, 0, 0, 1, 3, 9140},				//	or until we have moved forward an additional 16 inches (whichever occurs first)
			{9180, Arrive, 0, 1, 0, 122, 0},					//	Reset Robot Coordinates to the Courtyard side of the defense 122 inches up field

		{-1, 99999, -1, -1, -1, -1, -1},						//	End of Command Steps
	};

	AutoCommand_t		GoalSteps[142] = {
		{-1, LeftLower, 0, 0, 0, 0, 0},						//	Drive to the optimal spot to shoot the LeftLower goal and fire
			{10010, MoveTo, 4000, -92, 278, 80, 0},				//	move to position at 80% power
			{10020, Scooper, 2000, 9, -50, 0, 0},				//	lower the shooting angle to 9 degrees so we can pick up the retro-reflective tape
			{10030, Wait4, 4000, 10010, 0, 0, 0},				//	wait 4 sec to arrive in shooting position
			{10040, TurnTo, 2000, 060, 180, 0, 0},				//	spin to face the target
			{10050, Wait4, 2000, 10040, 0, 0, 0},				//	wait 0.5 sec to search for target

			{10060, Branch, 0, LeftPhoto, GE, 500, 10170},		//	If the left photo sensor is on, branch to CCW search step
			{10070, TurnRelative, 500, 1, 50, 1, 0},			//	While not seeing LeftPhoto, turn CW, 1 degree at a time, for up to 5 degrees
			{10080, Wait4, 500, 10070, 0, 0, 0},				//	wait 0.5 sec to search for target
			{10090, CountedLoop, 0, 0, 1, 5, 10060},			//	Max CW search of 5 degrees

			{10100, TurnRelative, 1000, -6, 50, -1, 0},			//	Failing to find the LeftPhoto hit, look the other way
			{10110, Wait4, 500, 10100, 0, 0, 0},				//	wait 0.5 sec to search for target

			{10120, Branch, 0, LeftPhoto, GE, 500, 10170},		//	If the left photo sensor is on, branch to CCW search step
			{10130, TurnRelative, 500, -1, 50, -1, 0},			//	While not seeing LeftPhoto, turn CCW, 1 degree at a time, for up to 5 degrees
			{10140, Wait4, 500, 10130, 0, 0, 0},				//	wait 0.5 sec to search for target
			{10150, CountedLoop, 0, 0, 1, 5, 10120},			//	Max CCW search of 5 degrees

			{10160, Branch, 0, 0, 0, 0, 10270},					//	Could not locate the target, quit

			{10170, Branch, 0, 	RightPhoto, GE, 500, 10220},		//	Left photo sensor is on, now search for right photo sensor
			{10180, TurnRelative, 500, -1, 50, -1, 0},			//	While not seeing RightPhoto, turn CCW, 1 degree at a time, for up to 15 degrees
			{10190, Wait4, 500, 10180, 0, 0, 0},				//	wait 0.5 sec to search for target
			{10200, CountedLoop, 0, 0, 1, 15, 10170},			//	Max CCW search of 15 degrees

			{10210, Branch, 0, 0, 0, 0, 10270},				//	Could not locate the target, quit

			{10220, Shooter, 2000, SHOOTER_LAUNCH_POWER, BOULDER_HOLD, 0, 0},		//	spin up the launcher to SHOOTER_LAUNCH_POWER watts
			{10230, Scooper, 1000, 14, 50, 0, 0},				//	raise the shooting angle to 14 degrees to hit the target center
			{10240, Wait4, 1000, 10230, 0, 0, 0},				//	wait 1 sec to raise the shooter elevation
			{10250, Wait4, 2000, 10250, 0, 0, 0},				//	wait 2 sec to spin up the shooter to the launch power
			{10260, Shooter, 2000, SHOOTER_LAUNCH_POWER, BOULDER_LAUNCH, 0, 0},	//	Launch the boulder
			{10270, Wait4, 2000, 10260, 0, 0, 0},				//	wait 2 seconds to complete the launch

		{-1, LeftUpper, 0, 0, 0, 0, 0},						//	Drive to the optimal spot to shoot the LeftUpper goal and fire
			{11010, MoveTo, 4000, -109, 268, 80, 0},			//	move to position at 80% power
			{11020, Scooper, 2000, 45, 50, 0, 0},				//	lower the shooting angle to 9 degrees so we can pick up the retro-reflective tape
			{11030, Wait4, 4000, 11010, 0, 0, 0},				//	wait 4 sec to arrive in shooting position
			{11040, TurnTo, 2000, 060, 180, 0, 0},				//	spin to face the target
			{11050, Wait4, 2000, 11040, 0, 0, 0},				//	wait 0.5 sec to search for target

			{11060, Branch, 0, LeftPhoto, GE, 500, 11170},		//	If the left photo sensor is on, branch to CCW search step
			{11070, TurnRelative, 500, 1, 50, 1, 0},			//	While not seeing LeftPhoto, turn CW, 1 degree at a time, for up to 5 degrees
			{11080, Wait4, 500, 11070, 0, 0, 0},				//	wait 0.5 sec to search for target
			{11090, CountedLoop, 0, 0, 1, 5, 11060},			//	Max CW search of 5 degrees

			{11100, TurnRelative, 1000, -6, 50, -1, 0},			//	Failing to find the LeftPhoto hit, look the other way
			{11110, Wait4, 500, 11100, 0, 0, 0},				//	wait 0.5 sec to search for target

			{11120, Branch, 0, LeftPhoto, GE, 500, 11170},		//	If the left photo sensor is on, branch to CCW search step
			{11130, TurnRelative, 500, -1, 50, -1, 0},			//	While not seeing LeftPhoto, turn CCW, 1 degree at a time, for up to 5 degrees
			{11140, Wait4, 500, 11130, 0, 0, 0},				//	wait 0.5 sec to search for target
			{11150, CountedLoop, 0, 0, 1, 5, 11120},			//	Max CCW search of 5 degrees

			{11160, Branch, 0, 0, 0, 0, 11270},					//	Could not locate the target, quit

			{11170, Branch, 0, RightPhoto, GE, 500, 11220},		//	Left photo sensor is on, now search for right photo sensor
			{11180, TurnRelative, 500, -1, 50, -1, 0},			//	While not seeing RightPhoto, turn CCW, 1 degree at a time, for up to 15 degrees
			{11190, Wait4, 500, 11180, 0, 0, 0},				//	wait 0.5 sec to search for target
			{11200, CountedLoop, 0, 0, 1, 15, 11170},			//	Max CCW search of 15 degrees

			{11210, Branch, 0, 0, 0, 0, 11270},				//	Could not locate the target, quit

			{11220, Shooter, 2000, SHOOTER_LAUNCH_POWER, BOULDER_HOLD, 0, 0},		//	spin up the launcher to SHOOTER_LAUNCH_POWER watts
			{11230, Scooper, 1000, 47, 50, 0, 0},				//	raise the shooting angle to 14 degrees to hit the target center
			{11240, Wait4, 1000, 11230, 0, 0, 0},				//	wait 1 sec to raise the shooter elevation
			{11250, Wait4, 2000, 11250, 0, 0, 0},				//	wait 2 sec to spin up the shooter to the launch power
			{11260, Shooter, 2000, SHOOTER_LAUNCH_POWER, BOULDER_LAUNCH, 0, 0},	//	Launch the boulder
			{11270, Wait4, 2000, 11260, 0, 0, 0},				//	wait 2 seconds to complete the launch

		{-1, Middle, 0, 0, 0, 0, 0},						//	Drive to the optimal spot to shoot the HighLeft goal and fire
			{12010, MoveTo, 4000, -12, 209, 80, 0},			//	move to position at 80% power
			{12020, Scooper, 2000, 45, 50, 0, 0},				//	lower the shooting angle to 9 degrees so we can pick up the retro-reflective tape
			{12030, Wait4, 4000, 12010, 0, 0, 0},				//	wait 4 sec to arrive in shooting position
			{12040, TurnTo, 2000, 000, 180, 0, 0},				//	spin to face the target
			{12050, Wait4, 2000, 12040, 0, 0, 0},				//	wait 0.5 sec to search for target

			{12060, Branch, 0, LeftPhoto, GE, 500, 12170},		//	If the left photo sensor is on, branch to CCW search step
			{12070, TurnRelative, 500, 1, 50, 1, 0},			//	While not seeing LeftPhoto, turn CW, 1 degree at a time, for up to 5 degrees
			{12080, Wait4, 500, 12070, 0, 0, 0},				//	wait 0.5 sec to search for target
			{12090, CountedLoop, 0, 0, 1, 5, 12060},			//	Max CW search of 5 degrees

			{12100, TurnRelative, 1000, -6, 50, -1, 0},			//	Failing to find the LeftPhoto hit, look the other way
			{12110, Wait4, 500, 12100, 0, 0, 0},				//	wait 0.5 sec to search for target

			{12120, Branch, 0, LeftPhoto, GE, 500, 12170},		//	If the left photo sensor is on, branch to CCW search step
			{12130, TurnRelative, 500, -1, 50, -1, 0},			//	While not seeing LeftPhoto, turn CCW, 1 degree at a time, for up to 5 degrees
			{12140, Wait4, 500, 12130, 0, 0, 0},				//	wait 0.5 sec to search for target
			{12150, CountedLoop, 0, 0, 1, 5, 12120},			//	Max CCW search of 5 degrees

			{12160, Branch, 0, 0, 0, 0, 12270},					//	Could not locate the target, quit

			{12170, Branch, 0, RightPhoto, GE, 500, 12220},		//	Left photo sensor is on, now search for right photo sensor
			{12180, TurnRelative, 500, -1, 50, -1, 0},			//	While not seeing RightPhoto, turn CCW, 1 degree at a time, for up to 15 degrees
			{12190, Wait4, 500, 12180, 0, 0, 0},				//	wait 0.5 sec to search for target
			{12200, CountedLoop, 0, 0, 1, 15, 12170},			//	Max CCW search of 15 degrees

			{12210, Branch, 0, 0, 0, 0, 12270},				//	Could not locate the target, quit

			{12220, Shooter, 2000, SHOOTER_LAUNCH_POWER, BOULDER_HOLD, 0, 0},		//	spin up the launcher to SHOOTER_LAUNCH_POWER watts
			{12230, Scooper, 1000, 47, 50, 0, 0},				//	raise the shooting angle to 14 degrees to hit the target center
			{12240, Wait4, 1000, 12230, 0, 0, 0},				//	wait 1 sec to raise the shooter elevation
			{12250, Wait4, 2000, 12250, 0, 0, 0},				//	wait 2 sec to spin up the shooter to the launch power
			{12260, Shooter, 2000, SHOOTER_LAUNCH_POWER, BOULDER_LAUNCH, 0, 0},	//	Launch the boulder
			{12270, Wait4, 2000, 12260, 0, 0, 0},				//	wait 2 seconds to complete the launch

		{-1, RightHigh, 0, 0, 0, 0, 0},						//	Drive to the optimal spot to shoot the RightHigh goal and fire
			{13010, MoveTo, 4000, 85, 268, 80, 0},			//	move to position at 80% power
			{13020, Scooper, 2000, 45, 50, 0, 0},				//	lower the shooting angle to 9 degrees so we can pick up the retro-reflective tape
			{13030, Wait4, 4000, 13010, 0, 0, 0},				//	wait 4 sec to arrive in shooting position
			{13040, TurnTo, 2000, 300, 180, 0, 0},				//	spin to face the target
			{13050, Wait4, 2000, 13040, 0, 0, 0},				//	wait 0.5 sec to search for target

			{13060, Branch, 0, LeftPhoto, GE, 500, 13170},		//	If the left photo sensor is on, branch to CCW search step
			{13070, TurnRelative, 500, 1, 50, 1, 0},			//	While not seeing LeftPhoto, turn CW, 1 degree at a time, for up to 5 degrees
			{13080, Wait4, 500, 13070, 0, 0, 0},				//	wait 0.5 sec to search for target
			{13090, CountedLoop, 0, 0, 1, 5, 13060},			//	Max CW search of 5 degrees

			{13100, TurnRelative, 1000, -6, 50, -1, 0},			//	Failing to find the LeftPhoto hit, look the other way
			{13110, Wait4, 500, 13100, 0, 0, 0},				//	wait 0.5 sec to search for target

			{13120, Branch, 0, LeftPhoto, GE, 500, 13170},		//	If the left photo sensor is on, branch to CCW search step
			{13130, TurnRelative, 500, -1, 50, -1, 0},			//	While not seeing LeftPhoto, turn CCW, 1 degree at a time, for up to 5 degrees
			{13140, Wait4, 500, 13130, 0, 0, 0},				//	wait 0.5 sec to search for target
			{13150, CountedLoop, 0, 0, 1, 5, 13120},			//	Max CCW search of 5 degrees

			{13160, Branch, 0, 0, 0, 0, 13270},					//	Could not locate the target, quit

			{13170, Branch, 0, RightPhoto, GE, 500, 13220},		//	Left photo sensor is on, now search for right photo sensor
			{13180, TurnRelative, 500, -1, 50, -1, 0},			//	While not seeing RightPhoto, turn CCW, 1 degree at a time, for up to 15 degrees
			{13190, Wait4, 500, 13180, 0, 0, 0},				//	wait 0.5 sec to search for target
			{13200, CountedLoop, 0, 0, 1, 15, 13170},			//	Max CCW search of 15 degrees

			{13210, Branch, 0, 0, 0, 0, 13270},				//	Could not locate the target, quit

			{13220, Shooter, 2000, SHOOTER_LAUNCH_POWER, BOULDER_HOLD, 0, 0},		//	spin up the launcher to SHOOTER_LAUNCH_POWER watts
			{13230, Scooper, 1000, 47, 50, 0, 0},				//	raise the shooting angle to 14 degrees to hit the target center
			{13240, Wait4, 1000, 13230, 0, 0, 0},				//	wait 1 sec to raise the shooter elevation
			{13250, Wait4, 2000, 13250, 0, 0, 0},				//	wait 2 sec to spin up the shooter to the launch power
			{13260, Shooter, 2000, SHOOTER_LAUNCH_POWER, BOULDER_LAUNCH, 0, 0},	//	Launch the boulder
			{13270, Wait4, 2000, 13260, 0, 0, 0},				//	wait 2 seconds to complete the launch

		{-1, RightLower, 0, 0, 0, 0, 0},					// Drive to the optimal spot to shoot the RightLower goal and fire
			{14010, MoveTo, 4000, 68, 278, 80, 0},				//	move to position at 80% power
			{14020, Scooper, 2000, 9, -50, 0, 0},				//	lower the shooting angle to 9 degrees so we can pick up the retro-reflective tape
			{14030, Wait4, 4000, 14010, 0, 0, 0},				//	wait 4 sec to arrive in shooting position
			{14040, TurnTo, 2000, 300, 180, 0, 0},				//	spin to face the target
			{14050, Wait4, 2000, 14040, 0, 0, 0},				//	wait 0.5 sec to search for target

			{14060, Branch, 0, LeftPhoto, GE, 500, 14170},		//	If the left photo sensor is on, branch to CCW search step
			{14070, TurnRelative, 500, 1, 50, 1, 0},			//	While not seeing LeftPhoto, turn CW, 1 degree at a time, for up to 5 degrees
			{14080, Wait4, 500, 14070, 0, 0, 0},				//	wait 0.5 sec to search for target
			{14090, CountedLoop, 0, 0, 1, 5, 14060},			//	Max CW search of 5 degrees

			{14100, TurnRelative, 1000, -6, 50, -1, 0},			//	Failing to find the LeftPhoto hit, look the other way
			{14110, Wait4, 500, 14100, 0, 0, 0},				//	wait 0.5 sec to search for target

			{14120, Branch, 0, LeftPhoto, GE, 500, 14170},		//	If the left photo sensor is on, branch to CCW search step
			{14130, TurnRelative, 500, -1, 50, -1, 0},			//	While not seeing LeftPhoto, turn CCW, 1 degree at a time, for up to 5 degrees
			{14140, Wait4, 500, 14130, 0, 0, 0},				//	wait 0.5 sec to search for target
			{14150, CountedLoop, 0, 0, 1, 5, 14120},			//	Max CCW search of 5 degrees

			{14160, Branch, 0, 0, 0, 0, 14270},					//	Could not locate the target, quit

			{14170, Branch, 0, RightPhoto, GE, 500, 14220},		//	Left photo sensor is on, now search for right photo sensor
			{14180, TurnRelative, 500, -1, 50, -1, 0},			//	While not seeing RightPhoto, turn CCW, 1 degree at a time, for up to 15 degrees
			{14190, Wait4, 500, 14180, 0, 0, 0},				//	wait 0.5 sec to search for target
			{14200, CountedLoop, 0, 0, 1, 15, 14170},			//	Max CCW search of 15 degrees

			{14210, Branch, 0, 0, 0, 0, 14270},					//	Could not locate the target, quit

			{14220, Shooter, 2000, SHOOTER_LAUNCH_POWER, BOULDER_HOLD, 0, 0},		//	spin up the launcher to SHOOTER_LAUNCH_POWER watts
			{14230, Scooper, 1000, 14, 50, 0, 0},				//	raise the shooting angle to 14 degrees to hit the target center
			{14240, Wait4, 1000, 14230, 0, 0, 0},				//	wait 1 sec to raise the shooter elevation
			{14250, Wait4, 2000, 14250, 0, 0, 0},				//	wait 2 sec to spin up the shooter to the launch power
			{14260, Shooter, 2000, SHOOTER_LAUNCH_POWER, BOULDER_LAUNCH, 0, 0},	//	Launch the boulder
			{14270, Wait4, 2000, 14260, 0, 0, 0},				//	wait 2 seconds to complete the launch

		{-1, 99999, -1, -1, -1, -1, -1},						//	End of Command Steps
		};

	Timer	AutonomousTimer;
//	RobotStateBuffer AutoTarget;		// Each autonomous command will
//	RobotStateBuffer	*pMyAutoTarget;

	SpeedControl		save_Speed_Control;	// motor = pwr * Drivetrain_Speed_Control / 4
	bool				save_Spin_Control;	// enabled/disabled by gamepad button input
	DriveOrientation	save_Directional_Orientation;

	typedef enum {notstarted=0, active=1, completed=2, timedout=3} CompletionStates;

	int step, next_step, autostep_index, timeout;
	int	AutoTime, ElapsedTime;
	float distance, prev_distance;
	AutoAction	action;

	struct AutoStep_t {
		int					LineNumber;					// this is the Line Number (-1 = header record)
		AutoAction			action;						// this really isn't needed (I might throw this out to save memory space).
		void				(AUTOPILOT::*function)();	// this is a pointer to the function needed to execute the command
		int					Timeout;
		CompletionStates	Status;
		union	{
			int				Timer;					// this will be initialized at the start of the task and chewcked with each iteration
			int				LoopCounter;			// the CountedLoop command has no Timeout component, so we use that spot to hold the LoopCounter value.
			};
		union  {
			struct	{
					int		SpeedControl;		// sets the speed control value for the AutoPilot
					int		SpinControl;		// sets the spin control value for the AutoPilot
					int		Orientation;		// sets the Robot or Field drive orientation (not applicable to tank drive)
					}SetControls;
			struct	{
					int		Initial;			// initial value of LoopCounter
					int		Increment;			// value to add to the LoopCounter value (positive or negative) with each cycle
					int		Final;				// End Loop when LoopCounter == Final (set Condition flag to completed)
					int		NextLine;			// If LoopCounter != Final, set next step to this line number (must be a backward branch).
												// Need to clear the condition flags (set to 0) for all steps
												// between current and branch to step (including the branch to step, but not including this step).
					} CountedLoop;					// (note: setting Initial == Final is a NoOp
			struct	{
					int		Conditon;			// 0=unconditional; 1=LT; 2=LE; 3=EQ; 4=GE; 5=GT
					double	*pStateValue;		// pointer to a StateBuffer value.  NOTE:  This value is multiplied by 1000 and compared to Value
					int		Value;				// Compare the StateBuffer value to this value.  NOTE:  This value is expressed in milli-units
					int		NextLine;			// If condition is met, set next step to this line number
												// If branching backwards, need to clear the condition flags (set to 0) for all steps between current
												// and branch to step (including this step and the branch to step).
												// If branching forwards, need to set the condition flags to completed for all steps between current
												// and branch to step (including this step, but not including the branch to step).
												// Post implementation is similar to wait4 (do not move to next step unless condition flag is set)
												// (note that branching forward sets the completion flag and allows moving on to the next step)
					} Branch;
			struct	{
					int		AbsoluteOrRelative;	// 0 = Absolute X and Y coordinates; 1 = Relative to Starting Coordinates
					int		X;
					int		Y;
					} Arrive;
			struct	{
					int		Wait4Line;			// line number of the command we are waiting to complete
					} Wait4;
			struct	{
					int		Xcoordinate;		// expressed in inches from a known origin point on the field
					int		Ycoordinate;
					int		MaxSpeed;			// expressed in inches per second
					} MoveTo;
			struct	{
					int		Xdistance;			// number of inches to move from the current position along the X axis
					int		Ydistance;			// number of inches to move from the current position along the Y axis
					int		MaxSpeed;			// expressed in inches per second
					} MoveRelative;
			struct	{
					int		Heading;			// compass bearing in degrees
					int		MaxRate;			// expressed in degrees per second
					int		Direction;			// 0=shortest direction; 1=clockwise; -1=counter-clockwise
					} TurnTo;
			struct	{
					int		Degrees;			// angular change in degrees
					int		MaxRate;			// expressed in degrees per second
					int		Direction;			// 0=shortest direction; 1=clockwise; -1=counter-clockwise
					} TurnRelative;
			struct	{
					int		LaunchPower;		// shooter wheel power
					int		BoulderPosition;	// forward or back
					} Shooter;
			struct	{
					int		Elevation;			// angle of elevation
					int		ElevatorSpeed;		// speed of changing in elevaton in degrees per second
					} Scooper;
			struct	{
					int		Elevation;			// angle of elevation
					int		ElevatorSpeed;		// speed of changing in elevaton in degrees per second
					int		Extension;			// distance of hooks in inches out in front of the robot
					int		ExtensionSpeed;		// speed of changing in extension in degrees per second
												// Note: This controls both the extension arms and the winch
												//		While extending, the winch has a slight back power to keep the winch cable taught
												//		While retracting, the extension motor keeps the extension line taught
					} Climber;
			} parameters;
		};


	struct Scenario_t
		{
		int				ScenarioID;
//		string			Description;		// deprecated
		int				NumberOfSteps;
		int				Starting_X_Position;
		int				Starting_Y_Position;
		int				Starting_Heading;
		int				Currently_Unused;				// available to implement new feature
		AutoStep_t		Steps[AUTOPILOT_MAX_STEPS];
		} *pDefenseProgram[5], *pGoalProgram[5], *pRogram;


	int SelectedScenario;
	int step_index;
	int outer_works_posn, outer_works_index, start_index, goal_index;


	void LoadProgram(AutoCommand_t Steps[], int ScenarioID, Scenario_t *Program, int ProgramIndex);
	void DisplaySteps(std::string command);
	void ExecSetControls();
	void ExecCountedLoop();
	void ExecBranch();
	void ExecArrive();
	void ExecWait4();
	void MoveToPosition();
	void ExecMoveTo();
	void ExecMoveRelative();
	void TurnToHeading();
	void ExecTurnTo();
	void ExecTurnRelative();
	void ExecShooter();
	void ExecScooper();
	void ExecClimber();

	/* all methods above (except the Wait4 method) execute and move on immediately to the next task
	 * If there are dependencies, they need to be satisfied by issuing a Wait4 command
	 * On each iteration, the pMyRobotState buffer is checked to see if any actuator/motor movements are needed in order
	 * to converge on the targets.  When targets are achieved, the actuators/motors are turned off and the completed boolean is set.
	 * Actuator/motor movements are initiated by updating the INPUT buffer values.
	 *
	 * Note:	Completed_Tasks[0] is used for non-synchronized operations.  <--- OOPS!  The code show this flag as the master program flag.  Which is correct?
	 * 			To wait for an elapsed time, execute the NoOp with a specified timeout value, followed by a Wait4.
	 * 			To wait for a game time, at the start of the game, execute the NoOp with the timeout set to the desired game time.
	 */

};

#endif /* AUTOPILOT_H */

