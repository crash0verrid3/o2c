/*
 * Autonomous.cpp
 *
 *  Created on: Jan 31, 2015
 *      Author: Rod
 */

#include <AutoPilot.h>

/*
 * We have multiple choosers presented to the SmartDashboard to allow the driver to select
 * 		- What defense is configured in each of the 5 outerworks positions
 * 				1 chooser for each of the 5 different positions
 * 				first chooser only has one choice (low bar)
 * 				each outerworks comes with a pre-canned set of autonomous steps to executed
 * 				when a defense is selected, the pre-canned set of autonomous steps are loaded into the position-dependent autonomous program
 * 		- What defense will be crossed in Autonomous mode (Starting Position)
 * 				1 chooser with 5 position choices
 * 				each position will designate starting coordinates and heading
 * 				when a starting position is selected, the starting coordinates are selected and the position-dependent autonoumous program is selected (by index)
 * 		- What goal to shoot at
 * 				1 chooser with 5 goal choices
 * 				each goal choice comes with a pre-canned set of autonomous steps to execute after arriving in the courtyard
 * 				when a goal is selected, the pre-canned set of autonomous steps are loaded into the goal program
 */


AUTOPILOT :: AUTOPILOT()
	{
	pStartChooser = new SendableChooser();
	pGoalChooser = new SendableChooser();
	for (outer_works_posn=0; outer_works_posn < POSITION_COUNT; outer_works_posn++)
		{
		pDefenseChooser[outer_works_posn] = new SendableChooser();
		pDefenseProgram[outer_works_posn] = new Scenario_t;
		}
	for (goal_index=0; goal_index < GOAL_COUNT; goal_index++)
		{
		pGoalProgram[goal_index] = new Scenario_t;
		}
	pMyTargetState = 0;
	pRogram = 0;
	SelectedScenario = 0;
	step = next_step = 0;
	action = SetControls;
	autostep_index = 0;
	AutonomousTimer.Start();
	AutoTime = ElapsedTime = 0;
	timeout = 0;
	save_Speed_Control = Normal;
	save_Spin_Control = true;
	save_Directional_Orientation = Robot;
	prev_distance = distance = 0;
	step_index = outer_works_posn = outer_works_index = start_index = goal_index = 0;
	}


void AUTOPILOT :: Init(RobotStateBuffer *pRobotState, RobotStateBuffer *pTargetState, INPUT *pInput)
	{
	pMyRobotState = pRobotState;
	pMyTargetState = pTargetState;
	pMyInput = pInput;
	pMyRobotState->Autonomous_Init++;
	pMyRobotState->Auto_Defense = pMyTargetState->Auto_Defense = -1;
	pMyRobotState->Auto_Goal = pMyTargetState->Auto_Goal = -1;
	// save Driver Preferences
	save_Speed_Control = pMyRobotState->Drivetrain_Speed_Control;
	save_Spin_Control = pMyRobotState->Drivetrain_Spin_Control;
	save_Directional_Orientation = pMyRobotState->Drivetrain_Directional_Orientation;
	pMyRobotState->Auto_Step = 0;
	pMyRobotState->Auto_State = 0;

	SmartDashboard::PutString("Autonomous Debug", "starting Init");

	// Now initialize all the Defense AutoPilot Starting Positions
	// And set values into the Starting Position and Defense Choosers
	for (int outer_works_posn = 0; outer_works_posn<POSITION_COUNT; outer_works_posn++)
		{
		pDefenseProgram[outer_works_posn]->ScenarioID = StartSteps[outer_works_posn][0];
		pDefenseProgram[outer_works_posn]->NumberOfSteps = StartSteps[outer_works_posn][1];
		pDefenseProgram[outer_works_posn]->Starting_X_Position = StartSteps[outer_works_posn][2];
		pDefenseProgram[outer_works_posn]->Starting_Y_Position = StartSteps[outer_works_posn][3];
		pDefenseProgram[outer_works_posn]->Starting_Heading = StartSteps[outer_works_posn][4];
		pDefenseProgram[outer_works_posn]->Currently_Unused = StartSteps[outer_works_posn][5];

//		SmartDashboard::PutString("Autonomous Debug", "pDefenseProgram["+std::to_string(outer_works_posn)+"]");

		// Set up the choices in the DefenseChooser for the outworks at position i + 1 (we count from 1, but c++ counts from 0)
		for (outer_works_index = 0; outer_works_index < DEFENSE_COUNT; outer_works_index++)
			{
			if (DefenseName[outer_works_index].compare(DefenseDefault[outer_works_posn]) == 0)
				{
				pDefenseChooser[outer_works_posn]->AddDefault(DefenseName[outer_works_index], (void*)&DefenseName[outer_works_index]);
				pMyRobotState->Field_Defense[outer_works_posn] = outer_works_index + 1;

//				for (step_index = 0; Steps[step_index][0] != -1 || Steps[step_index][1] < ScenarioID; step_index++);

				LoadProgram(DefenseSteps, outer_works_index, pDefenseProgram[outer_works_posn], outer_works_posn);			// Load pGoalProgram[goal_index] program steps from GoalSteps[step_index]

				}
			else if ((outer_works_posn > 0) && (outer_works_index > 0))		// posn 0 is the low bar and does not change, so no reason to add other choices to the posn 0 chooser
				{															// and the low bar cannot be in any other position
				pDefenseChooser[outer_works_posn]->AddObject(DefenseName[outer_works_index], (void*)&DefenseName[outer_works_index]);
				}
			}
		// Load the values for the default Defense as if the driver had selected the default
		if (StartPosition[outer_works_posn].compare(StartDefault) == 0)
			{
			pStartChooser->AddDefault(StartPosition[outer_works_posn], (void*)&StartPosition[outer_works_posn]);
			pMyRobotState->Auto_Defense = outer_works_posn + 1;
			pMyRobotState->Auto_Start_Position = outer_works_posn + 1;												// Moved to Select method
			pMyRobotState->Auto_Defense_Steps = pDefenseProgram[outer_works_posn]->NumberOfSteps;
			pMyTargetState->Robot_Position_X = pDefenseProgram[outer_works_posn]->Starting_X_Position;
			pMyTargetState->Robot_Position_Y = pDefenseProgram[outer_works_posn]->Starting_Y_Position;
			pMyTargetState->Robot_Heading = pMyTargetState->Robot_Direction = pDefenseProgram[outer_works_posn]->Starting_Heading;
			pMyRobotState->Robot_Direction = pMyRobotState->Robot_Heading = pMyTargetState->Robot_Heading;
			pMyTargetState->Robot_Speed = pMyRobotState->Robot_Speed = 0;
			}
		else
			{
			pStartChooser->AddObject(StartPosition[outer_works_posn], (void*)&StartPosition[outer_works_posn]);
			}
		}
	// Now initialize all the Goal AutoPilot Programs
	// And set values into the Goal Chooser
	// Since the goals are all fixed, we can preprogram all five goal programs
	for (goal_index = 0; goal_index < GOAL_COUNT; goal_index++)
		{
		if (Goal[goal_index].compare(GoalDefault) == 0)
			{
			pGoalChooser->AddDefault(Goal[goal_index], (void*)&Goal[goal_index]);
			}
		else
			{
			pGoalChooser->AddObject(Goal[goal_index], (void*)&Goal[goal_index]);
			}
		LoadProgram(GoalSteps, goal_index, pGoalProgram[goal_index], goal_index);			// Load pGoalProgram[goal_index] program steps from GoalSteps[step_index]
		}
	// Send all choosers to the SmartDashboard, 5 Defense choosers, 1 Start chooser and 1 Goal chooser
	for (outer_works_posn = 0; outer_works_posn < POSITION_COUNT; outer_works_posn++)
		{
		SmartDashboard::PutData("Defense"+std::to_string(outer_works_posn), pDefenseChooser[outer_works_posn]);
		}
	SmartDashboard::PutData("Start Position", pStartChooser);
	SmartDashboard::PutData("Goal", pGoalChooser);

	SelectScenario();			// Select the default outerworks and goal scenarios

	pMyRobotState->Autonomous_Init++;
	}

void AUTOPILOT :: LoadProgram(AutoCommand_t Steps[], int ScenarioID, Scenario_t *Program, int ProgramIndex)
	{
	int s;
	// First, locate the step_index within the Steps array where the Scenario commands begin
	for (step_index = 0; Steps[step_index][0] != -1 || Steps[step_index][1] < ScenarioID; step_index++);

	// Then load in the header information
//	if (Steps == DefenseSteps)
//		{
//		SmartDashboard::PutString("Autonomous Looking"+std::to_string(ProgramIndex), "Looking for ScenarioID "+DefenseName[ScenarioID]);
//		SmartDashboard::PutString("Autonomous Loading"+std::to_string(ProgramIndex), "Loading "+DefenseName[ScenarioID]+" DefenseSteps["+std::to_string(step_index)+"] into Program");
//		}
//	if (Steps == GoalSteps)
//		{
//		SmartDashboard::PutString("Autonomous Loading"+std::to_string(ScenarioID), "Loading "+Goal[ScenarioID]+" GoalSteps["+std::to_string(step_index)+"] into Program");
//		}

	Program->ScenarioID = Steps[step_index][1];
	Program->Starting_X_Position = Steps[step_index][2];			// This is ignored, we start from wherever the Navigation system says we are
	Program->Starting_Y_Position = Steps[step_index][3];			// This is ignored, we start from wherever the Navigation system says we are
	Program->Starting_Heading = Steps[step_index][4];			// This is ignored, we start from wherever the Navigation system says we are
	Program->Currently_Unused = Steps[step_index][5];
	// And then compile all the commands described by each step until you get to the next scenario header record
	int i = 0;
//	int q = step_index;
	for (int first_step = ++step_index; Steps[step_index][0] != -1; ++step_index && ++i)
		{
		#ifdef REPORT_AUTO_DEBUG
 			if (Steps == DefenseSteps)
				{
 				SmartDashboard::PutString("DefenseStep"+std::to_string(step_index + 1 - first_step), "");
				}
 			else if (Steps == GoalSteps)
				{
 				SmartDashboard::PutString("GoalStep"+std::to_string(step_index + 1 - first_step), "");
				}
		#endif
		Program->Steps[i].LineNumber = Steps[step_index][0];
		Program->Steps[i].action = (AutoAction) Steps[step_index][1];
		Program->Steps[i].Timeout = Steps[step_index][2];
		if (Steps[step_index][2] == 0) Program->Steps[i].Timeout = AUTONOMOUS_PERIOD;
		Program->Steps[i].Status = notstarted;
		switch (Steps[step_index][1])
			{
			case CountedLoop:
				Program->Steps[i].function = &AUTOPILOT::ExecCountedLoop;
				Program->Steps[i].parameters.CountedLoop.Initial = Steps[step_index][3];
				Program->Steps[i].parameters.CountedLoop.Increment = Steps[step_index][4];
				Program->Steps[i].parameters.CountedLoop.Final = Steps[step_index][5];
				// here we need to decipher the line number contained in Steps[step_index][6] into a command step index
				for (s = 0; (s <= i) && (Steps[first_step + s][0] != Steps[step_index][6]); s++);
				Program->Steps[i].parameters.CountedLoop.NextLine = s;
				SmartDashboard::PutString("Autonomous Command", "CountedLoop");
				break;

			case Arrive:
				Program->Steps[i].function = &AUTOPILOT::ExecArrive;
				Program->Steps[i].parameters.Arrive.AbsoluteOrRelative = Steps[step_index][3];
				Program->Steps[i].parameters.Arrive.X = Steps[step_index][4];
				Program->Steps[i].parameters.Arrive.Y = Steps[step_index][5];
				SmartDashboard::PutString("Autonomous Command", "Arrive");
				break;

			case SetControls:
				Program->Steps[i].function = &AUTOPILOT::ExecSetControls;
				Program->Steps[i].parameters.SetControls.SpeedControl = Steps[step_index][3];
				Program->Steps[i].parameters.SetControls.SpinControl = Steps[step_index][4];
				Program->Steps[i].parameters.SetControls.Orientation = Steps[step_index][5];
				SmartDashboard::PutString("Autonomous Command", "SetControls");
				break;

			case Wait4:
				Program->Steps[i].function = &AUTOPILOT::ExecWait4;
				// here we need to decipher the line number contained in Steps[step_index][3] into a command step index
//				Program->Steps[i].parameters.Wait4.Wait4Line = i;
//				for (int s = 0; (Steps[first_step + s][0] != Steps[step_index][0]) && (Steps[s][0] != Steps[step_index][3]); Program->Steps[i].parameters.Wait4.Wait4Line = s++);
//				for (int s = 1; (s <= i) && (Steps[first_step + s][0] != Steps[step_index][3]); Program->Steps[i].parameters.Wait4.Wait4Line = s++);
				for (s = 0; (s <= i) && (Steps[first_step + s][0] != Steps[step_index][3]); s++);
				Program->Steps[i].parameters.Wait4.Wait4Line = s;
				SmartDashboard::PutString("Autonomous Command", "Wait4");
				break;

			case MoveTo:
				Program->Steps[i].function = &AUTOPILOT::ExecMoveTo;
				Program->Steps[i].parameters.MoveTo.Xcoordinate = Steps[step_index][3];
				Program->Steps[i].parameters.MoveTo.Ycoordinate = Steps[step_index][4];
				Program->Steps[i].parameters.MoveTo.MaxSpeed = Steps[step_index][5];
				SmartDashboard::PutString("Autonomous Command", "MoveTo");
				break;

			case MoveRelative:
				Program->Steps[i].function = &AUTOPILOT::ExecMoveRelative;
				Program->Steps[i].parameters.MoveRelative.Xdistance = Steps[step_index][3];
				Program->Steps[i].parameters.MoveRelative.Ydistance = Steps[step_index][4];
				Program->Steps[i].parameters.MoveRelative.MaxSpeed = Steps[step_index][5];
				SmartDashboard::PutString("Autonomous Command", "MoveRelative");
				break;

			case TurnTo:
				Program->Steps[i].function = &AUTOPILOT::ExecTurnTo;
				Program->Steps[i].parameters.TurnTo.Heading = Steps[step_index][3];
				Program->Steps[i].parameters.TurnTo.MaxRate = Steps[step_index][4];
				SmartDashboard::PutString("Autonomous Command", "TurnTo");
				break;

			case TurnRelative:
				Program->Steps[i].function = &AUTOPILOT::ExecTurnRelative;
				Program->Steps[i].parameters.TurnRelative.Degrees = Steps[step_index][3];
				Program->Steps[i].parameters.TurnRelative.MaxRate = Steps[step_index][4];
				SmartDashboard::PutString("Autonomous Command", "TurnRelative");
				break;

			case Branch:
				Program->Steps[i].function = &AUTOPILOT::ExecBranch;
				switch (Steps[step_index][3])
					{
					case Xcoord:
						Program->Steps[i].parameters.Branch.pStateValue = &pMyRobotState->Robot_Position_X;
						break;

					case Ycoord:
						Program->Steps[i].parameters.Branch.pStateValue = &pMyRobotState->Robot_Position_Y;
						break;

					case Heading:
						Program->Steps[i].parameters.Branch.pStateValue = &pMyRobotState->Robot_Heading;
						break;

					case Roll:
						Program->Steps[i].parameters.Branch.pStateValue = &pMyRobotState->Robot_Roll;
						break;

					case Pitch:
						Program->Steps[i].parameters.Branch.pStateValue = &pMyRobotState->Robot_Pitch;
						break;

					case LeftPhoto:
						Program->Steps[i].parameters.Branch.pStateValue = &pMyRobotState->Sensors_LeftPhoto;
						break;

					case RightPhoto:
						Program->Steps[i].parameters.Branch.pStateValue = &pMyRobotState->Sensors_RightPhoto;
						break;

					case light:
						Program->Steps[i].parameters.Branch.pStateValue = &pMyRobotState->Sensors_LightDetect;
						break;

					case Range:
						Program->Steps[i].parameters.Branch.pStateValue = &pMyRobotState->Sensors_Range;
						break;

					case ShooterAngle:
						Program->Steps[i].parameters.Branch.pStateValue = &pMyRobotState->Shooter_Elevation;
						break;

					case ClimberAngle:
//						Program->Steps[i].parameters.Branch.pStateValue = &pMyRobotState->Climber_Elevation;
						break;

					case ClimberReach:
//						Program->Steps[i].parameters.Branch.pStateValue = &pMyRobotState->Climber_Extension;
						break;

					default:
						break;
					}
				Program->Steps[i].parameters.Branch.Conditon = Steps[step_index][4];
				Program->Steps[i].parameters.Branch.Value = Steps[step_index][5];
//				Program->Steps[i].parameters.Branch.NextLine = i;
				// here we need to decipher the line number contained in Steps[step_index][6] into a command step index
//				for (int s = 0; (Steps[first_step + s][0] != Steps[step_index][0]) && (Steps[s][0] != Steps[step_index][3]); Program->Steps[i].parameters.Wait4.Wait4Line = s++);
//				for (int s = 1; (Steps[first_step + s][0] != -1) && (Steps[first_step + s][0] != Steps[step_index][6]); s++)
//					Program->Steps[i].parameters.Branch.NextLine = s;	// + (Steps[first_step + s][0] == -1) ? -1 : 0;
//				for (int s = 1; (Steps[first_step + s][0] != -1) && (Steps[first_step + s][0] != Steps[step_index][6]); Program->Steps[i].parameters.Branch.NextLine = s++)
				for (s = 0; (Steps[first_step + s][0] != -1) && (Steps[first_step + s][0] != Steps[step_index][6]); s++);
				Program->Steps[i].parameters.Branch.NextLine = s; // - (Steps[first_step + s][0] == -1) ? 1: 0;	// avoid branching beyond end of program if step number not found
				SmartDashboard::PutString("Autonomous Command", "Branch");
				break;

			case Shooter:
				Program->Steps[i].function = &AUTOPILOT::ExecShooter;
				Program->Steps[i].parameters.Shooter.LaunchPower = Steps[step_index][3];
				Program->Steps[i].parameters.Shooter.BoulderPosition = Steps[step_index][4];
				SmartDashboard::PutString("Autonomous Command", "Shooter");
				break;

			case Scooper:
				Program->Steps[i].function = &AUTOPILOT::ExecScooper;
				Program->Steps[i].parameters.Scooper.Elevation = Steps[step_index][3];
				Program->Steps[i].parameters.Scooper.ElevatorSpeed = Steps[step_index][4];
				SmartDashboard::PutString("Autonomous Command", "Scooper");
				break;

			case Climber:
				Program->Steps[i].function = &AUTOPILOT::ExecClimber;
				Program->Steps[i].parameters.Climber.Elevation = Steps[step_index][3];
				Program->Steps[i].parameters.Climber.ElevatorSpeed = Steps[step_index][4];
				Program->Steps[i].parameters.Climber.Extension = Steps[step_index][5];
				Program->Steps[i].parameters.Climber.ExtensionSpeed = Steps[step_index][6];
				SmartDashboard::PutString("Autonomous Command", "Climber");
				break;

			default:
				Program->Steps[i].function = 0;
				break;
			}
		}
	Program->NumberOfSteps = i;
//	if (Steps == DefenseSteps) SmartDashboard::PutString("DefenseSteps Loaded"+std::to_string(ProgramIndex), "Loaded "+std::to_string(Program->NumberOfSteps)+" "+DefenseName[ScenarioID]+" steps from DefenseSteps["+std::to_string(q)+"] into Program");
//	if (Steps == GoalSteps) SmartDashboard::PutString("GoalSteps Loaded"+std::to_string(ProgramIndex), "Loaded "+std::to_string(Program->NumberOfSteps)+" "+Goal[ScenarioID]+" steps from  GoalSteps["+std::to_string(q)+"] into Program");
	}






/*
 * We will start building our autonomous program with the pMyRobotState->Auto_Start_Position value.
 * This will translate to specific starting coordinates and heading.  Note that robot positioning
 * within an acceptable margin of error will be critical.
 *
 * The next phase of the autonomous program will be based off the pMyRobotState->Field_Defense[pMyRobotState->Auto_Start_Position] value.
 * This will translate to the starting index within the AutonomousSteps matrix.
 *
 * The next phase of the autonomous program will again be based off the pMyRobotState->Auto_Start_Position value.
 * This section of code will be designed to detect successful arrival in the courtyard and then reset the
 * robot position to the appropriate offset from the original starting position.  This will allow us track wheel
 * movement via the encoders during the next phase.
 *
 * The final phase of the autonomous program will be based off the pMyRobotState->Auto_Goal value.
 * This value will translate to absolute field coordinates directing the robot to move that location
 * then turn to face the goal.  While moving to position, the shooter will be elevated to the correct
 * angle.  On arrival at the optimal shooting coordinates, spin up the thrower motors, locate the target
 * center, and launch the boulder.   Note:  This phase of the AutoPilot code does not change much
 * when the driver selects a different goal.  So, all five Goal Programs are prebuilt in the Init method.
 *
 * It might make a lot of sense to break the old AutonomousSteps matrix into several phase-related matrices.
 * 		StartPositionSteps
 * 		DefenseSteps
 * 		GoalSteps
 *
 * It might work nicely to be able to reload some phases of the autonomous program during TeleOp in order
 * to provide a one-button auto-score feature.  For example, if you drive up close to one of the 5 pre-defined
 * scoring positions and hit the auto-score button, it will execute the final phase of the autonomous program,
 * perhaps more quickly than the human driver could do it.
 *
 * A different auto-score button might be used to initiate the autonomous program at an earlier entry point,
 * such as automatically drive the robot to the nearest (or selected) scoring position and shoot.
 *
 * I'm also thinking that we might enable a feature to automagically elevate the shooter to the proper angle
 * whenever we are driving around in the courtyard.
 *
 * We may also be able to execute certain AutoPilot programs to cross defenses.
 */
void AUTOPILOT :: SelectScenario()
	{
	GoalSelected = *((std::string*)pGoalChooser->GetSelected());	// Get the current selected goal from the goal chooser

	for (goal_index = 0; goal_index < GOAL_COUNT; goal_index++)		// rifle through all the goals looking for the goal that matches the goal chooser value
		{
		if ((GoalSelected == Goal[goal_index]) 						// locate the goal_index that corresponds to the selected value from the goal chooser
		&& (pMyRobotState->Auto_Goal != (goal_index + 1)))			// and, if it is different than the previously selected value (or default value)
			{
			pMyRobotState->Auto_Goal = goal_index + 1;				// then, update robotstate buffer to point to the newly selected goal program
			pMyRobotState->Auto_Goal_Steps = pGoalProgram[goal_index]->NumberOfSteps;
			// There is a potential flaw here.  I can add objects to the chooser in any order I like.  But the chooser will alphabetize the objects.
			// So, when the driver selects choice 3, it will not necessarily correspond to the 4th object that I added to the chooser.
			// Therefore, we will load the AutoPilot program steps in the order in which they are provided in the GoalSteps matrix.
			// But, when the driver selects a GoalChooser value, we will need to identify the correct corresponding AutoPilot program index.
			}
		}

	StartPostionSelected = *((std::string*)pStartChooser->GetSelected());				// Next, get the current selected StartPosition (which corresponds to the outerworks defense)
	for (outer_works_posn = 0; outer_works_posn < POSITION_COUNT; outer_works_posn++)	// rifle through all the outerworks positions for a position that matches the selected start position
		{
		if ((StartPostionSelected == StartPosition[outer_works_posn])		// locate the outer works position that matches the selected value from the start position chooser
		&& (pMyRobotState->Auto_Start_Position != (outer_works_posn + 1)))	// and, if it is different than the previously selected value (or default value)
			{
			pMyRobotState->Auto_Start_Position = outer_works_posn + 1;		// then, update the robotstate buffer to the newly selected start position
			pMyTargetState->Robot_Position_X = pDefenseProgram[outer_works_posn]->Starting_X_Position;
			pMyTargetState->Robot_Position_Y = pDefenseProgram[outer_works_posn]->Starting_Y_Position;
//			pMyTargetState->Robot_Heading = pMyTargetState->Drivetrain_Heading = pMyTargetState->Robot_Direction = pDefenseProgram[outer_works_posn]->Starting_Heading;
			pMyTargetState->Robot_Heading = pMyTargetState->Robot_Direction = pDefenseProgram[outer_works_posn]->Starting_Heading;
			pMyRobotState->Auto_Defense = outer_works_posn + 1;
			pMyRobotState->Auto_Defense_Steps = pDefenseProgram[outer_works_posn]->NumberOfSteps;
//			pMyRobotState->Drivetrain_Heading = pMyRobotState->Robot_Direction = pMyRobotState->Robot_Heading = pMyTargetState->Robot_Heading;
			pMyRobotState->Robot_Direction = pMyRobotState->Robot_Heading = pMyTargetState->Robot_Heading;
			pMyTargetState->Robot_Speed = pMyRobotState->Robot_Speed = 0;
			}
		DefenseSelected[outer_works_posn] = *((std::string*)pDefenseChooser[outer_works_posn]->GetSelected());	// For each of the outer defense positions, get the chooser value
		for (int d=0; d<DEFENSE_COUNT; d++)												// and rifle through all the defenses looking for the defense that matches the selected value
			{																			// for the associated position
			if ((DefenseSelected[outer_works_posn] == DefenseName[d]) 			// locate the defense that matches the selected value for the associated position
			&& (pMyRobotState->Field_Defense[outer_works_posn] != (d + 1)))		// and, if it is different than the previously selected value (or default value)
				{
				pMyRobotState->Field_Defense[outer_works_posn] = d + 1;			// then, update the robot state buffer and the program steps for the associated position
				LoadProgram(DefenseSteps, d, pDefenseProgram[outer_works_posn], outer_works_posn);			// Load pGoalProgram[goal_index] program steps from GoalSteps[step_index]
				}
			if (pMyRobotState->Auto_Start_Position == outer_works_posn) pMyRobotState->Auto_Defense_Steps = pDefenseProgram[outer_works_posn]->NumberOfSteps;
			}
		}
	pMyRobotState->Auto_Total_Steps = pDefenseProgram[pMyRobotState->Auto_Defense-1]->NumberOfSteps
									+ pGoalProgram[pMyRobotState->Auto_Goal-1]->NumberOfSteps;
	pMyRobotState->Auto_Step = 0;
	pMyRobotState->Auto_State = 1;
	}


void AUTOPILOT :: Start()
	{
	AutonomousTimer.Reset();
	// Set Autonomous Preferences -- will restore when Autonomous completes
	pMyRobotState->Drivetrain_Speed_Control = Speedy;	// motor = pwr * Drivetrain_Speed_Control / 4
	pMyRobotState->Drivetrain_Spin_Control = true;	// enabled/disabled by gamepad button input
	pMyInput->SprintModeBtn = false;
	pMyInput->SpinControlBtn = true;
	#if DRIVETRAINTYPE==MecanumDriveTrain
		pMyRobotState->Drivetrain_Directional_Orientation = Field;
		pMyInput->OrientModeBtn = true;
	#endif
	// We must presume that we placed the robot at the correct field coordinates and facing the correct heading
	pMyRobotState->Robot_Position_X = pMyTargetState->Robot_Position_X;
	pMyRobotState->Robot_Position_Y = pMyTargetState->Robot_Position_Y;
	pMyRobotState->Robot_Heading = pMyTargetState->Robot_Heading;
//	pMyRobotState->Robot_Heading = pMyRobotState->Drivetrain_Heading = pMyTargetState->Robot_Heading;
	pMyRobotState->Robot_Direction = 0;
	pMyRobotState->Robot_Speed = 0;
//	SelectedScenario = pMyRobotState->Auto_Defense - 1;
	pRogram = pDefenseProgram[pMyRobotState->Auto_Defense - 1];	// pRogram pointer set to selected defense program
	pMyRobotState->Auto_Step = 0;
	pMyRobotState->Auto_State = 2;					// Set Auto_State to running outerworks program
	}

void AUTOPILOT :: RunAuto()									// Execute all autonomous tasks in the defined scenario
	{
	// This is the public method called approx every 20msec while in autonomous mode.
	/*
	 * We need to check to see if any of the commands we have issued have either completed or timed out.
	 * If any previously executed commands have timed out, we need to set the appropriate MyInput values to 0 and set
	 * the associated completion flag. If any previously executed commands have completed, we need to set the appropriate
	 * completion flag.  If they have not completed, we need to update the MyInput instructions as necessary to complete
	 * the command(s).  We do this by making another call to the associated function.
	 *
	 * If the status of the current step is completed or the current step is not a Wait4 command, move on to the next step.
	 */
	switch (pMyRobotState->Auto_State)
		{
		case 1:
			// This never happens.  Auto_Init sets the Auto_State to 2 after initializing our starting parameters.
			// This will serve as the entry point when the driver hits the "A" button during TeleOp
			// If the robot is in the Neutral Zone, the pRogram pointer will be set to the selected outerworks program.
				pRogram = pDefenseProgram[pMyRobotState->Auto_Defense - 1];	// pRogram pointer set to selected defense program
			// If the robot is in the Courtyard, the pRogram pointer will be set to the selected goal program.
				pRogram = pGoalProgram[pMyRobotState->Auto_Goal - 1];

			// Reset all the Status flags so we can run any of the programs again later
			pMyRobotState->Auto_Total_Steps = pRogram->NumberOfSteps;
			for (int i = 0; i < pRogram->NumberOfSteps; pRogram->Steps[i++].Status = notstarted);
			pMyRobotState->Auto_Step = 0;
			pMyRobotState->Auto_State = 3;
			break;

		case 2:			// execute the code sequence with the pRogram pointer set to the seleted outer works defense program
			// break;	// case 2 and case 3 execute the same code sequence

		case 3:			// execute the code sequence with the pRogram pointer set to the seleted goal program
			AutoTime = 	int(AutonomousTimer.Get() * 1000);							// all timings expressed in milliseconds
			for (next_step = 0; next_step <= pMyRobotState->Auto_Step; next_step++)	// for each step in the program up to our current step
				{
				step = next_step;
				if (pRogram->Steps[step].Status < completed)		// check to see if the step is already completed
					{
					#ifdef REPORT_AUTO_DEBUG
						SmartDashboard::PutNumber("Autonomous Action",(int)(pRogram->Steps[step].action));
						SmartDashboard::PutNumber("Action Timeout",pRogram->Steps[step].Timer - AutoTime);
					#endif
					(this->*pRogram->Steps[step].function) ();
					}
				}		// at the end of the previous for loop, step is pointing to the next step to be executed

			/*
			 *  If the Steps[step].function was a (sucessful) Branch, we would have
			 *  	- Set the status flag to completed
			 *  	- Set pMyRobotState->Auto_Step to the step we are going to
			 *  	- Set all status flags completed between the current step and the pMyRobotState->Auto_Step (not including the pMyRobotState->Auto_Step)
			 *  	- Reset/Restart all status flags (to notstarted) from the pMyRobotState->Auto_Step forward to the end of the program
			 *  	- If branching forward, decrement pMyRobotState->Auto_Step
			 *  	- Set next_step to pMyRobotState->Auto_Step (to terminate the for loop)
			 *  If the Branch command failed the test, set the status flag to completed and move along to the next step
			 *
			 *  So, when we return from the ExecBranch method, step is pointing at the Branch command and we can just
			 *  increment pMyRobotState->Auto_Step, then cycle back and start running through the program steps again.
			 *
			 *
			 *  If the Steps[step].function was a CountedLoop, requiring the program to loop back to a prior step, we would
			 *  	- Set pMyRobotState->Auto_Step to the step we are going to
			 *  	- Reset/Restart all step status flags (to notstarted) from the pMyRobotState->Auto_Step forward to the end of the program
			 *  	- Decrement pMyRobotState->Auto_Step
			 *  	- Set next_step to pMyRobotState->Auto_Step (to terminate the for loop)
			 *
			 *  If the CountedLoop command completed the count, we just set the completed flag and move along to the next step
			 *
			 *  So, when we return from the ExecCountedLoop method, step is pointing at the CountedLoop command and we can just
			 *  increment pMyRobotState->Auto_Step, then cycle back and start running through the program steps again.
			 *
			 */
			if ((pRogram->Steps[step].action != Wait4) || (pRogram->Steps[step].Status >= completed))
				{
				if (pMyRobotState->Auto_Step < pRogram->NumberOfSteps - 1)	// are there more steps in this scenario?
					{
					pMyRobotState->Auto_Step++;								// go on to the next step
					}
				else if (pRogram->Steps[step].Status >= completed)			// the last step is complete or timed out
					{
					pMyRobotState->Auto_State++;							// we're done with this program when this flag is set
					pMyRobotState->Auto_Step = 0;							// start the next program at step 0
					pRogram = pGoalProgram[pMyRobotState->Auto_Goal - 1];	// set pRogram pointer to selected Goal program
//					SelectedScenario = pMyRobotState->Auto_Goal - 1;		// setup to run the selected Auto_Goal program
					}
				}
			break;

		case 4:
			// This means we are done with the autonomous program
			Stop();
			break;
		}
	}


void AUTOPILOT :: Stop()									// If we enter TeleOp before the autonomous tasks complete, stop all autonomous actions
	{
	#if DRIVECONTROL==TankDriveControl
		pMyInput->DriveL = 0;
		pMyInput->DriveR = 0;
	#elif DRIVECONTROL==ArcadeDriveControl
		pMyInput->DriveX = 0;
		pMyInput->DriveY = 0;
	#endif
	// restore Driver Preferences
	pMyRobotState->Drivetrain_Speed_Control = pMyTargetState->Drivetrain_Speed_Control = save_Speed_Control;
	pMyRobotState->Drivetrain_Spin_Control = pMyTargetState->Drivetrain_Spin_Control = save_Spin_Control;
	pMyRobotState->Drivetrain_Directional_Orientation = pMyTargetState->Drivetrain_Directional_Orientation = save_Directional_Orientation;
	// Restoring these values and then setting the buttons to false may be resulting in a switch
	// to the next state for each of these.  So, we should set the default state to one less
	// than the desired state???  But then, what if we don't run an Autonomous scenario first???
	pMyInput->SprintModeBtn = false;
	pMyInput->SpinControlBtn = false;
	#if DRIVETRAINTYPE==MecanumDriveTrain
		pMyInput->OrientModeBtn = false;
	#endif
	}

/*
 * For each of the autonomous commands below, we need to first determine the status.
 * if (completed !! timedout) return immediately.  This step is already done.
 *
 * if (notstarted), we need to add the current AutoTime value to the timeout specified
 * for the command and set the status to active.  Then, we need to determine the target
 * state of the robot and set appropriate values in the target state buffer.
 * Note:  if the timeout was specified as 0, there is no timeout feature, so add the
 * current AutoTime plus the AUTONOMOUS_PERIOD to the timeout value for this step.
 *
 * if (active) (including if we just started it), we need to determine if the timeout value
 * has been exceeded.  If so, set the status flag to timedout and stop the associated motor
 * controllers.  If not timedout, evaluate the current state buffer against the target
 * state buffer and determine what, if any, motor controllers (or other devices) need to
 * be modified.  If the current state buffer matches the target state buffer, set the
 * status flag to completed.
 */

void AUTOPILOT :: DisplaySteps(std::string command)
	{
	#ifdef REPORT_AUTO_DEBUG
		if (pMyRobotState->Auto_State == 2)
			{
			SmartDashboard::PutString("DefenseStep"+std::to_string(step+1), command);
			}
		else if (pMyRobotState->Auto_State == 3)
			{
			SmartDashboard::PutString("GoalStep"+std::to_string(step+1), command);
			}
	#endif
	}


void AUTOPILOT :: ExecSetControls()
	{
	DisplaySteps("SetControls");
	switch (pRogram->Steps[step].Status)
		{
		case notstarted:
			pRogram->Steps[step].Timer = pRogram->Steps[step].Timeout + AutoTime;
			pRogram->Steps[step].Status = active;
			pMyRobotState->Drivetrain_Speed_Control = pMyTargetState->Drivetrain_Speed_Control = (SpeedControl) pRogram->Steps[step].parameters.SetControls.SpeedControl;
			pMyRobotState->Drivetrain_Spin_Control = pMyTargetState->Drivetrain_Spin_Control = (pRogram->Steps[step].parameters.SetControls.SpinControl == 1);
			pMyRobotState->Drivetrain_Directional_Orientation = pMyTargetState->Drivetrain_Directional_Orientation = (DriveOrientation) pRogram->Steps[step].parameters.SetControls.Orientation;
//			break;		// no break here, we just started the step, so now we need to evaluate the operation

		case active:
			// there is nothing to do (so we are already done).
			pRogram->Steps[step].Status = completed;
			break;

		case completed:
			break;

		case timedout:
			break;
		}
	}


void AUTOPILOT :: ExecCountedLoop()
	{
	DisplaySteps("CountedLoop to "+std::to_string(pRogram->Steps[step].parameters.CountedLoop.NextLine + 1)
				+" Until "+std::to_string(pRogram->Steps[step].LoopCounter)
				+" > "+std::to_string(pRogram->Steps[step].parameters.CountedLoop.Final));

	switch (pRogram->Steps[step].Status)
		{
		case notstarted:
			pRogram->Steps[step].LoopCounter = pRogram->Steps[step].parameters.CountedLoop.Initial;
			pRogram->Steps[step].Status = active;
			//			break;		// no break here, we just started the step, so now we need to evaluate the operation

		case active:
			if (pRogram->Steps[step].LoopCounter > pRogram->Steps[step].parameters.CountedLoop.Final)
				{
				pRogram->Steps[step].Status = completed;
				}
			else
				{
				pRogram->Steps[step].LoopCounter += pRogram->Steps[step].parameters.CountedLoop.Increment;		// increment the LoopCounter
				/*
				 *  If the LoopCount is less than or equal to the Final value, we need to loop back to a prior step as follows:
				 *    	- Set pMyRobotState->Auto_Step to the step we are going to
				 *  	- Reset/Restart all step conditions (to notstarted) from the pMyRobotState->Auto_Step forward to, but not including this step
				 *  	- Decrement pMyRobotState->Auto_Step (Auto_Step will be incremented when the for loop completes)
				 *  	- Set next_step to pMyRobotState->Auto_Step (to terminate the for loop)
				 *
				 *  If the CountedLoop command completed the count, we just set the completed flag and move along to the next step
				 */
				pMyRobotState->Auto_Step = pRogram->Steps[step].parameters.CountedLoop.NextLine;
				for (int reset_step = pMyRobotState->Auto_Step; reset_step < step; reset_step++)	// reset all steps from here to the end of pRogram
					{
					pRogram->Steps[reset_step].Status = notstarted;
					}
				pMyRobotState->Auto_Step--;
				next_step = pMyRobotState->Auto_Step;
				}
			break;

		case completed:
			/*
			 * Once completed, we do not want to Loop Back (endless loop of completed steps).  So, if the NextLine is negative, we ignore it.
			 * If the NextLine is positive (skip some steps), then we will want to skip forward, but not reset any commands.
			 *
			 * On further review, it makes no sense for a CountedLoop to branch forward.  So, whenever the CountedLoop command completes,
			 * we will just fall through to the next step.  Note, I have not updated the code to sanity check that the NextLine represents a backward branch.
			 * The expectation is that the programmer will do it right or pay the consequences.
			 *
			 */
			break;

		case timedout:
			break;
		}
	}


void AUTOPILOT :: ExecBranch()
	{
	bool condition_met = false;
	int	var, val;
	DisplaySteps("Branch to "+std::to_string(pRogram->Steps[step].parameters.Branch.NextLine + 1)
				+" If "+std::to_string(int(*pRogram->Steps[step].parameters.Branch.pStateValue * 1000))
				+"<"+std::to_string(pRogram->Steps[step].parameters.Branch.Conditon)+">"
				+std::to_string(pRogram->Steps[step].parameters.Branch.Value));
	switch (pRogram->Steps[step].Status)
		{
		case notstarted:
			pRogram->Steps[step].Status = active;
			//break;	no break here -- go right into active/running

		case active:
			pRogram->Steps[step].Status = completed;		// Branch completes every time, but resets itself if it is a branch backwards.
			var = int(*pRogram->Steps[step].parameters.Branch.pStateValue * 1000);
//			var = int(pMyRobotState->Robot_Pitch * RADIANS_TO_DEGREES);
			val = pRogram->Steps[step].parameters.Branch.Value;
			switch (pRogram->Steps[step].parameters.Branch.Conditon)
				{
				case UN:		// Unconditional branch
					condition_met = true;
					break;

				case LT:		// LT
					condition_met = (var < val);
					break;

				case LE:		// LE
					condition_met = (var <= val);
					break;

				case EQ:		// EQ
					condition_met = (var == val);
					break;

				case GE:		// GE
					condition_met = (var >= val);
					break;

				case GT:		// GT
					condition_met = (var > val);
					break;
				}
			if (condition_met)
				{
				/*
	 			 *  If the condition is met, we need to set the current step to the target step as follows:
				 *  	- Set the condition flag to completed (flag gets set to completed every time and reset to notstarted on a backward branch)
				 *  	- Set pMyRobotState->Auto_Step to the step we are going to
				 *  	- Set all steps completed between the current step and the pMyRobotState->Auto_Step (not including the pMyRobotState->Auto_Step)
				 *  	- Reset/Restart all step conditions (to notstarted) from the pMyRobotState->Auto_Step forward to (and including) this step
				 *  	- Decrement pMyRobotState->Auto_Step (Auto_Step will be incremented when the for loop completes)
				 *  	- Set next_step to pMyRobotState->Auto_Step (to terminate the for loop)
				 *  If the Branch command failed the test, we just move along to the next step
				 */
//				pRogram->Steps[step].Status = completed;
				pMyRobotState->Auto_Step = pRogram->Steps[step].parameters.Branch.NextLine;
//				int step_increment = copysign(1, pMyRobotState->Auto_Step - step);
				for (int completed_step = step; completed_step < pMyRobotState->Auto_Step; completed_step++)
					{															// complete all forward steps between here and the target step
					pRogram->Steps[completed_step].Status = completed;
					}
				for (int reset_step = pMyRobotState->Auto_Step; reset_step <= step; reset_step++)
					{															// reset all backward steps between here and the target step
					pRogram->Steps[reset_step].Status = notstarted;
					}
//				if (step_increment > 0)
					pMyRobotState->Auto_Step--;
				next_step = pMyRobotState->Auto_Step;	// (terminate loop)
				}
			break;

		case completed:
			break;

		case timedout:
			break;
		}
	}



void AUTOPILOT :: ExecArrive()
	{
	switch (pRogram->Steps[step].Status)
		{
		case notstarted:
			pMyTargetState->Robot_Position_X = pRogram->Steps[step].parameters.Arrive.X;
			pMyTargetState->Robot_Position_Y = pRogram->Steps[step].parameters.Arrive.Y;
			if (pRogram->Steps[step].parameters.Arrive.AbsoluteOrRelative == 1)				// Relative Coordinates
				{																			// need to add starting coordinates
				pMyTargetState->Robot_Position_X += pRogram->Starting_X_Position;
				pMyTargetState->Robot_Position_Y += pRogram->Starting_Y_Position;
				}
//			pMyTargetState->Robot_Heading = pMyAutoTarget->Robot_Heading;
			pRogram->Steps[step].Status = active;
			//			break;		// no break here, we just started the step, so now we need to evaluate the operation

		case active:
			// there is a potential race condition where the Navigator could override this position update
			DisplaySteps("Arrive("+std::to_string(pMyTargetState->Robot_Position_X)
								+","+std::to_string(pMyTargetState->Robot_Position_Y)
								+")");
			pMyRobotState->Robot_Position_X = pMyTargetState->Robot_Position_X;
			pMyRobotState->Robot_Position_Y = pMyTargetState->Robot_Position_Y;
//			pMyRobotState->Robot_Heading = pMyTargetState->Robot_Heading;
			pRogram->Steps[step].Status = completed;
			break;

		case completed:
			break;

		case timedout:
			break;
		}
	}


void AUTOPILOT :: ExecWait4()
	{
	DisplaySteps("Wait4 "+std::to_string(pRogram->Steps[step].Timer - AutoTime)+" || pRogram->Steps["
						 +std::to_string(pRogram->Steps[step].parameters.Wait4.Wait4Line + 1)+"].Status");

	switch (pRogram->Steps[step].Status)
		{
		case notstarted:
			pRogram->Steps[step].Timer = pRogram->Steps[step].Timeout + AutoTime;
			pRogram->Steps[step].Status = active;
			// there is no change to the target state buffer.
			//			break;		// no break here, we just started the step, so now we need to evaluate the operation

		case active:
			// Copy the pRogram step status to this status
			pRogram->Steps[step].Status = pRogram->Steps[pRogram->Steps[step].parameters.Wait4.Wait4Line].Status;
			// Regardless of any above Status updates, check for timeout condition
			if (pRogram->Steps[step].Timer <= AutoTime) pRogram->Steps[step].Status = timedout;
			break;

		case completed:
			break;

		case timedout:
			break;
		}
	}


void AUTOPILOT :: MoveToPosition()
	{
	float	xdiff, ydiff, direction, fwd_angular_difference, bkwd_angular_difference, eta;
//	float	xpower = 0;
	float	ypower = (pMyInput->DriveL + pMyInput->DriveR) * pMyRobotState->Drivetrain_SprintFactor / 0.02;			// initialize to average current input setting * 100
//	float brakepower, scale_speed_to_distance, joystick_angle, scale_to_normalize;

	#if DRIVECONTROL==TankDriveControl
		pMyInput->DriveL = 0;
		pMyInput->DriveR = 0;
	#elif DRIVECONTROL==ArcadeDriveControl
		pMyInput->DriveX = 0;
		pMyInput->DriveY = 0;
	#endif
	if (pRogram->Steps[step].Timer <= AutoTime)
		{
		pRogram->Steps[step].Status = timedout;
		return;
		}
	// First compute the distance that I need to travel.
	xdiff = pMyTargetState->Robot_Position_X - pMyRobotState->Robot_Position_X;
	ydiff = pMyTargetState->Robot_Position_Y - pMyRobotState->Robot_Position_Y;
	distance = fabs(pow(pow(xdiff,2)+pow(ydiff,2),0.5));
	if ((fabs(distance) < 1) && (pMyRobotState->Robot_Speed < 1))		// If I'm within 1 inch of the Target and my speed is near zero, I'm done
		{
		pRogram->Steps[step].Status = completed;
		return;
		}
	// Next, compute the direction that I need to travel.
	direction = (HALF_PI - atan2(ydiff, xdiff));	// compass direction is given in radians
	if (direction < 0) direction += TWO_PI;			// normalize direction between 0 and TWO_PI (000 degrees to 359.999 degrees)

	#if DRIVETRAINTYPE==TankDriveTrain
		// If the direction is significantly different (more than 10 degrees) than the current robot heading
		// we will need to turn toward the target location before we start moving
		// Note, we also have the option of turning away from the target location and backing up
		// This is critical to recognize, especially if we accidentally overshoot our approach to the target location
		fwd_angular_difference = direction - pMyRobotState->Robot_Heading;				// between -TWO_PI and +TWO_PI
		if (fwd_angular_difference > PI) fwd_angular_difference -= TWO_PI;
		else if (fwd_angular_difference < -1 * PI) fwd_angular_difference += TWO_PI;	// always turn the shortest direction
		bkwd_angular_difference = fwd_angular_difference + PI;
		if (bkwd_angular_difference > PI) bkwd_angular_difference -= TWO_PI;			// turn the other direction to go backwards
		// Now decide which direction (forward or backward) gives us the smallest turn
		if (fabs(bkwd_angular_difference) < fabs(fwd_angular_difference))
			{
			distance *= -1.0;
			pMyRobotState->Drivetrain_DriftAngle = bkwd_angular_difference;
			direction = (direction + PI);
			if (direction > PI) direction -= TWO_PI;			// turn the other direction to go backwards
			}
		else
			{
			pMyRobotState->Drivetrain_DriftAngle = fwd_angular_difference;
			}
		pMyTargetState->Robot_Heading = pMyTargetState->Robot_Direction = direction;
		// Anything less than 10 degree variance, we will let SpinControl correct as we move toward our destination
		if (fabs(pMyRobotState->Drivetrain_DriftAngle) > (10 * DEGREES_TO_RADIANS))
			{
			pMyTargetState->Robot_Spin_Rate = fmax(pMyRobotState->Drivetrain_DriftAngle, 1);	// not less than 1 radian per second, preferably complete turn in 1 sec or less
			TurnToHeading();
			return;
			}
	#elif DRIVETRAINTYPE==MecanumDriveTrain
		pMyTargetState->Robot_Direction = direction;						// no need to turn; just head in the desired direction
		/*
		 * this would be the place to compute the pMyInput->DriveX and pMyInput->DriveY values
		 * Then modify these values below as we approach the target (or overshoot the target)
		 */
	#endif

	// Next, we need to compute the amount of power to apply to move to the target position and stop upon arrival
	// If we are closing on the target position fast, we need to reverse power (brake) to avoid overrunning
	// Otherwise, we want to add 10% of the max power, up to max power setting

	eta = distance / copysign(fmax(1, fabs(pMyRobotState->Robot_Speed)), pMyRobotState->Robot_Speed);	// number of seconds to arrival at current speed
	// If the eta is less than 0, we overshot the target (or the target moved) and the Robot is going the wrong way.
	// So, the lower the eta, the more we want to reduce the Drive power
	// Note that, if we reduce the power such that it changes polarity, but the robot momentum continues to carry us away from the target,
	// then we need to be very careful about what it means to reduce the Drive power again


	#ifdef REPORT_AUTO_DEBUG
		SmartDashboard::PutNumber("distance", distance);
		SmartDashboard::PutNumber("direction", direction);
		SmartDashboard::PutNumber("eta", eta);
	#endif

	if (eta < 0)												// Oops!, we overshot the target.  The robot is moving away from the target
		{														// Apply power in the direction toward the target
		#if DRIVECONTROL==TankDriveControl
			ypower += copysign(fmin(MAXIMUM_ACCELERATION, fabs(distance)), distance);
		#elif DRIVECONTROL==ArcadeDriveControl
		#endif
		}
	else if (eta < 1)											// Hit the brakes!
		{														// Braking power is proportional to Robot_Speed with the direction reversed
		#if DRIVECONTROL==TankDriveControl
			ypower = 0 - copysign(fmin(MAXIMUM_ACCELERATION, fabs(pMyRobotState->Robot_Speed)), distance);
		#elif DRIVECONTROL==ArcadeDriveControl
		#endif
		}
	else if (eta < 2)											// if arrival is less than 2 seconds away, let off the gas
		{
		#if DRIVECONTROL==TankDriveControl
			ypower -= copysign(fmin(MAXIMUM_ACCELERATION, fabs(pMyRobotState->Robot_Speed)), distance);
		#elif DRIVECONTROL==ArcadeDriveControl
		#endif
		}
	else if (fabs(pMyRobotState->Robot_Speed) < fabs(pMyTargetState->Robot_Speed))		// accelerate up to the maximum prescribed Robot_Speed
		{
		#if DRIVECONTROL==TankDriveControl
			ypower += copysign(fmin(MAXIMUM_ACCELERATION, fabs(distance)), distance);
		#elif DRIVECONTROL==ArcadeDriveControl
		#endif
		}
	else if (fabs(pMyRobotState->Robot_Speed) > fabs(pMyTargetState->Robot_Speed))		// speeding;  need to let off the gas a bit
		{
		#if DRIVECONTROL==TankDriveControl
			ypower -= copysign(fmin(MAXIMUM_ACCELERATION, fabs(distance)) / 10, distance);
		#elif DRIVECONTROL==ArcadeDriveControl
		#endif
		}

	#if DRIVECONTROL==TankDriveControl
		ypower = (fabs(ypower) > 1.0) ? copysign(fmax(ypower, MINIMUM_DRIVE_POWER), distance) : 0;
		pMyInput->DriveL = ypower * (INVERTLEFTDRIVEMOTORPOWER ? -0.01 : 0.01) / pMyRobotState->Drivetrain_SprintFactor;
		pMyInput->DriveR = ypower * (INVERTRIGHTDRIVEMOTORPOWER ? -0.01 : 0.01) / pMyRobotState->Drivetrain_SprintFactor;
	#elif DRIVECONTROL==ArcadeDriveControl
	#endif

	/*
		brakepower = fmin(2, fmax(-0.25, 3 - (distance / (pMyRobotState->Robot_Speed + 1)))) * scale_speed_to_distance;
		switch (int(pMyRobotState->Robot_Direction))
			{
			case 0 ... 89:
				if ((xdiff < 0) || (ydiff < 0)) brakepower = 0;		// overran the target, so turn off braking (counter-intuitive)
				break;

			case 90 ... 179:
				if ((xdiff < 0) || (ydiff > 0)) brakepower = 0;		// overran the target, so turn off braking (counter-intuitive)
				break;

			case 180 ... 269:
				if ((xdiff > 0) || (ydiff > 0)) brakepower = 0;		// overran the target, so turn off braking (counter-intuitive)
				break;

			case 270 ... 360:
				if ((xdiff > 0) || (ydiff < 0)) brakepower = 0;		// overran the target, so turn off braking (counter-intuitive)
				break;
			}
		scale_speed_to_distance -= brakepower;

		switch (pMyRobotState->Drivetrain_Directional_Orientation)
			{
			case Field:
				xpower = xdiff * scale_speed_to_distance / fmax(1, distance);
				ypower = ydiff * scale_speed_to_distance / fmax(1, distance);
				break;

			case Robot:
				// need to calculate the direction of intended movement relative to the robot's heading
				// and then figure out how much xpower and ypower to apply to travel in the intended direction
				joystick_angle = ((90 + pMyRobotState->Robot_Heading) * DEGREES_TO_RADIANS) - atan2(xdiff,ydiff);
				xpower = cos(joystick_angle) * scale_speed_to_distance;
				ypower = sin(joystick_angle) * scale_speed_to_distance;
				break;
			}

		// Either xpower or ypower could exceed 1.  But, the motor controller will only put out 100%.
		// So that means both power settings will need to be reduced by whatever factor brings the largest
		// power setting back within the -100% to +100% range.  This will ensure that the robot travels
		// in the intended direction.
		scale_to_normalize = fmax(1, fmax(fabs(xpower), fabs(ypower)));
		xpower /= scale_to_normalize;
		ypower /= scale_to_normalize;

		// Input the power control joystick settings (adjusted for speed control and Y-axis inversion)
		#if DRIVETRAINTYPE==TankDriveTrain
			pMyInput->DriveL = ypower * -1 * INVERT_Y_AXIS_INPUT;
			pMyInput->DriveR = ypower * -1 * INVERT_Y_AXIS_INPUT;
		#elif DRIVETRAINTYPE==MecanumDriveTrain
			pMyInput->DriveX = xpower;
			pMyInput->DriveY = ypower * -1 * INVERT_Y_AXIS_INPUT;
		#endif

		}
	*/
	}


void AUTOPILOT :: ExecMoveTo()
	{
	switch (pRogram->Steps[step].Status)
		{
		case notstarted:
			pRogram->Steps[step].Timer = pRogram->Steps[step].Timeout + AutoTime;
			pRogram->Steps[step].Status = active;
			pMyTargetState->Robot_Position_X = pRogram->Steps[step].parameters.MoveTo.Xcoordinate;
			pMyTargetState->Robot_Position_Y = pRogram->Steps[step].parameters.MoveTo.Ycoordinate;
			pMyTargetState->Robot_Speed = pRogram->Steps[step].parameters.MoveTo.MaxSpeed;
			//			break;		// no break here, we just started the step, so now we need to evaluate the operation

		case active:
			DisplaySteps("MoveTo ("+std::to_string(pMyTargetState->Robot_Position_X)
						+","+std::to_string(pMyTargetState->Robot_Position_Y)
						+")");
			MoveToPosition();
			break;

		case completed:
			break;

		case timedout:
			break;
		}
	}


void AUTOPILOT :: ExecMoveRelative()
	{
	switch ((int)pRogram->Steps[step].Status)
		{
		case notstarted:
			pRogram->Steps[step].Timer = pRogram->Steps[step].Timeout + AutoTime;
			pRogram->Steps[step].Status = active;
			pMyTargetState->Robot_Position_X += pRogram->Steps[step].parameters.MoveRelative.Xdistance;
			pMyTargetState->Robot_Position_Y += pRogram->Steps[step].parameters.MoveRelative.Ydistance;
			pMyTargetState->Robot_Speed = pRogram->Steps[step].parameters.MoveRelative.MaxSpeed;
//			break;		// no break here, we just started the step, so now we need to evaluate the operation

		case active:
			DisplaySteps("MoveRelative to ("+std::to_string(pMyTargetState->Robot_Position_X)
						+","+std::to_string(pMyTargetState->Robot_Position_Y)
						+")");
			MoveToPosition();
			break;

		case completed:
			break;

		case timedout:
			break;

		}
	}


void AUTOPILOT :: TurnToHeading()
	{
	if (pRogram->Steps[step].Timer <= AutoTime)
		{
		pRogram->Steps[step].Status = timedout;
		pMyInput->DriveL = 0;
		pMyInput->DriveR = 0;
		}
	else if ((fabs(pMyTargetState->Robot_Heading - pMyRobotState->Robot_Heading) < (0.25 * DEGREES_TO_RADIANS))
			&& (fabs(pMyRobotState->Robot_Spin_Rate) < DEGREES_TO_RADIANS))
		{											// Done when heading is within .25 degrees and spin rate is less than 1 degree per second
		pRogram->Steps[step].Status = completed;
		pMyInput->DriveR = 0;
		}
	else if (! pMyTargetState->Drivetrain_Spin_Control)		// If Spin_Control is enabled, we will just let the Drivetrain code spin the bot to the correct heading
		{													// Otherwise, we use the same method to calculate the spin_correction value
		pMyRobotState->Drivetrain_SpinCorrection = SpinCalc();

		pMyInput->DriveL += (0.5 * pMyRobotState->Drivetrain_SpinCorrection);
		pMyInput->DriveR -= (0.5 * pMyRobotState->Drivetrain_SpinCorrection);
		// Now compensate for an imbalance between left and right drive effeciencies
		pMyInput->DriveL = (pMyInput->DriveL + DRIVEPOWERBALANCEOFFSET) * DRIVEPOWERBALANCEFACTOR;
		// Now scale L and R so neither is greater than 100%
		double scalar = 1 / fmax(1, (fmax (fabs (pMyInput->DriveL), fabs (pMyInput->DriveR))));
		pMyInput->DriveL *= scalar;
		pMyInput->DriveR *= scalar;
		}
	}


void AUTOPILOT :: ExecTurnTo()
	{
	DisplaySteps("TurnTo "+std::to_string(pRogram->Steps[step].parameters.TurnTo.Heading));
	switch (pRogram->Steps[step].Status)
		{
		case notstarted:
			pRogram->Steps[step].Timer = pRogram->Steps[step].Timeout + AutoTime;
			pRogram->Steps[step].Status = active;
			pMyTargetState->Robot_Heading = pRogram->Steps[step].parameters.TurnTo.Heading;
			pMyTargetState->Robot_Spin_Rate = pRogram->Steps[step].parameters.TurnTo.MaxRate;
//			SmartDashboard::PutNumber("Target_Heading",pMyTargetState->Robot_Heading);
//			SmartDashboard::PutNumber("Target_Spin_Rate",pMyTargetState->Robot_Spin_Rate);
			/*
			 * Note:  An alternative approach is to enable the SpinControl feature in the Drivetrain and
			 * just set the pMyRobotState->Drivetrain_Heading to the desired target heading.
			 */
//			break;		// no break here, we just started the step, so now we need to evaluate the operation

		case active:
			TurnToHeading();
			break;

		case completed:
			break;

		case timedout:
			break;
		}
	}


void AUTOPILOT :: ExecTurnRelative()
	{
//	DisplaySteps("TurnRelative");
	DisplaySteps("TurnRelative to ("+std::to_string(pMyTargetState->Robot_Heading + pRogram->Steps[step].parameters.TurnRelative.Degrees));
	switch (pRogram->Steps[step].Status)
		{
		case notstarted:
			pRogram->Steps[step].Timer = pRogram->Steps[step].Timeout + AutoTime;
			pRogram->Steps[step].Status = active;
			pMyTargetState->Robot_Heading = (double) (int(1000 * (pMyTargetState->Robot_Heading + pRogram->Steps[step].parameters.TurnRelative.Degrees + 360)) % 360000) / 1000.000;
			pMyTargetState->Robot_Spin_Rate = pRogram->Steps[step].parameters.TurnRelative.MaxRate;
			SmartDashboard::PutNumber("Target_Heading",pMyTargetState->Robot_Heading);
			SmartDashboard::PutNumber("Target_Spin_Rate",pMyTargetState->Robot_Spin_Rate);
//			break;		// no break here, we just started the step, so now we need to evaluate the operation

		case active:
			TurnToHeading();
			break;

		case completed:
			break;

		case timedout:
			break;
		}
	}


void AUTOPILOT :: ExecShooter()
	{
	DisplaySteps("Shooter");
	switch (pRogram->Steps[step].Status)
		{
		case notstarted:
			pRogram->Steps[step].Timer = pRogram->Steps[step].Timeout + AutoTime;
			pRogram->Steps[step].Status = active;
			pMyTargetState->Shooter_Launch_Power = pRogram->Steps[step].parameters.Shooter.LaunchPower;
			pMyTargetState->Shooter_Boulder_Position = pRogram->Steps[step].parameters.Shooter.BoulderPosition;
			//			break;		// no break here, we just started the step, so now we need to evaluate the operation

		case active:
			if (pRogram->Steps[step].Timer <= AutoTime)
				{
				pRogram->Steps[step].Status = timedout;
				pMyTargetState->Shooter_Launch_Power = 0;
				pMyTargetState->Shooter_Boulder_Position = BOULDER_HOLD;
				}
			if ((fabs(pMyRobotState->Shooter_Launch_Power - pRogram->Steps[step].parameters.Shooter.LaunchPower) < 20)
				&& (pMyRobotState->Shooter_Boulder_Position == pRogram->Steps[step].parameters.Shooter.BoulderPosition))
				pRogram->Steps[step].Status = completed;
			break;

		case completed:
			break;

		case timedout:
			break;
		}
	}


void AUTOPILOT :: ExecScooper()
	{
	DisplaySteps("Scooper -> "+std::to_string(pRogram->Steps[step].parameters.Scooper.Elevation)+" @ "+std::to_string(pRogram->Steps[step].parameters.Scooper.ElevatorSpeed));
	switch (pRogram->Steps[step].Status)
		{
		case notstarted:
			pRogram->Steps[step].Timer = pRogram->Steps[step].Timeout + AutoTime;
			pRogram->Steps[step].Status = active;
			pMyTargetState->Shooter_Elevation_Speed = pRogram->Steps[step].parameters.Scooper.ElevatorSpeed;
			//			break;		// no break here, we just started the step, so now we need to evaluate the operation

		case active:
			if (pMyTargetState->Shooter_Elevation == pRogram->Steps[step].parameters.Scooper.Elevation)
				{
				pRogram->Steps[step].Status = completed;
				}
			else if (pRogram->Steps[step].Timer <= AutoTime)
				{
				pRogram->Steps[step].Status = timedout;
				pMyTargetState->Shooter_Elevation = pRogram->Steps[step].parameters.Scooper.Elevation;
				}
			else
				{
				pMyTargetState->Shooter_Elevation +=
						copysign(	fmin(	SCOOPER_MAX_DEGREES_PER_CYCLE,
											fabs(pRogram->Steps[step].parameters.Scooper.Elevation - pMyTargetState->Shooter_Elevation)),
									pRogram->Steps[step].parameters.Scooper.Elevation - pMyTargetState->Shooter_Elevation);
				}


			/*
			 * 			elevator_left_up_power += (SCOOPER_MAX_DEGREES_PER_CYCLE - (((double) left_delta) / SCOOPER_ENC_TICKS_PER_DEGREE));		// increase elevator_down_power by the amount of lag
			 *
			 */

			if (pRogram->Steps[step].Timer <= AutoTime)
				{
				pMyTargetState->Shooter_Elevation = pMyRobotState->Shooter_Elevation;
				}
			break;

		case completed:
			break;

		case timedout:
			break;
		}
	}


void AUTOPILOT :: ExecClimber()
	{
	DisplaySteps("Climber");
	/*
	switch (pRogram->Steps[step].Status)
		{
		case notstarted:
			pRogram->Steps[step].Timer = pRogram->Steps[step].Timeout + AutoTime;
			pRogram->Steps[step].Status = active;
			// there is no change to the target state buffer (kindof what NoOp means).
//			break;		// no break here, we just started the step, so now we need to evaluate the operation

		case active:
			if (pRogram->Steps[step].Timer <= AutoTime)
				{
				pRogram->Steps[step].Status = timedout;
				// no associated motor controllers to stop
				}
			// there are no target state buffer completion criteria (kindof what NoOp means).
			break;

		case completed:
			break;

		case timedout:
			break;
		}
	*/
	}




