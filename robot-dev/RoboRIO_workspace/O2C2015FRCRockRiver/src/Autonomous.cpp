/*
 * Autonomous.cpp
 *
 *  Created on: Jan 31, 2015
 *      Author: Rod
 */

#include <Autonomous.h>


AUTONOMOUS :: AUTONOMOUS()
	{
	SelectedScenario = 0;
	step = 0;
	action = NoOp;
	autostep_index = 0;
	pScenario = new Scenario_t;
	AutonomousTimer.Start();
	AutoTime = ElapsedTime = 0;
	timeout = 0;
	save_Speed_Control = Normal;
	save_Spin_Control = true;
	save_Directional_Orientation = Robot;
	prev_distance = distance = 0;
	pMyAutoTarget = 0;
	}


void AUTONOMOUS :: Init(RobotStateBuffer *pRobotState, INPUT *pInput, RobotStateBuffer *pAutoTarget)
	// pMyRobotState->Autonomous_Scenario tells us which program we will be running
	{
	SmartDashboard::PutNumber("AutoInit", 1);
	pMyRobotState = pRobotState;
	pMyInput = pInput;
	pMyAutoTarget = pAutoTarget;
	for (int i = 0; i < AUTONOMOUS_MAX_STEPS; CompletedTasks[i++] = notstarted);
	pMyRobotState->Autonomous_Scenario = -1;
	SelectScenario();
	// save Driver Preferences
	save_Speed_Control = pMyRobotState->Drivetrain_Speed_Control;
	save_Spin_Control = pMyRobotState->Drivetrain_Spin_Control;
	save_Directional_Orientation = pMyRobotState->Drivetrain_Directional_Orientation;
	pMyRobotState->Autonomous_Step = 0;
	pMyRobotState->Autonomous_State = 0;
	SmartDashboard::PutNumber("AutoInit", 2);
	}


void AUTONOMOUS :: SelectScenario()
	{
	int	starting_index = 0;

	SelectedScenario = int(SmartDashboard::GetNumber("Autonomous Scenario",AUTO_DEFAULT_SCENARIO));
	if (SelectedScenario != pMyRobotState->Autonomous_Scenario)		// a new Scenario has been selected
		{	// Need to load in the details required to run the new scenario
		pMyRobotState->Autonomous_State = 0;
		for (int s = 0; s < SelectedScenario; s++)
			{
			starting_index += AutonomousSteps[starting_index++][1];
			}
		pScenario->ScenarioName = (AutoScenario)AutonomousSteps[starting_index][0];
		pScenario->NumberOfSteps = AutonomousSteps[starting_index][1];
		pMyRobotState->Robot_Position_X = pScenario->Starting_X_Position = AutonomousSteps[starting_index][2];
		pMyRobotState->Robot_Position_Y = pScenario->Starting_Y_Position = AutonomousSteps[starting_index][3];
		pMyRobotState->Robot_Heading = pScenario->Starting_Heading = AutonomousSteps[starting_index][4];
		pMyRobotState->Drivetrain_Heading = pMyRobotState->Robot_Direction = pMyRobotState->Robot_Heading;
		pMyAutoTarget->Robot_Position_X = pMyRobotState->Robot_Position_X;
		pMyAutoTarget->Robot_Position_Y = pMyRobotState->Robot_Position_Y;
		pMyAutoTarget->Robot_Heading = pMyRobotState->Robot_Heading;
		pMyAutoTarget->Robot_Speed = 0;
		SmartDashboard::PutNumber("Start_Heading", pScenario->Starting_Heading);
		SmartDashboard::PutNumber("Start_X", pScenario->Starting_X_Position);
		SmartDashboard::PutNumber("Start_Y", pScenario->Starting_Y_Position);
		SmartDashboard::PutNumber("Autonomous Steps", pScenario->NumberOfSteps);
		for (step = 0; step < pScenario->NumberOfSteps; step++)
			{
			autostep_index = step + starting_index + 1;
			action = (AutoAction)AutonomousSteps[autostep_index][0];
			SmartDashboard::PutNumber("Autonomous Action",(int)(action));
			pScenario->Steps[step].action = action;
			pScenario->Steps[step].TaskIndex = AutonomousSteps[autostep_index][1];
			pScenario->Steps[step].Timeout = AutonomousSteps[autostep_index][2];
			SmartDashboard::PutNumber("Action Timeout",pScenario->Steps[step].Timeout - AutoTime);
			switch (int(action))
				{
				case NoOp:
					pScenario->Steps[step].function = &AUTONOMOUS::ExecNoOp;
					SmartDashboard::PutString("Autonomous Command", "NoOp");
					break;

				case Arrive:
					pScenario->Steps[step].function = &AUTONOMOUS::ExecArrive;
					SmartDashboard::PutString("Autonomous Command", "Arrive");
					break;

				case SetControls:
					pScenario->Steps[step].function = &AUTONOMOUS::ExecSetControls;
					pScenario->Steps[step].parameters.SetControls.SpeedControl = AutonomousSteps[autostep_index][3];
					pScenario->Steps[step].parameters.SetControls.SpinControl = AutonomousSteps[autostep_index][4];
					pScenario->Steps[step].parameters.SetControls.Orientation = AutonomousSteps[autostep_index][5];
					SmartDashboard::PutString("Autonomous Command", "SetControls");
					break;

				case Wait4:
					pScenario->Steps[step].function = &AUTONOMOUS::ExecWait4;
					SmartDashboard::PutString("Autonomous Command", "Wait4");
					break;

				case MoveTo:
					pScenario->Steps[step].function = &AUTONOMOUS::ExecMoveTo;
					pScenario->Steps[step].parameters.MoveTo.Xcoordinate = AutonomousSteps[autostep_index][3];
					pScenario->Steps[step].parameters.MoveTo.Ycoordinate = AutonomousSteps[autostep_index][4];
					pScenario->Steps[step].parameters.MoveTo.Power = AutonomousSteps[autostep_index][5];
					SmartDashboard::PutString("Autonomous Command", "MoveTo");
					break;

				case MoveRelative:
					pScenario->Steps[step].function = &AUTONOMOUS::ExecMoveRelative;
					pScenario->Steps[step].parameters.MoveRelative.Xdistance = AutonomousSteps[autostep_index][3];
					pScenario->Steps[step].parameters.MoveRelative.Ydistance = AutonomousSteps[autostep_index][4];
					pScenario->Steps[step].parameters.MoveRelative.Power = AutonomousSteps[autostep_index][5];
					SmartDashboard::PutString("Autonomous Command", "MoveRelative");
					break;

				case TurnTo:
					pScenario->Steps[step].function = &AUTONOMOUS::ExecTurnTo;
					pScenario->Steps[step].parameters.TurnTo.Heading = AutonomousSteps[autostep_index][3];
					pScenario->Steps[step].parameters.TurnTo.Power = AutonomousSteps[autostep_index][4];
					SmartDashboard::PutString("Autonomous Command", "TurnTo");
					break;

				case TurnRelative:
					pScenario->Steps[step].function = &AUTONOMOUS::ExecTurnRelative;
					pScenario->Steps[step].parameters.TurnRelative.Degrees = AutonomousSteps[autostep_index][3];
					pScenario->Steps[step].parameters.TurnRelative.Power = AutonomousSteps[autostep_index][4];
					SmartDashboard::PutString("Autonomous Command", "TurnRelative");
					break;

				case ToteWheels:
					pScenario->Steps[step].function = &AUTONOMOUS::ExecToteWheels;
					pScenario->Steps[step].parameters.ToteWheels.LeftPower = AutonomousSteps[autostep_index][3];
					pScenario->Steps[step].parameters.ToteWheels.RightPower = AutonomousSteps[autostep_index][4];
					SmartDashboard::PutString("Autonomous Command", "ToteWheels");
					break;

				case ToteArms:
					pScenario->Steps[step].function = &AUTONOMOUS::ExecToteArms;
					pScenario->Steps[step].parameters.ToteArms.WheelPosition = AutonomousSteps[autostep_index][3];
					pScenario->Steps[step].parameters.ToteArms.DutyCycle = AutonomousSteps[autostep_index][4];
					SmartDashboard::PutString("Autonomous Command", "ToteArms");
					break;

				case ToteLift:
					pScenario->Steps[step].function = &AUTONOMOUS::ExecToteLift;
					pScenario->Steps[step].parameters.ToteLift.Height = AutonomousSteps[autostep_index][3];
					pScenario->Steps[step].parameters.ToteLift.JoyMode = AutonomousSteps[autostep_index][4];
					pScenario->Steps[step].parameters.ToteLift.Power = AutonomousSteps[autostep_index][5];
					SmartDashboard::PutString("Autonomous Command", "ToteLift");
					break;
/*
 * 			{ToteLift, 1, 1500, LIFT_TOTE_UP, LIFT_JOY_OFF, LIFT_UP_POWER},
 *
 */
/*
 * ClawArm and Pincer are abandoned subsystems
				case ClawArm:
					pScenario->Steps[step].function = &AUTONOMOUS::ExecClawArm;
					pScenario->Steps[step].parameters.ClawArm.Height = AutonomousSteps[autostep_index][3];
					pScenario->Steps[step].parameters.ClawArm.Reach = AutonomousSteps[autostep_index][4];
					pScenario->Steps[step].parameters.ClawArm.Power = AutonomousSteps[autostep_index][5];
					SmartDashboard::PutString("Autonomous Command", "ClawArm");
					break;

				case Pincer:
					pScenario->Steps[step].function = &AUTONOMOUS::ExecPincer;
					pScenario->Steps[step].parameters.Pincer.Position = AutonomousSteps[autostep_index][3];
					pScenario->Steps[step].parameters.Pincer.Power = AutonomousSteps[autostep_index][4];
					SmartDashboard::PutString("Autonomous Command", "Pincer");
					break;
 */
				}
//			Wait (.5);	// watch the scenario steps loading
			}
		}
	pMyRobotState->Autonomous_Scenario = SelectedScenario;
	pMyRobotState->Autonomous_Step = 0;
	pMyRobotState->Autonomous_State = 1;
	}


void AUTONOMOUS :: Start()
	{
	AutonomousTimer.Reset();
	pMyRobotState->Autonomous_Step = 0;
	CompletedTasks[0] = notstarted;
	// Set Autonomous Preferences -- will restore when Autonomous completes
	pMyRobotState->Drivetrain_Speed_Control = Speedy;	// motor = pwr * Drivetrain_Speed_Control / 4
	pMyRobotState->Drivetrain_Spin_Control = true;	// enabled/disabled by gamepad button input
	pMyRobotState->Drivetrain_Directional_Orientation = Field;
	pMyInput->SprintModeBtn = false;
	pMyInput->OrientModeBtn = true;
	pMyInput->SpinControlBtn = true;
	pMyRobotState->Robot_Position_X = pMyAutoTarget->Robot_Position_X;
	pMyRobotState->Robot_Position_Y = pMyAutoTarget->Robot_Position_Y;
	pMyRobotState->Robot_Heading = pMyRobotState->Drivetrain_Heading = pMyAutoTarget->Robot_Heading;
	pMyRobotState->Robot_Direction = 0;
	pMyRobotState->Robot_Speed = 0;
	pMyRobotState->Autonomous_State = 2;
	}

void AUTONOMOUS :: RunAuto()									// Execute all autonomous tasks in the defined scenario
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

	if (CompletedTasks[0] >= completed)		// if we are all done, no need to check anything, just return
		{
		return;	// we're done when this flag is set
		}
	AutoTime = 	int(AutonomousTimer.Get() * 1000);		// all timings expressed in milliseconds
	SmartDashboard::PutNumber("Autonomous Step",pMyRobotState->Autonomous_Step);
	for (step = 0; step <= pMyRobotState->Autonomous_Step; step++)
		{
		if (CompletedTasks[pScenario->Steps[step].TaskIndex] < completed)
			{
			SmartDashboard::PutNumber("Autonomous Action",(int)(pScenario->Steps[step].action));
			SmartDashboard::PutNumber("Action Timeout",pScenario->Steps[step].Timeout - AutoTime);
			(this->*pScenario->Steps[step].function) ();
			}
		}		// at the end of the previous for loop, step is pointing to the next step to be executed
	step--;		// but we need to check the current step to determine if it is appropriate to start the next step
	if ((pScenario->Steps[step].action != Wait4)
			|| (CompletedTasks[pScenario->Steps[step].TaskIndex] >= completed)
			|| (AutoTime >= pScenario->Steps[step].Timeout))
		{
		if (pMyRobotState->Autonomous_Step < pScenario->NumberOfSteps - 1)	// are there more steps in this scenario?
			{
			pMyRobotState->Autonomous_Step++;
			pScenario->Steps[pMyRobotState->Autonomous_Step].Timeout += AutoTime;
			}
		else if ((CompletedTasks[pScenario->Steps[step].TaskIndex] >= completed)	// the last step is complete
				|| (pScenario->Steps[step].Timeout <= AutoTime))					// or timed out
			{
			Stop();
			}
		}
	if (pMyRobotState->Autonomous_Step < pScenario->NumberOfSteps - 1)	// are there more steps in this scenario?
		{
		CompletedTasks[0] = notstarted;		// don't count setting Task 0 completed until it's set by the last step
		}
	}


void AUTONOMOUS :: Stop()									// If we enter TeleOp before the autonomous tasks complete, stop all autonomous actions
	{
	pMyInput->DriveX = 0;
	pMyInput->DriveY = 0;
	pMyInput->DriveR = 0;
	SmartDashboard::PutNumber("AutoState",3);
	CompletedTasks[0] = completed;
	// restore Driver Preferences
	pMyRobotState->Drivetrain_Speed_Control = save_Speed_Control;
	pMyRobotState->Drivetrain_Spin_Control = save_Spin_Control;
	pMyRobotState->Drivetrain_Directional_Orientation = save_Directional_Orientation;
	// Restoring these values and then setting the buttons to false may be resulting in a switch
	// to the next state for each of these.  So, we should set the default state to one less
	// than the desired state???  But then, what if we don't run an Autonomous scenario first???
	pMyInput->SprintModeBtn = false;
	pMyInput->OrientModeBtn = false;
	pMyInput->SpinControlBtn = false;
	//	Need to update the Staring Position and Heading for TeleOp to wherever we finished with Autonomous
	SmartDashboard::PutNumber("Start_X", pMyRobotState->Robot_Position_X);
	SmartDashboard::PutNumber("Start_Y", pMyRobotState->Robot_Position_Y);
	SmartDashboard::PutNumber("Start_Heading", pMyRobotState->Robot_Heading);
	}

/*
 * For each of the autonomous commands below, we need to first determine the status.
 * if (completed !! timedout) return immediately.  This step is already done.
 *
 * if (notstarted), we need to add the current AutoTime value to the timeout specified
 * for the command and set the status to started.  Then, we need to determine the target
 * state of the robot and set appropriate values in the target state buffer.
 * Note:  if the timeout was specified as 0, there is no timeout feature, so add the
 * current AutoTime plus the AUTONOMOUS_PERIOD to the timeout value for this step.
 *
 * if (started) (including if we just started it), we need to determine if the timeout value
 * has been exceeded.  If so, set the status flag to timedout and stop the associated motor
 * controllers.  If not timedout, evaluate the current state buffer against the target
 * state buffer and determine what, if any, motor controllers (or other devices) need to
 * be modified.  If the current state buffer matches the target state buffer, set the
 * status flag to completed.
 */

void AUTONOMOUS :: ExecNoOp()
	{
	SmartDashboard::PutString("Autonomous Command", "NoOp");
	switch (CompletedTasks[pScenario->Steps[step].TaskIndex])
		{
		case notstarted:
			if (pScenario->Steps[step].Timeout == 0) pScenario->Steps[step].Timeout = AUTONOMOUS_PERIOD;
			pScenario->Steps[step].Timeout += AutoTime;
			CompletedTasks[pScenario->Steps[step].TaskIndex] = started;
			// there is no change to the target state buffer (kindof what NoOp means).
//			break;		// no break here, we just started the step, so now we need to evaluate the operation

		case started:
			if (pScenario->Steps[step].Timeout <= AutoTime)
				{
				CompletedTasks[pScenario->Steps[step].TaskIndex] = timedout;
				// no associated motor controllers to stop
				}
			// there are no target state buffer completion criteria (kindof what NoOp means).
			return;
			break;

		case completed:
			return;
			break;

		case timedout:
			return;
			break;
		}
	}

void AUTONOMOUS :: ExecArrive()
	{
	SmartDashboard::PutString("Autonomous Command", "Arrive");
	switch (CompletedTasks[pScenario->Steps[step].TaskIndex])
		{
		case notstarted:
			if (pScenario->Steps[step].Timeout == 0) pScenario->Steps[step].Timeout = AUTONOMOUS_PERIOD;
			pScenario->Steps[step].Timeout += AutoTime;
			CompletedTasks[pScenario->Steps[step].TaskIndex] = started;
			pMyRobotState->Robot_Position_X = pMyAutoTarget->Robot_Position_X;
			pMyRobotState->Robot_Position_Y = pMyAutoTarget->Robot_Position_Y;
			pMyRobotState->Robot_Heading = pMyAutoTarget->Robot_Heading;
			//			break;		// no break here, we just started the step, so now we need to evaluate the operation

		case started:
			// there is nothing to do (so we are already done).
			CompletedTasks[pScenario->Steps[step].TaskIndex] = completed;
			return;
			break;

		case completed:
			return;
			break;

		case timedout:
			return;
			break;
		}
	}


void AUTONOMOUS :: ExecSetControls()
	{
	SmartDashboard::PutString("Autonomous Command", "SetControls");
	switch (CompletedTasks[pScenario->Steps[step].TaskIndex])
		{
		case notstarted:
			if (pScenario->Steps[step].Timeout == 0) pScenario->Steps[step].Timeout = AUTONOMOUS_PERIOD;
			pScenario->Steps[step].Timeout += AutoTime;
			CompletedTasks[pScenario->Steps[step].TaskIndex] = started;
			pMyRobotState->Drivetrain_Speed_Control = (SpeedControl) pScenario->Steps[step].parameters.SetControls.SpeedControl;
			pMyRobotState->Drivetrain_Spin_Control = (pScenario->Steps[step].parameters.SetControls.SpinControl == 1);
			pMyRobotState->Drivetrain_Directional_Orientation = (DriveOrientation) pScenario->Steps[step].parameters.SetControls.Orientation;
			SmartDashboard::PutNumber("SprintMode",pMyRobotState->Drivetrain_Speed_Control);
			SmartDashboard::PutNumber("OrientMode",pMyRobotState->Drivetrain_Directional_Orientation);
			SmartDashboard::PutBoolean("SpinControl",pMyRobotState->Drivetrain_Spin_Control);
			//			break;		// no break here, we just started the step, so now we need to evaluate the operation

		case started:
			// there is nothing to do (so we are already done).
			CompletedTasks[pScenario->Steps[step].TaskIndex] = completed;
			return;
			break;

		case completed:
			return;
			break;

		case timedout:
			return;
			break;
		}
	}


void AUTONOMOUS :: ExecWait4()
	{
//	SmartDashboard::PutString("Autonomous Command", "Wait4_" + SmartDashboard::GetString("Autonomous Command"));
	switch (CompletedTasks[pScenario->Steps[step].TaskIndex])
		{
		case notstarted:
			if (pScenario->Steps[step].Timeout == 0) pScenario->Steps[step].Timeout = AUTONOMOUS_PERIOD;
			pScenario->Steps[step].Timeout += AutoTime;
			CompletedTasks[pScenario->Steps[step].TaskIndex] = started;
			// there is no change to the target state buffer (kindof what NoOp means).
			return;
			break;

		case started:
			return;
			break;

		case completed:
			return;
			break;

		case timedout:
			return;
			break;
		}
	}

void AUTONOMOUS :: MoveToPosition()
	{
	float xdiff, ydiff, xpower, ypower, brakepower, scale_speed_to_distance, joystick_angle, scale_to_normalize;

	if (pScenario->Steps[step].Timeout <= AutoTime)
		{
		CompletedTasks[pScenario->Steps[step].TaskIndex] = timedout;
		pMyInput->DriveX = 0;
		pMyInput->DriveY = 0;
		}
	else
		{
		xdiff = pMyAutoTarget->Robot_Position_X - pMyRobotState->Robot_Position_X;
		ydiff = pMyAutoTarget->Robot_Position_Y - pMyRobotState->Robot_Position_Y;
		distance = pow(pow(xdiff,2)+pow(ydiff,2),0.5);	// max distance = 785 inches corner to corner

		// min distance that we would want to use 100% power on is about 277 inches
		scale_speed_to_distance = fmin(fabs(pMyAutoTarget->Robot_Speed), pow(distance, 0.8) + MINIMUM_POWER_TO_MOVE) / 100;

		// experimentation shows that we need to apply braking power to stop at the desired coordinates
		// otherwise the robot overruns the target coordinates, or we have to start coasting to the target so early that it takes too long.

		//		scale_speed_to_distance -= (fabs(pMyAutoTarget->Robot_Speed) * 0.1 * fabs(distance - prev_distance) / fmax(0.2, distance));
		//		prev_distance = distance;
		// I don't like the above algorithm.  I think we need to compare the distance to our speed of closure
		// and set a braking power between -0.25 and 2 * power settings we would otherwise be applying

		/*
		 * To do this "right" -- kind of like when you're driving a car.  We will keep the power setting given in the instructions
		 * until we reach a point at whcih we need to begin braking.  That braking point will be proportional to the speed of closure
		 * (which is presumably consistent with the power setting, but may be influenced by other factors -- like defending robots).
		 * When we get to the braking point, we start applying reverse power commenserate with the robot's speed.
		 *
		 * so, for example:
		 * if pMyRobotState->Robot_Speed == distance, reverse power	(brakepower = 2 * scale_speed_to_distance)
		 * if pMyRobotState->Robot_Speed == distance / 2, cut power (brakepower = 1 * scale_speed_to_distance)
		 * if pMyRobotState->Robot_Speed <= distance / 3, hold power as requested (brakepower = 0 * scale_speed_to_distance)
		 * if pMyRobotState->Robot_Speed <= distance / 4, add 25% power to requested (brakepower = -0.25 * scale_speed_to_distance
		 *
		 * gives us this formula:
		 * brakepower = fmin(2, fmax(-0.25, 3 - (distance / (pMyRobotState->Robot_Speed + 1)))) * scale_speed_to_distance;
		 *
		 * However...
		 * If we overrun the target coordinates (maybe the autoscenario updated the target coordinates before we reached the previous destination),
		 * we need to vacate the brakepower for this cycle only.  We could check to see if the robot direction of movement is taking us closer
		 * to, or farther away from, the target coordinates.  That would tell us if we overran the target.
		 *
		 * bool overshot =	(0 < pMyRobotState->Robot_Direction < 180 && xdiff < 0) || (180 < pMyRobotState->Robot_Direction < 360 && xdiff > 0) ||
		 * 					(0 < pMyRobotState->Robot_Direction < 90 && ydiff < 0) || (90 < pMyRobotState->Robot_Direction < 270 && ydiff > 0) ||
		 * 					(270 < pMyRobotState->Robot_Direction < 360 && ydiff < 0)
		 * pMyRobotState->Robot_Direction
		 *
		 * Could consider multiplying brakepower * -0.25 rather than just turning it off.
		 * Turning off brakepower in the case of an overrun situation seems counter-intuitive, but in an overrun, the power to the drivetrain
		 * will be reversed in order to move toward the target.  This would cause the brakepower to be set away from the target, which we do
		 * not want.
		 */

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
		pMyInput->DriveX = xpower;
		pMyInput->DriveY = ypower * -1 * INVERT_Y_AXIS_INPUT;
		if (fabs(distance) < 1)
			{
			CompletedTasks[pScenario->Steps[step].TaskIndex] = completed;
			}
		}
	}

void AUTONOMOUS :: ExecMoveTo()
	{
	SmartDashboard::PutString("Autonomous Command", "MoveTo");
	switch (CompletedTasks[pScenario->Steps[step].TaskIndex])
		{
		case notstarted:
			if (pScenario->Steps[step].Timeout == 0) pScenario->Steps[step].Timeout = AUTONOMOUS_PERIOD;
			pScenario->Steps[step].Timeout += AutoTime;
			CompletedTasks[pScenario->Steps[step].TaskIndex] = started;
			pMyAutoTarget->Robot_Position_X = pScenario->Steps[step].parameters.MoveTo.Xcoordinate;
			pMyAutoTarget->Robot_Position_Y = pScenario->Steps[step].parameters.MoveTo.Ycoordinate;
			pMyAutoTarget->Robot_Speed = pScenario->Steps[step].parameters.MoveTo.Power;
			SmartDashboard::PutNumber("Target_X",pMyAutoTarget->Robot_Position_X);
			SmartDashboard::PutNumber("Target_Y",pMyAutoTarget->Robot_Position_Y);
			SmartDashboard::PutNumber("Target_Speed",pMyAutoTarget->Robot_Speed);
			//			break;		// no break here, we just started the step, so now we need to evaluate the operation

		case started:
			MoveToPosition();
			return;
			break;

		case completed:
			return;
			break;

		case timedout:
			return;
			break;
		}
	}


void AUTONOMOUS :: ExecMoveRelative()
	{
	SmartDashboard::PutString("Autonomous Command", "MoveRelative");
	switch ((int)CompletedTasks[pScenario->Steps[step].TaskIndex])
		{
		case notstarted:
			if (pScenario->Steps[step].Timeout == 0) pScenario->Steps[step].Timeout = AUTONOMOUS_PERIOD;
			pScenario->Steps[step].Timeout += AutoTime;
			CompletedTasks[pScenario->Steps[step].TaskIndex] = started;
			pMyAutoTarget->Robot_Position_X += pScenario->Steps[step].parameters.MoveRelative.Xdistance;
			pMyAutoTarget->Robot_Position_Y += pScenario->Steps[step].parameters.MoveRelative.Ydistance;
			pMyAutoTarget->Robot_Speed = pScenario->Steps[step].parameters.MoveRelative.Power;
			SmartDashboard::PutNumber("Target_X",pMyAutoTarget->Robot_Position_X);
			SmartDashboard::PutNumber("Target_Y",pMyAutoTarget->Robot_Position_Y);
			SmartDashboard::PutNumber("Target_Speed",pMyAutoTarget->Robot_Speed);
//			break;		// no break here, we just started the step, so now we need to evaluate the operation

		case started:
			MoveToPosition();
			return;
			break;

		case completed:
			return;
			break;

		case timedout:
			return;
			break;

		}
	}

void AUTONOMOUS :: TurnToHeading()
	{
	if (pScenario->Steps[step].Timeout <= AutoTime)
		{
		CompletedTasks[pScenario->Steps[step].TaskIndex] = timedout;
		pMyRobotState->Drivetrain_Heading = pMyAutoTarget->Robot_Heading;	// Let Spin Control complete the turn
		pMyInput->DriveR = 0;
		}
	else
		{
		double angular_difference = (int(1000 * (pMyAutoTarget->Robot_Heading - pMyRobotState->Robot_Heading + 360)) % 360000) / 1000;
		angular_difference -= angular_difference > 180 ? 360 : 0;		// spin the shortest direction
		// need to scale DriveR value such that
		//		when angular_difference becomes small, DriveR gets small, but not too small that we stop moving prematurely
		//		when angular_difference is large, DriveR does not exceed pMyAutoTarget->Robot_Speed
//		pMyInput->DriveR = fmax(-1 * pMyAutoTarget->Robot_Speed, fmin(pMyAutoTarget->Robot_Speed, angular_difference / ACTIVESENSE));
		pMyInput->DriveR = fabs(angular_difference) < 0.25 ? 0 : copysign(fmin(pMyAutoTarget->Robot_Speed/100, (0.02 * fabs(angular_difference)) + (log1p(int(fabs(angular_difference)) % 10)/(fabs(ACTIVESENSE)+1.0))), angular_difference);
		// when angular_differece is near zero, anticipate stopping based on speed to avoid significantly overshooting
		// set the Drivetrain_Heading and let Spin Control complete the turn
		if (fabs(angular_difference) < (pMyInput->DriveR * 5))
			{
					CompletedTasks[pScenario->Steps[step].TaskIndex] = completed;
			pMyRobotState->Drivetrain_Heading = pMyAutoTarget->Robot_Heading;	// Let Spin Control complete the turn
			pMyInput->DriveR = 0;
			}
		if (fabs(angular_difference) < 0.25)
			{
			CompletedTasks[pScenario->Steps[step].TaskIndex] = completed;
			pMyRobotState->Drivetrain_Heading = pMyAutoTarget->Robot_Heading;	// Let Spin Control complete the turn
			pMyInput->DriveR = 0;
			}
		}
	SmartDashboard::PutNumber("DriveR",pMyInput->DriveR);
	}


void AUTONOMOUS :: ExecTurnTo()
	{
	SmartDashboard::PutString("Autonomous Command", "TurnTo");
	switch (CompletedTasks[pScenario->Steps[step].TaskIndex])
		{
		case notstarted:
			if (pScenario->Steps[step].Timeout == 0) pScenario->Steps[step].Timeout = AUTONOMOUS_PERIOD;
			pScenario->Steps[step].Timeout += AutoTime;
			CompletedTasks[pScenario->Steps[step].TaskIndex] = started;
			pMyAutoTarget->Robot_Heading = pScenario->Steps[step].parameters.TurnTo.Heading;
			pMyRobotState->Drivetrain_Heading = pMyRobotState->Robot_Heading;	// Let Spin Control turn the robot
			pMyAutoTarget->Robot_Speed = pScenario->Steps[step].parameters.TurnTo.Power;
			SmartDashboard::PutNumber("Target_Heading",pMyAutoTarget->Robot_Heading);
			SmartDashboard::PutNumber("Target_Speed",pMyAutoTarget->Robot_Speed);
			/*
			 * Note:  An alternative approach is to enable the SpinControl feature in the Drivetrain and
			 * just set the pMyRobotState->Drivetrain_Heading to the desired target heading.
			 */
//			break;		// no break here, we just started the step, so now we need to evaluate the operation

		case started:
			TurnToHeading();
			return;
			break;

		case completed:
			return;
			break;

		case timedout:
			return;
			break;
		}
	}


void AUTONOMOUS :: ExecTurnRelative()
	{
	SmartDashboard::PutString("Autonomous Command", "TurnRelative");
	switch (CompletedTasks[pScenario->Steps[step].TaskIndex])
		{
		case notstarted:
			if (pScenario->Steps[step].Timeout == 0) pScenario->Steps[step].Timeout = AUTONOMOUS_PERIOD;
			pScenario->Steps[step].Timeout += AutoTime;
			CompletedTasks[pScenario->Steps[step].TaskIndex] = started;
			pMyAutoTarget->Robot_Heading = (double) (int(1000 * (pMyAutoTarget->Robot_Heading + pScenario->Steps[step].parameters.TurnRelative.Degrees + 360)) % 360000) / 1000.000;
			pMyAutoTarget->Robot_Speed = pScenario->Steps[step].parameters.TurnRelative.Power;
			SmartDashboard::PutNumber("Target_Heading",pMyAutoTarget->Robot_Heading);
			SmartDashboard::PutNumber("Target_Speed",pMyAutoTarget->Robot_Speed);
//			break;		// no break here, we just started the step, so now we need to evaluate the operation

		case started:
			TurnToHeading();
			return;
			break;

		case completed:
			return;
			break;

		case timedout:
			return;
			break;
		}
	}

void AUTONOMOUS :: ExecToteWheels()
	{
	SmartDashboard::PutString("Autonomous Command", "ToteWheels");
	switch (CompletedTasks[pScenario->Steps[step].TaskIndex])
		{
		case notstarted:
			if (pScenario->Steps[step].Timeout == 0) pScenario->Steps[step].Timeout = AUTONOMOUS_PERIOD;
			pScenario->Steps[step].Timeout += AutoTime;
			CompletedTasks[pScenario->Steps[step].TaskIndex] = started;
			pMyAutoTarget->ToteWheels_Left_Speed = pScenario->Steps[step].parameters.ToteWheels.LeftPower;
			pMyAutoTarget->ToteWheels_Right_Speed = pScenario->Steps[step].parameters.ToteWheels.RightPower;
//			break;		// no break here, we just started the step, so now we need to evaluate the operation

		case started:
			if (pScenario->Steps[step].Timeout <= AutoTime)
				{
				CompletedTasks[pScenario->Steps[step].TaskIndex] = timedout;
				pMyInput->LeftWheel = pMyInput->RightWheel = 0;
				}
			// there is no completion criteria.  The wheels will run until the step times out or is overridden.
			pMyInput->LeftWheel = pMyAutoTarget->ToteWheels_Left_Speed / 100;
			pMyInput->RightWheel = pMyAutoTarget->ToteWheels_Right_Speed / 100;
			return;
			break;

		case completed:
			return;
			break;

		case timedout:
			return;
			break;
		}
	}


void AUTONOMOUS :: ExecToteArms()
	{
	SmartDashboard::PutString("Autonomous Command", "ToteArms");
	switch (CompletedTasks[pScenario->Steps[step].TaskIndex])
		{
		case notstarted:
			if (pScenario->Steps[step].Timeout == 0) pScenario->Steps[step].Timeout = AUTONOMOUS_PERIOD;
			pScenario->Steps[step].Timeout += AutoTime;
			CompletedTasks[pScenario->Steps[step].TaskIndex] = started;
			pMyAutoTarget->ToteWheelArm_Left_Position = pScenario->Steps[step].parameters.ToteArms.WheelPosition;
			pMyAutoTarget->ToteWheelArm_Right_Position = pScenario->Steps[step].parameters.ToteArms.WheelPosition;
//			pMyAutoTarget->DutyCycle = pScenario->Steps[step].parameters.ToteArms.DutyCycle;	// not currently implemented; using defined default value
//			break;		// no break here, we just started the step, so now we need to evaluate the operation

		case started:
			if (pScenario->Steps[step].Timeout <= AutoTime)
				{
				CompletedTasks[pScenario->Steps[step].TaskIndex] = timedout;
				pMyInput->LeftArm = 0;
				pMyInput->RightArm = 0;
				}
			if ((pMyAutoTarget->ToteWheelArm_Left_Position == pMyRobotState->ToteWheelArm_Left_Position)
			&& 	(pMyAutoTarget->ToteWheelArm_Right_Position == pMyRobotState->ToteWheelArm_Right_Position))
				{
				CompletedTasks[pScenario->Steps[step].TaskIndex] = completed;
				pMyInput->LeftArm = 0;
				pMyInput->RightArm = 0;
				}
			else
				{
				pMyInput->LeftArm = (pMyAutoTarget->ToteWheelArm_Left_Position == pMyRobotState->ToteWheelArm_Left_Position) ? 0 :
									(pMyAutoTarget->ToteWheelArm_Left_Position < pMyRobotState->ToteWheelArm_Left_Position) ? 1 : -1;
				pMyInput->RightArm = (pMyAutoTarget->ToteWheelArm_Right_Position == pMyRobotState->ToteWheelArm_Right_Position) ? 0 :
									(pMyAutoTarget->ToteWheelArm_Right_Position < pMyRobotState->ToteWheelArm_Right_Position) ? -1 : 1;
				}
			return;
			break;

		case completed:
			return;
			break;

		case timedout:
			return;
			break;
		}
	}


void AUTONOMOUS :: ExecToteLift()
	{
	SmartDashboard::PutString("Autonomous Command", "ToteLift");
	switch (CompletedTasks[pScenario->Steps[step].TaskIndex])
		{
		case notstarted:
			if (pScenario->Steps[step].Timeout == 0) pScenario->Steps[step].Timeout = AUTONOMOUS_PERIOD;
			pScenario->Steps[step].Timeout += AutoTime;
			CompletedTasks[pScenario->Steps[step].TaskIndex] = started;
			pMyAutoTarget->ToteLift_Position = pScenario->Steps[step].parameters.ToteLift.Height;
			pMyAutoTarget->ToteLift_Speed = pScenario->Steps[step].parameters.ToteLift.Power;
			//			break;		// no break here, we just started the step, so now we need to evaluate the operation

		case started:
			if (pScenario->Steps[step].Timeout <= AutoTime)
				{
				CompletedTasks[pScenario->Steps[step].TaskIndex] = timedout;
				// no associated motor controllers to stop
				}
			// compare pMyAutoTarget->ToteLift_Target to pMyRobotState->ToteLift_Target
			// push the up button to go up, or the down button to go down, or set completed flag
			if (pMyAutoTarget->ToteLift_Position < pMyRobotState->ToteLift_Position)
				{
				pMyInput->ToteUPBtn = false;
				pMyInput->ToteDNBtn = true;
				}
			else if (pMyAutoTarget->ToteLift_Position > pMyRobotState->ToteLift_Position)
				{
				pMyInput->ToteUPBtn = true;
				pMyInput->ToteDNBtn = false;
				}
			else
				{
				pMyInput->ToteUPBtn = false;
				pMyInput->ToteDNBtn = false;
				CompletedTasks[pScenario->Steps[step].TaskIndex] = completed;
				}
			return;
			break;

		case completed:
			return;
			break;

		case timedout:
			return;
			break;
		}
	}

/*
void AUTONOMOUS :: ExecClawArm()
	{
	SmartDashboard::PutString("Autonomous Command", "ClawArm");
	switch (CompletedTasks[pScenario->Steps[step].TaskIndex])
		{
		case notstarted:
			if (pScenario->Steps[step].Timeout == 0) pScenario->Steps[step].Timeout = AUTONOMOUS_PERIOD;
			pScenario->Steps[step].Timeout += AutoTime;
			CompletedTasks[pScenario->Steps[step].TaskIndex] = started;
			// there is no change to the target state buffer (kindof what NoOp means).
//			break;		// no break here, we just started the step, so now we need to evaluate the operation

		case started:
			if (pScenario->Steps[step].Timeout <= AutoTime)
				{
				CompletedTasks[pScenario->Steps[step].TaskIndex] = timedout;
				// no associated motor controllers to stop
				}
			// there are no target state buffer completion criteria (kindof what NoOp means).
			return;
			break;

		case completed:
			return;
			break;

		case timedout:
			return;
			break;
		}
	}


void AUTONOMOUS :: ExecPincer()
	{
	SmartDashboard::PutString("Autonomous Command", "Pincer");
	switch (CompletedTasks[pScenario->Steps[step].TaskIndex])
		{
		case notstarted:
			if (pScenario->Steps[step].Timeout == 0) pScenario->Steps[step].Timeout = AUTONOMOUS_PERIOD;
			pScenario->Steps[step].Timeout += AutoTime;
			CompletedTasks[pScenario->Steps[step].TaskIndex] = started;
			// there is no change to the target state buffer (kindof what NoOp means).
//			break;		// no break here, we just started the step, so now we need to evaluate the operation

		case started:
			if (pScenario->Steps[step].Timeout <= AutoTime)
				{
				CompletedTasks[pScenario->Steps[step].TaskIndex] = timedout;
				// no associated motor controllers to stop
				}
			// there are no target state buffer completion criteria (kindof what NoOp means).
			return;
			break;

		case completed:
			return;
			break;

		case timedout:
			return;
			break;
		}
	}

*/

