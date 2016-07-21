#include <ROBOT.h>

ROBOT :: ROBOT()
	{
//	lw = LiveWindow::GetInstance();
	pMyTargetState = 0;
	}

void ROBOT :: RobotInit()
	{
	SmartDashboard::init();
	Wait (2);
	pMyTargetState = new RobotStateBuffer;				// This is the state we want the robot to get to
	pMyRobotState = new RobotStateBuffer;				// This is the state the robot is actually in
	pMyInput = new INPUT;								// These are the input signals to change the target state
	memset(pMyTargetState, 0, sizeof(*pMyTargetState));
	memset(pMyRobotState, 0, sizeof(*pMyRobotState));
	memset(pMyInput, 0, sizeof(*pMyInput));
	Wait (1);
	#ifdef REPORT_ROBOT_DEBUG
		SmartDashboard::PutNumber("RobotInit",pMyRobotState->Robot_Init);		// Just push the Robot and Comms Init values
		SmartDashboard::PutString("Version","0.1");		// Just push the Robot and Comms Init values
	#endif

	Comms.Init(pMyRobotState, pMyTargetState, pMyInput);	// Send values from all state buffers that have changed
	pMyRobotState->Robot_Init++;
	Comms.Send();

	Sensors.Init(pMyRobotState, pMyTargetState);							// Sensors to not take control inputs and do not change the target state
	Comms.Send();

	Drivetrain.Init(pMyRobotState, pMyTargetState, pMyInput);	// Take control inputs, change the target state, drive the robot to the target state
	Comms.Send();

	SuperDuperScooperShooter.Init(pMyRobotState, pMyTargetState, pMyInput);		// Take control inputs, change target state, converge to target state
	Comms.Send();

	Climber.Init(pMyRobotState, pMyTargetState, pMyInput);		// Take control inputs, change target state, converge to target state
	Comms.Send();

	Navigator.Init(pMyRobotState);							// Navigator takes no control inputs and does not modify the target state
	Comms.Send();											// It uses a gyroscope, accelerometer, and wheel encoders to track robot coordinates and orientation

	AutoPilot.Init(pMyRobotState, pMyTargetState, pMyInput);	// Execute autonomous program to modify the target state, compare robot state to target state
	Comms.Send();												// and generate control inputs to converge robot state to target state

	pMyRobotState->Robot_Init++;
	Comms.Send();
	}


void ROBOT :: AutonomousInit()
	{
	Sensors.CheckAll();
	Navigator.Zero();
//	pMyInput->ScooperUpBtn = true;						// make sure when we enable the scooper, it doesn't run off and do something stupid
	pMyTargetState->Shooter_Elevation = SCOOPER_START_ELEVATION;
	SuperDuperScooperShooter.RunElevator();
	AutoPilot.Start();
	}

void ROBOT :: TestInit()
	{
	Sensors.CheckAll();
	Navigator.Zero();
	if (pMyRobotState->Auto_State < 2)						// Autonomous program did not execute
		{
//		pMyInput->ScooperUpBtn = true;						// make sure when we enable the scooper, it doesn't run off and do something stupid
		pMyTargetState->Shooter_Elevation = SCOOPER_START_ELEVATION;
		SuperDuperScooperShooter.RunElevator();
		}
	}

void ROBOT :: DisabledInit()
	{
	AutoPilot.Stop();
	Sensors.CheckAll();
	Drivetrain.StopMotors(); 	// stop robot
	Climber.StopMotors();
//	if (pMyRobotState->Auto_State < 2)						// Autonomous program did not execute
//		{
//		pMyInput->ScooperUpBtn = true;						// make sure when we enable the scooper, it doesn't run off and do something stupid
//		pMyTargetState->Shooter_Elevation = SCOOPER_START_ELEVATION;
//		SuperDuperScooperShooter.RunElevator();
//		}
	}

void ROBOT :: TeleopInit()
	{
	AutoPilot.Stop();
	Sensors.CheckAll();
	Navigator.Zero();
	if (pMyRobotState->Auto_State < 2)						// Autonomous program did not execute
		{
//		pMyInput->ScooperUpBtn = true;						// make sure when we enable the scooper, it doesn't run off and do something stupid
		pMyTargetState->Shooter_Elevation = SCOOPER_START_ELEVATION;
		SuperDuperScooperShooter.RunElevator();
		}
	}

void ROBOT :: AutonomousPeriodic()
	{
	Sensors.CheckAll();
	AutoPilot.RunAuto();
	Drivetrain.Drive();
	SuperDuperScooperShooter.RunElevator();
	SuperDuperScooperShooter.RunShooter();
	Climber.RunWinch();
	pMyRobotState->Field_Totalruns++;
	pMyRobotState->Field_Enabledruns++;
	pMyRobotState->Field_Autoruns++;
	Comms.Send();
	}

void ROBOT :: TeleopPeriodic()
	{
	Comms.Get();				// get pInput data
	Sensors.CheckAll();
	Drivetrain.Drive();
	SuperDuperScooperShooter.RunElevator();
	SuperDuperScooperShooter.RunShooter();
	Climber.RunWinch();
	pMyRobotState->Field_Totalruns++;
	pMyRobotState->Field_Enabledruns++;
	pMyRobotState->Field_Teleruns++;
	Comms.Send();
	}

void ROBOT :: TestPeriodic()
	{
	Comms.Get();				// get pInput data
	Sensors.CheckAll();
	Drivetrain.Drive();
	SuperDuperScooperShooter.RunElevator();
	SuperDuperScooperShooter.RunShooter();
	Climber.RunWinch();
	//	Navigator.ReadIMUCalStat();
	//	lw->Run();
	pMyRobotState->Field_Totalruns++;
	pMyRobotState->Field_Enabledruns++;
	pMyRobotState->Field_Testruns++;
	Comms.Send();
	}

void ROBOT :: DisabledPeriodic()
	/*
	 * While the robot is disabled, we will continously poll the DriverStation (SmartDashboard) for changes to the
	 * field configuration, start position, and goal selection.  If any changes to these selections is detected,
	 * we will rebuild the approprate autonomous program.
	 */
	{
	Comms.Get();				// get pInput data
	Sensors.CheckAll();
	SuperDuperScooperShooter.GetElevator();
	AutoPilot.SelectScenario();
//	SuperDuperScooperShooter.RunElevator();
	Navigator.SetStartingPosition();
	pMyRobotState->Field_Totalruns++;
	pMyRobotState->Field_Disabledruns++;
	Comms.Send();
	}


START_ROBOT_CLASS(ROBOT)

