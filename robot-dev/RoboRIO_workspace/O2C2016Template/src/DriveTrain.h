/*
 * DriveTrain.h
 *
 *  Created on: Jan 25, 2016
 *      Author: Rod
 */

#ifndef DRIVETRAIN_H_
#define DRIVETRAIN_H_

#include "Definitions.h"

class DRIVETRAIN
{
public:
	DRIVETRAIN();

	friend class AUTOPILOT;

	void Init(RobotStateBuffer *pRobotState, RobotStateBuffer *pTargetState, INPUT *pInput);
	void Drive();
	void StopMotors();

private:

#if DRIVETRAINTYPE==TankDriveTrain
	Talon 		*pMotorL;
	Talon 		*pMotorR;
//	Talon		*pGhostL;
//	Talon		*pGhostR;
	RobotDrive 	*pRobotDrive;

	double SprintCalc();
	double SpinControl();
	double SpinCalc();


#elif DRIVETRAINTYPE==MecanumDriveTrain
	Talon *pMotorFL;
	Talon *pMotorBL;
	Talon *pMotorFR;
	Talon *pMotorBR;
	RobotDrive 	*pRobotDrive;

	double SprintCalc();
	double SpinControl();
	void OrientCalc();



#elif DRIVETRAINTYPE==HolonomicDriveTrain


#elif DRIVETRAINTYPE==KiwiDriveTrain


#elif DRIVETRAINTYPE==SwerveDriveTrain


#elif DRIVETRAINTYPE==NanoDriveTrain


#endif


#if DRIVECONTROL==TankDriveControl


#elif DRIVECONTROL==ArcadeDriveControl


#endif



	DriveOrientation OrientMode;
	float 			OrientAngle;
//	SpeedControl	SprintMode;
	double			sprint_factor, lateral_movement_compensation_factor;
	bool			spincontrol, SprintModeBtn_enabled, SpinControlBtn_enabled, prev_sprint_btn, prev_spin_btn;
	double 			driftangle, prev_driftangle;
	double 			rotation, prev_spin_correction;
	int				SprintMode_button_debouncer, SpinControl_button_debouncer;
	#if DRIVECONTROL==TankDriveControl
		double			left_stick, right_stick;
	#elif DRIVECONTROL==ArcadeDriveControl
		double			x_stick, y_stick, spin_stick;
	#endif



};


#endif /* DRIVETRAIN_H_ */
