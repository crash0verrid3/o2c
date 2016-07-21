#ifndef SENSORS_H
#define SENSORS_H
//this is where all the comms to and from the robot happen

#include <Definitions.h>
#include <PowerDistributionPanel.h>
#include <DigitalInput.h>

class SENSORS
{

public:
	SENSORS();
	void Init(RobotStateBuffer *pRobotState, RobotStateBuffer *pTargetState);
	void CheckAll();

private:
	#ifdef PHOTO_SENSORS
		void CheckLeftPhotoSensor();
		void CheckRightPhotoSensor();
		DigitalInput	*photo_left, *photo_right;
	#endif
	void CheckBatteryVoltage();
	void CheckPowerConsumption();
	void CheckShooterPower();
	#ifdef LIGHT_SENSOR
		AnalogInput		*light_sensor;
		void CheckLightSensor();
	#endif
	#ifdef RANGE_FINDER
		void CheckRange();
	#endif
	#ifdef CAMERA
		void StreamCamera();
	#endif

	PowerDistributionPanel	*pdp;


};

#endif
