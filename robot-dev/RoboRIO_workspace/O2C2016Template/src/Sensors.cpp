#include "Sensors.h"
// Set up sensors that are not necessarily integral to any other subsystem


SENSORS :: SENSORS()
	{
	photo_left = new DigitalInput(LEFT_PHOTO_DIO_PORT);
	photo_right = new DigitalInput(RIGHT_PHOTO_DIO_PORT);
	#ifdef LIGHT_SENSOR
		light_sensor = new AnalogInput(LIGHT_AIN_PORT);
	#endif

	pdp = new PowerDistributionPanel();
	pMyInput = 0;
	pMyTargetState = 0;
	}

void SENSORS :: Init(RobotStateBuffer *pRobotState, RobotStateBuffer *pTargetState)
	{
	pMyRobotState = pRobotState;
	pMyRobotState->Sensors_Init++;
	pMyTargetState = pTargetState;
	#ifdef CAMERA
		StreamCamera();
	#endif
	pMyRobotState->Sensors_Init++;
	}


void SENSORS :: CheckAll()
	{
	#ifdef PHOTO_SENSORS
		CheckLeftPhotoSensor();
		CheckRightPhotoSensor();
	#endif
	CheckBatteryVoltage();
	CheckPowerConsumption();
	CheckShooterPower();
	#ifdef LIGHT_SENSOR
		CheckLightSensor();
	#endif
	#ifdef RANGE_FINDER
		CheckRange();
	#endif
	}


#ifdef PHOTO_SENSORS

void SENSORS :: CheckLeftPhotoSensor()
	{
	pMyRobotState->Sensors_LeftPhoto = photo_left->Get() ? 1.0 : 0.0;
	}

void SENSORS :: CheckRightPhotoSensor()
	{
	pMyRobotState->Sensors_RightPhoto = photo_right->Get() ? 1.0 : 0.0;
	}

#endif

void SENSORS :: CheckBatteryVoltage()
	{
	pMyRobotState->Battery_Voltage = pdp->GetVoltage();
	}

void SENSORS :: CheckPowerConsumption()
	{
	pMyRobotState->Power_Consumption = pdp->GetTotalPower();
	}

void SENSORS :: CheckShooterPower()
	{
//	#ifdef PRACTICEBOT	// The PracticeBot does not have a PDP, so cannot actually measure shooter power directly
//		pMyRobotState->Shooter_Launch_Power = 0.6 * pMyTargetState->Shooter_Launch_Power;
//	#endif
//	#ifdef REALBOT
		pMyRobotState->Shooter_Launch_Power = (pdp->GetCurrent(SHOOTER_PDP_PORT_LEFT) + pdp->GetCurrent(SHOOTER_PDP_PORT_RIGHT)) * pMyRobotState->Battery_Voltage;
//	#endif
	}


#ifdef LIGHT_SENSOR

void SENSORS :: CheckLightSensor()
	{
	pMyRobotState->Sensors_LightDetect = light_sensor->GetVoltage();
	}
#endif


#ifdef RANGE_FINDER

void SENSORS :: CheckRange()
	{
	}

#endif


#ifdef CAMERA

void SENSORS :: StreamCamera()
	{
	CameraServer::GetInstance()->SetQuality(pMyRobotState->Sensors_Camera_Quality);
	//the camera name (ex "cam0") can be found through the roborio web interface
	CameraServer::GetInstance()->StartAutomaticCapture("cam0");
	}

#endif



