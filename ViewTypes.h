#pragma once
#include "BNO055\BNO055.h"
#include <string.h>
#include "SensorDataTypes.h"




static public ref class SensorDataNames
{
public:
	static array<String ^> ^Names = gcnew array<String ^>(16);;
	static SensorDataNames()
	{

		Names[0] = "Euler 0";
		Names[1] = "Euler 1";
		Names[2] = "Euler 2";

		Names[3] = "Acc 0";
		Names[4] = "Acc 1";
		Names[5] = "Acc 2";

		Names[6] = "Magneto 0";
		Names[7] = "Magneto 1";
		Names[8] = "Magneto 2";

		Names[9] = "Gyro 0";
		Names[10] = "Gyro 1";
		Names[11] = "Gyro 2";

		Names[12] = "Quat 0";
		Names[13] = "Quat 1";
		Names[14] = "Quat 2";
		Names[15] = "Quat 3";

	}

};



public value class ViewSerialEvent {
public:
	S_BNO055_SENSOR_SAMPLE *pEvent;
	float *pProcesseds;

public:
	
	ViewSerialEvent(S_BNO055_SENSOR_SAMPLE *p_event) {
		
		this->pEvent = new S_BNO055_SENSOR_SAMPLE;
		if (NULL != p_event) {
			
			memcpy(pEvent, p_event, sizeof(S_BNO055_SENSOR_SAMPLE));
		}
	}
	void Clear()
	{
		delete(pEvent);
	}
	S_BNO055_SENSOR_SAMPLE *GetEvent() {
		return this->pEvent;
	}
	
};
