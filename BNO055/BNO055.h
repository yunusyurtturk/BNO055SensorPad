#pragma once

#include "SerialPort\SerialControl.h"
#include "BNO055Registers.h"
#include <string.h>
using namespace System;
using namespace System::Collections;
using namespace System::Collections::Generic;
using namespace System::IO::Ports;




typedef enum {
	IMU_MODE_I2C = 1,
	IMU_MODE_SERIAL = 2
}E_IMU_MODE;

typedef bool boolean;
typedef unsigned char uint8_t;
typedef signed char int8_t;
typedef int int32_t;
typedef unsigned int uint32_t;


typedef struct {
	union { 
		float v[3];
		struct {
			float x;
			float y;
			float z;
		};
		/* Orientation sensors */
		struct {
			float roll;    /**< Rotation around the longitudinal axis (the plane body, 'X axis'). Roll is positive and increasing when moving downward. -90°<=roll<=90° */
			float pitch;   /**< Rotation around the lateral axis (the wing span, 'Y axis'). Pitch is positive and increasing when moving upwards. -180°<=pitch<=180°) */
			float heading; /**< Angle between the longitudinal axis (the plane body) and magnetic north, measured clockwise when viewing from the top of the device. 0-359° */
		};
	};
	int8_t status;
	uint8_t reserved[3];
} sensors_vec_t;

/** struct sensors_color_s is used to return color data in a common format. */
typedef struct {
	union {
		float c[3];
		/* RGB color space */
		struct {
			float r;       /**< Red component */
			float g;       /**< Green component */
			float b;       /**< Blue component */
		};
	};
	uint32_t rgba;         /**< 24-bit RGBA value */
} sensors_color_t;

typedef struct
{
	int32_t  version;
	int32_t  sensor_id;
	int32_t  type;
	int32_t  reserved0;
	int32_t  timestamp;
	union
	{
		float           data[4];
		sensors_vec_t   acceleration;
		sensors_vec_t   magnetic;
		sensors_vec_t   orientation;
		sensors_vec_t   gyro;
		float           temperature;
		float           distance;
		float           light;
		float           pressure;
		float           relative_humidity;
		float           current;
		float           voltage;
		sensors_color_t color;
	};
} sensors_event_t;


typedef union {
	uint32_t calibval;
	struct {
		uint8_t sys;
		uint8_t gyro;
		uint8_t acc;
		uint8_t mag;
	};
}TypeCalibration;

typedef struct {
	int32_t start_marker;
	int32_t version;
	int32_t sensor_id;
	int32_t type;
	uint32_t timestamp;
	float euler[3];
	float acc[3];
	float magneto[3];
	float gyro[3];
	float quat[4];
	TypeCalibration calibration;
	int32_t end_marker;
	int32_t checksum;
}S_BNO055_SENSOR_SAMPLE;

ref class CIMUControl{
private:
	CSerialControl		 ^SerialControl;
	
	S_BNO055_SENSOR_SAMPLE		 *pSensorEvent;
	Object ^ __lockObj;
	
public:
	array<unsigned char> ^SerialBuffer;
	String ^ReadLine;

	CIMUControl()
	{
		__lockObj = gcnew Object();
	}
	void Init(CSerialControl ^serial_control) 
	{
		this->SerialControl = serial_control;

		this->SerialBuffer = gcnew array<unsigned char>(40960);

		SerialControl->Open();

		pin_ptr<unsigned char>	pin_ptr = &SerialBuffer[0];

		pSensorEvent = (S_BNO055_SENSOR_SAMPLE *)pin_ptr;

		//array<Byte> ^buff = gcnew array<Byte>(MAX_BUFFER_LEN);

		

		//Begin();
		// SerialControl->WriteByte(BNO055_PAGE_ID_ADDR, 0, false);
		// 
		// 
		// 
		// 
		// 
		// 
		// SerialControl->WriteByte(BNO055_OPR_MODE_ADDR, OPERATION_MODE_CONFIG, false);	/* Config Mode */
		// 
		// buff = SerialControl->ReadBytes(BNO055_CHIP_ID_ADDR, 1, true);
		// 
		// //SerialControl->WriteByte(BNO055_PAGE_ID_ADDR, 0, false);	
		// 
		// buff = SerialControl->ReadBytes(BNO055_CHIP_ID_ADDR, 1, true);
		// buff = SerialControl->ReadBytes(BNO055_MAG_REV_ID_ADDR, 1, true);
		// 
		// 
		// buff = SerialControl->ReadBytes(BNO055_ACCEL_REV_ID_ADDR, 1, true);
		// buff = SerialControl->ReadBytes(BNO055_CHIP_ID_ADDR, 1, true);
		// 
		// 
		// buff = SerialControl->ReadBytes(BNO055_GYRO_REV_ID_ADDR, 1, true);




	}
	int ReadSensorData()
	{
		//Threading::Monitor::Enter(__lockObj);
		ReadLine = SerialControl->ReadLine();
		Text::Encoding ^ascii = Text::Encoding::GetEncoding("iso-8859-1");
		
		array<Byte> ^read_line_buffer = ascii->GetBytes(ReadLine);
		pin_ptr<Byte> serial_buffer = &SerialBuffer[0];
		pin_ptr<Byte> pin_read_line_buffer = &read_line_buffer[0];
		
		memcpy(serial_buffer, pin_read_line_buffer, ascii->GetByteCount(ReadLine));


		//Threading::Monitor::Exit(__lockObj);
		//System::Runtime::InteropServices::Marshal::Copy((IntPtr)&read_data[0], SerialBuffer, 0, 32);
		//System::Runtime::InteropServices::Marshal::Copy(IntPtr(&ReadLine[0]), SerialBuffer, 0, rdatastr.size());
		//int read_count = SerialControl->ReadByte(this->SerialBuffer);
		//ReadLine = System::Text::ASCIIEncoding::ASCII->GetString(SerialBuffer);
	
		return ascii->GetByteCount(ReadLine);;
	}
	float GetAccX()
	{
		return pSensorEvent->acc[0];
	}

	S_BNO055_SENSOR_SAMPLE *GetEvent()
	{

		int i = 0;
		while ('S' != SerialBuffer[i] && 'E' != SerialBuffer[i + 1] 
			&& 'S' != SerialBuffer[i+2] && 'E' != SerialBuffer[i + 3] && i < 4000) {
			i++;
		}

		pin_ptr<unsigned char>	pin_ptr = &SerialBuffer[i+4];

		pSensorEvent = (S_BNO055_SENSOR_SAMPLE *)pin_ptr;
		
		return pSensorEvent;
	}

	S_BNO055_SENSOR_SAMPLE GetEventValue()
	{
		S_BNO055_SENSOR_SAMPLE sensor_event;
		//Threading::Monitor::Enter(__lockObj);
		int i = 0;
		while ('S' != SerialBuffer[i] && 'E' != SerialBuffer[i + 1]
			&& 'S' != SerialBuffer[i + 2] && 'E' != SerialBuffer[i + 3] && i < 4000) {
			i++;
		}

		pin_ptr<unsigned char>	pin_ptr = &SerialBuffer[i + 4];

		sensor_event = *(S_BNO055_SENSOR_SAMPLE *)pin_ptr;
		//Threading::Monitor::Exit(__lockObj);
		return sensor_event;
	}

	void Begin()
	{
		SerialControl->WriteByte(BNO055_PAGE_ID_ADDR, 0, false);
		SetConfigMode();

		SerialControl->WriteByte(BNO055_PAGE_ID_ADDR, 0, true);

		Byte chip_id = SerialControl->ReadByte(BNO055_CHIP_ID_ADDR);

		GetRevision();

		SerialControl->WriteByte(BNO055_SYS_TRIGGER_ADDR, 0x20, false);

		for (int i = 0; i < 1000; i++);

		SerialControl->WriteByte(BNO055_PWR_MODE_ADDR, POWER_MODE_NORMAL, true);
		
		SerialControl->WriteByte(BNO055_SYS_TRIGGER_ADDR, 0x0, true);

		

		SetOperationMode();

		

	}

	void GetRevision() {
		array<Byte> ^buff = gcnew array<Byte>(MAX_BUFFER_LEN);
		Byte acc, mag, gyro, bl, sw_lsb, sw_msb, sw;

		acc		= SerialControl->ReadByte(BNO055_ACCEL_REV_ID_ADDR);
		mag		= SerialControl->ReadByte(BNO055_MAG_REV_ID_ADDR);
		gyro	= SerialControl->ReadByte(BNO055_GYRO_REV_ID_ADDR);
		bl		= SerialControl->ReadByte(BNO055_BL_REV_ID_ADDR);
		sw_lsb	= SerialControl->ReadByte(BNO055_SW_REV_ID_LSB_ADDR);
		sw_msb	= SerialControl->ReadByte(BNO055_SW_REV_ID_MSB_ADDR);
		sw		= ((sw_msb << 8) | sw_lsb) & 0xFFFF;


	}
	void SetConfigMode() {
		
		
		SerialControl->WriteByte(BNO055_OPR_MODE_ADDR, OPERATION_MODE_CONFIG, true);
	}
	void SetOperationMode() {
		SerialControl->WriteByte(BNO055_OPR_MODE_ADDR, OPERATION_MODE_NDOF, true);
	}

	void Write(String ^str)
	{
//		SerialControl->Write(str);
	}



};