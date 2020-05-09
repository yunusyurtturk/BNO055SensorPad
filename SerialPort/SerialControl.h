#pragma once

using namespace System;
using namespace System::Collections;
using namespace System::Collections::Generic;


#define MAX_BUFFER_LEN		132


ref class CSerialControl
{
private:
	IO::Ports::SerialPort ^Port;
public:
	CSerialControl();

	List<String ^>	^GetAvailablePorts();
	List<int >		^GetBaudRates();

	IO::Ports::SerialPort ^ Init(String ^com_port_name);
	IO::Ports::SerialPort ^ Open();

	int CSerialControl::WriteByte(int register, int value, bool ack);
	//int CSerialControl::Write(String ^text);
	int Write(array<Byte> ^text, int length, bool ack);

	Byte CSerialControl::ReadByte(int addr);
	int CSerialControl::ReadByte(array<unsigned char> ^dest);
	array<Byte> ^ ReadBytes(int addr, int length, bool ack);
	String ^CSerialControl::ReadLine();
	
};

