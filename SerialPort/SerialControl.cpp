#include "SerialControl.h"

CSerialControl::CSerialControl()
{
	

}

List<String ^> ^CSerialControl::GetAvailablePorts()
{
	array<String ^> ^port_names_array = System::IO::Ports::SerialPort::GetPortNames();
	List<String ^>  ^port_names = gcnew List<String ^>(port_names_array);

	return port_names;
}

List<int > ^CSerialControl::GetBaudRates()
{
	List<int> ^baud_rates = gcnew List<int>();

	baud_rates->Add(300);
	baud_rates->Add(600);
	baud_rates->Add(1200);
	baud_rates->Add(2400);
	baud_rates->Add(9600);
	baud_rates->Add(14400);
	baud_rates->Add(19200);
	baud_rates->Add(38400);
	baud_rates->Add(57600);
	baud_rates->Add(115200);

	return baud_rates;

}
IO::Ports::SerialPort ^  CSerialControl::Init(String ^com_port_name)
{
	IO::Ports::SerialPort ^port = gcnew IO::Ports::SerialPort(com_port_name, 9600, IO::Ports::Parity::None, 8, IO::Ports::StopBits::One);
	port->ReadTimeout = 4000;
	port->WriteTimeout = 4000;
	this->Port = port;
	return Port;
}
IO::Ports::SerialPort ^ CSerialControl::Open()
{
	Port->Open();
	return Port;
}

int CSerialControl::WriteByte(int addr, int value, bool ack )
{
	int res = 0;
	array<unsigned char> ^command = gcnew array<unsigned char>(5);
	command[0] = 0xAA; // Start Byte
	command[1] = 0x00;	//Write
	command[2] = addr & 0xFF;
	command[3] = 1;
	command[4] = value & 0xFF;

	res = this->Write(command,  5, ack);

	return res;
}

int CSerialControl::Write(array<unsigned char> ^text, int length, bool ack)
{
	int attemps = 0;
	int read_count = 0;
	array<unsigned char> ^read_buf = gcnew array<unsigned char>(2);

	while (true) {
		Port->DiscardInBuffer();
		Port->Write(text, 0, length);
		if (!ack) {
			return 1;
		}
		Array::Clear(read_buf, 0, 2);

		read_count = Port->Read(read_buf, 0, 2);

		
		if (attemps > 4) {

			return 0;
		}
		attemps++;

		if ( read_count != 2) {

			continue;
		}

		if (!(read_buf[0] == 0xEE && read_buf[1] == 0x07)) {

			return read_buf[1];
		}

		



	}
	return 0;
}
//int CSerialControl::Write(String ^text)
//{
//	return Port->Write(text);
//}

Byte CSerialControl::ReadByte(int addr) {
	array<unsigned char> ^command = gcnew array<unsigned char>(4);
	Byte res;
	command = this->ReadBytes(addr, 1, true);

	if (nullptr == command) {
		res = 0;
	}
	else {
		res = command[2];
	}

	return res;

}

int CSerialControl::ReadByte(array<unsigned char> ^dest)
{
	//int retval = Port->Read(dest, 0, 4096);
	int retval = 0;
	if (Port->BytesToRead >= sizeof(92)) {
		int i = 0;
		//	line = port->ReadLine();
		retval = Port->Read(dest, 0, 4096);

		
	}
	return retval;
}
array<unsigned char> ^ CSerialControl::ReadBytes(int addr, int length, bool ack )
{
	int res = 0;
	array<unsigned char> ^command = gcnew array<unsigned char>(4);
	command[0] = 0xAA; // Start Byte
	command[1] = 0x1;	//Read
	command[2] = addr & 0xFF;
	command[3] = length & 0xFF;

	res = this->Write(command, 4, true);
	//Port->Write(command, 0, 4);

	Array::Clear(command, 0, 4);
	if (res == 1) {
		int bytesToRead = length;
		this->Port->Read(command, 0, bytesToRead);
 		return command;
	}
	else {
		
		return nullptr;
	
	}
	



}
String ^CSerialControl::ReadLine()
{
	return Port->ReadLine();
}




