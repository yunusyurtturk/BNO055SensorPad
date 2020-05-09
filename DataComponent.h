#pragma once
using namespace System;

ref class DataComponent
{
private:
private:
		String ^name;
		int message_offset;


public:
	property String ^Name {
		String ^get() {
			return name;
		}
		void set(String ^val) {
			name = val;
		}
	}

	property int Offset {
		int get() {
			return message_offset;
		}
		void set(int val) {
			message_offset = val;
		}

	}
public:
	DataComponent(String ^name, int offset)
	{
		Name = name;
		Offset = offset;
	}

	virtual String ^ToString() override
	{
		return Name;
	}
};

