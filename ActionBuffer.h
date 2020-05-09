#pragma once
#include "ViewTypes.h"

using namespace System::Collections;
ref class ActionBuffer
{
private:

	Generic::List<array<float> ^> ^pBuffer;
	unsigned int Count;
public:

	ActionBuffer(int buff_size) {
		pBuffer = gcnew Generic::List<array<float> ^>();
		Count = 0;

	}

	void Push(array<float> ^ event)
	{
		Count += 1;
		pBuffer->Add(event);
	}
	array<float> ^ Pop()
	{
		array<float> ^p_ret = pBuffer[0];
		Count -= 1;
		pBuffer->RemoveAt(0);
		return p_ret;
	}
	unsigned int GetCount()
	{
		return Count;
	}
	Generic::List<array<float> ^> ^GetQueue()
	{
		return pBuffer;
	}

	array<array<float> ^> ^GetBuffer()
	{
		return pBuffer->ToArray();
	}

	void LoadPattern(array<array<float> ^>^ p_event) {
		for (int i = 0; i < p_event->Length; i++) {
			pBuffer->Add(p_event[i]);
		}

	}
};