#pragma once

#include "SensorValueOps.h"
public ref class SensorOps
{
private:
	array<float> ^HistoryBuffer;
	unsigned int MessageIndex;
	unsigned int HistoryLength;
	float CurrentSumRoot;
public:
	SensorOps() {
		Init(4);
	}
	SensorOps(unsigned int history_lenght) {
		Init(history_lenght);
	}

	void Init(unsigned int history_lenght)
	{
		HistoryBuffer = gcnew array<float>(history_lenght);
		MessageIndex = 0;
		HistoryLength = history_lenght;
	}

	float CalcHistoryWeightedValue(float values[], unsigned int count)
	{
		float av_current_sum = 0;

		MessageIndex++;
		CurrentSumRoot = SensorValueOps::RMS(values, 3);

		HistoryBuffer[(MessageIndex - 1) % HistoryLength] = CurrentSumRoot;
		
		for (int i = 0; i < 4; i++) {

			av_current_sum += HistoryBuffer[i];

		}
		av_current_sum = av_current_sum / HistoryLength;
		HistoryBuffer[(MessageIndex - 1) % HistoryLength] = av_current_sum;

		return av_current_sum;
	}
	~SensorOps()
	{

	}

};