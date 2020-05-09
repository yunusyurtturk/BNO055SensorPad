#pragma once


ref class SensorValueOps
{
public:
	static float RMS(float values[], unsigned int count) {

		float sum = 0;
		for (int i = 0; i < count; i++) {
			sum += values[i] * values[i];
		}

		sum = sum / count;

		return System::Math::Sqrt(sum);
	}

};