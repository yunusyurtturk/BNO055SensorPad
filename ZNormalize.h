#pragma once

ref class ZNormalize
{
public:
	static void Normalize(array<array<float>^> ^p_pattern, array<array<float>^> ^p_out_pattern, array<int> ^input_sdi, array<int> ^output_sdi, array<double>^mean, array<double> ^std_dev)
	{
		int pattern_length = p_pattern->Length;

		for (int i = 0; i < input_sdi->Length; i++) {

			unsigned int sensor_data_index = input_sdi[i];
			unsigned int output_sensor_data_index = output_sdi[i];

			for (int sample = 0; sample < pattern_length; sample++) {

				p_out_pattern[sample][output_sensor_data_index] = (p_pattern[sample][sensor_data_index] - mean[i]) / std_dev[i];
			}
		}
	}

	static void Normalize(array<float> ^p_pattern, array<float> ^p_out_pattern, unsigned int sdi, float mean, float std_dev)
	{
		int pattern_length = p_pattern->Length;

		for (int sample = 0; sample < pattern_length; sample++) {

			p_out_pattern[sample] = (p_pattern[sample] - mean) / std_dev;
		}
	}
};