#pragma once

ref class StandartDeviation
{
public:

	static void Calculate(array<array<float>^> ^p_pattern, unsigned int result_index, unsigned int sdi, int start, int length, array<double> ^p_in_means, array<double> ^p_out_std_devs)
	{
		int pattern_length = p_pattern->Length;
		double std_dev = 0;
		unsigned int element_count = 0;

		for (int sample = start; sample < pattern_length && sample < start + length; sample++) {

			std_dev = std_dev + System::Math::Pow((p_pattern[sample][sdi] - p_in_means[result_index]), 2);
			element_count++;
		}

		std_dev = std_dev / (element_count - 1);

		p_out_std_devs[result_index] = Math::Sqrt(std_dev);
	}
	static void Calculate(array<array<float>^> ^p_pattern, unsigned int result_index, unsigned int sdi, array<double> ^p_in_means, array<double> ^p_out_std_devs)
	{
		Calculate(p_pattern, result_index, sdi, 0, p_pattern->Length, p_in_means, p_out_std_devs);
	}
};