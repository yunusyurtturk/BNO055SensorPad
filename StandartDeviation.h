#pragma once

ref class StandartDeviation
{
public:
	static void Calculate(array<array<float>^> ^p_pattern, unsigned int result_index, unsigned int sdi, array<double> ^p_in_means, array<double> ^p_out_std_devs)
	{
		int pattern_length = p_pattern->Length;
		double std_dev = 0;

		for (int sample = 0; sample < pattern_length; sample++) {

			std_dev = std_dev + System::Math::Pow((p_pattern[sample][sdi] - p_in_means[result_index]), 2);
		}

		std_dev = std_dev / pattern_length;

		p_out_std_devs[result_index] = Math::Sqrt(std_dev);
	}
};