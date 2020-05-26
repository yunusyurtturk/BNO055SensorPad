#pragma once

ref class MeanAndCenterOffMass
{
public:
	static void Calculate(array<array<float>^> ^p_pattern, unsigned int result_index, unsigned int sdi, int start_index, int length, array<double> ^p_out_mean, array<double> ^p_out_center_off_mass)
	{
		int pattern_length = p_pattern->Length;
		double mean = 0;
		double cumulative_weighted_mass = 0;
		double cumulative_mass = 0;
		unsigned int element_count = 0;

		for (int sample = start_index; sample < pattern_length && sample < (start_index + length); sample++) {

			mean = mean + p_pattern[sample][sdi];

			cumulative_weighted_mass = cumulative_weighted_mass + (sample + 1) * p_pattern[sample][sdi];

			cumulative_mass = cumulative_mass + p_pattern[sample][sdi];
			element_count++;
		}




		p_out_center_off_mass[result_index] = (cumulative_weighted_mass / cumulative_mass);

		mean = mean / element_count;	// Burada element_count kullaniyoruz cunku length ifadesi array boyutundan fazla girilmis olabilir
		p_out_mean[result_index] = mean;
	}
	static void Calculate(array<array<float>^> ^p_pattern, unsigned int result_index, unsigned int sdi, array<double> ^p_out_mean, array<double> ^p_out_center_off_mass)
	{
		Calculate(p_pattern, result_index, sdi, 0, p_pattern->Length, p_out_mean, p_out_center_off_mass);
	}
	
	static void CenteredCalculate(array<array<float>^> ^p_pattern, unsigned int result_index, unsigned int sdi, array<double> ^p_out_mean, array<double> ^p_out_center_off_mass)
	{
		int pattern_length = p_pattern->Length;
		double mean = 0;
		double cumulative_weighted_mass = 0;
		double cumulative_mass = 0;

		for (int sample = 0; (sample < pattern_length && sample < pattern_length / 2); sample++) {

			mean = mean + p_pattern[sample][sdi];
			mean = mean + p_pattern[pattern_length - sample - 1][sdi];

			cumulative_weighted_mass = cumulative_weighted_mass + (sample - pattern_length / 2) * p_pattern[sample][sdi];
			cumulative_weighted_mass = cumulative_weighted_mass - (pattern_length / 2 - sample) * p_pattern[pattern_length - sample - 1][sdi];

			cumulative_mass = cumulative_mass + p_pattern[sample][sdi];
			cumulative_mass = cumulative_mass + p_pattern[pattern_length - sample - 1][sdi];
		}

		if (pattern_length % 2 != 0) {
			unsigned int middle_sample_index = pattern_length / 2; // int division
			mean = mean + p_pattern[middle_sample_index][sdi];
			cumulative_mass = cumulative_mass + p_pattern[middle_sample_index][sdi];
		}

		p_out_center_off_mass[result_index] = (cumulative_weighted_mass / cumulative_mass);

		mean = mean / pattern_length;
		p_out_mean[result_index] = mean;
	}
};