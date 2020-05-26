#pragma once

ref class RMS
{
public:

	static void Calculate(array<array<float>^> ^p_pattern, unsigned int result_index, unsigned int sdi, int start, int length, array<double> ^p_out_rms)
	{
		int pattern_length = p_pattern->Length;
		double total_rms = 0;
		unsigned int element_count = 0;

		for (int sample = start; sample < pattern_length && sample < start + length; sample++) {

			total_rms = total_rms + System::Math::Pow(System::Math::Abs((p_pattern[sample][sdi])), 2);
			element_count++;
		}

		total_rms = total_rms / (element_count - 1);

		p_out_rms[result_index] = System::Math::Sqrt(total_rms);
	}
};