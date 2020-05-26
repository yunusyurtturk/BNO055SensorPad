#pragma once

#include "ISignalOp.h"

namespace SignalProcessing
{
	ref class Normalizer  {
	private:
		
	public:
		Normalizer() {

			
		}
		static array<array<float> ^> ^ NormalizeBetween(array<array<float> ^> ^p_saved_action_definition, array<int> ^p_sdi, double norm_min, double norm_max)
		{
			array<array<float> ^> ^p_retval = gcnew array<array<float> ^>(p_saved_action_definition->Length);
			double max = -999999.0f;
			double min = 999999.0f;

			array<double> ^maximums = gcnew array<double>(p_sdi->Length);
			array<double> ^minimums = gcnew array<double>(p_sdi->Length);

			unsigned int saved_action_length = p_saved_action_definition->Length;

			for (int i = 0; i < p_saved_action_definition->Length; i++) {

				p_retval[i] = gcnew array<float>(p_sdi->Length);
			}

			for (int sdi = 0; sdi < p_sdi->Length; sdi++)
			{
				max = -999999.0f;
				min = 999999.0f;
				for (int i = 0; i < saved_action_length; i++) {

					if (p_saved_action_definition[i][sdi] < min) {
						min = p_saved_action_definition[i][sdi];
					}
					if (p_saved_action_definition[i][sdi] > max) {
						max = p_saved_action_definition[i][sdi];
					}

				}

				maximums[sdi] = max;
				minimums[sdi] = min;
			}

			for (int sdi = 0; sdi < p_sdi->Length; sdi++)
			{
				for (int i = 0; i < saved_action_length; i++) {

					p_retval[i][sdi] = norm_min + (p_saved_action_definition[i][sdi] - minimums[sdi]) * ((norm_max - norm_min) / (maximums[sdi] - minimums[sdi]));
				}

			}

			return p_retval;
		}
		static array<array<float> ^> ^ Normalize(array<array<float> ^> ^p_pattern, array<int> ^p_sdi)
		{
			array<array<float> ^> ^p_normalized_pattern = gcnew array<array<float> ^>(p_pattern->Length);


			array<float> ^max = gcnew array<float>(p_sdi->Length);
			array<float> ^min = gcnew array<float>(p_sdi->Length);

			for (int i = 0; i < p_sdi->Length; i++) {

				max[i] = -9999;
				min[i] = 9999;
			}



			for (int sdi = 0; sdi < p_sdi->Length; sdi++) {

				int data_index = p_sdi[sdi];

				for (int i = 0; i < p_pattern->Length; i++) {

					if (p_pattern[i][data_index] < min[sdi]) {
						min[sdi] = p_pattern[i][data_index];
					}

					if (p_pattern[i][data_index] > max[sdi]) {
						max[sdi] = p_pattern[i][data_index];
					}


				}
			}

			for (int i = 0; i < p_pattern->Length; i++) {

				p_normalized_pattern[i] = gcnew array<float>(p_sdi->Length);

			}

			for (int sdi = 0; sdi < p_sdi->Length; sdi++) {

				int data_index = p_sdi[sdi];

				float diff = max[sdi] - min[sdi];

				if (diff < 0.0001) {
					diff = 0.0001;
				}

				for (int i = 0; i < p_pattern->Length; i++) {

					p_normalized_pattern[i][sdi] = (p_pattern[i][data_index] - min[sdi]) / (diff);
				}
			}

			return p_normalized_pattern;

		}
	
	};

}