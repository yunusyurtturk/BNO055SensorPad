#pragma once

#include "ISignalOp.h"

namespace SignalProcessing
{
	ref class Normalizer  {
	private:
		
	public:
		Normalizer() {

			
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