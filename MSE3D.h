#pragma once

#include "ISignalOp.h"

using namespace System;

namespace SignalProcessing
{
	ref class MSE3D : public Operation {
	private:
		unsigned int Order;
		String ^name;
	public:
		MSE3D() {

			this->Name = "MSE3D";
		}
		property String ^Name {
			String ^get() {
				return name;
			}
			void set(String ^val) {
				name = val;
			}
		}

		virtual array<ViewSerialEvent> ^Apply(array<ViewSerialEvent> ^p_action)
		{
			return nullptr;
		}
		virtual array<double>^ Apply(array<array<float> ^> ^p_saved_action_definition, array<array<float> ^> ^p_last_actions, array<int> ^p_data_offsets) override
		{
			array<double> ^distances = gcnew array<double>(p_data_offsets->Length);
			array<double> ^ mse = gcnew array<double>(p_data_offsets->Length);
			double action_original_length = 0;
			unsigned int sensor_data_index_of_saved = 0;
			unsigned int sensor_data_index_of_last = p_data_offsets[0];
			const unsigned int const saved_action_length	= p_saved_action_definition->Length;
			const unsigned int const last_action_length		= p_last_actions->Length;

	
			double sum_of_squares = 0.0f;

			for (int sdi = 0; sdi < p_data_offsets->Length; sdi++) {

				sum_of_squares = 0;

				action_original_length = 0;

				sensor_data_index_of_last = p_data_offsets[sdi];

				for (int i = 0; i < saved_action_length; i++) {

					
					sum_of_squares = sum_of_squares + Math::Pow((p_last_actions[last_action_length - i - 1][sensor_data_index_of_last] - p_saved_action_definition[saved_action_length - i - 1][sdi]), 2);
					
					action_original_length = action_original_length + Math::Pow(p_saved_action_definition[saved_action_length - i - 1][sensor_data_index_of_saved], 2);

				}

				mse[sdi] = sum_of_squares;
				
				distances[sdi] = Math::Sqrt(mse[sdi] / action_original_length);

			}
			
			return distances;
		}
	};

}