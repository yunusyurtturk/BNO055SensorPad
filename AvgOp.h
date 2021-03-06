#pragma once

#include "ISignalOp.h"

namespace SignalProcessing
{
	ref class AvgOp : public Operation {
	private:
		String ^name;
		unsigned int Order;
		unsigned int SensorDataIndex;
		double threshold;
	public:
		AvgOp(unsigned int order, unsigned int sensor_data_index) {

			SensorDataIndex = sensor_data_index;

			if (Order > 1) {
				Order = order;
			}
			else {
				Order = 2;
			}
		}

		property double Threshold {
			double get() {
				return threshold;
			}
			void set(double val) {
				threshold = val;
			}
		}

		property String ^Name {
			String ^get() {
				return name;
			}
			void set(String ^val) {
				name = val;
			}
		}

		virtual void SetName(String ^p_name)
		{
			this->Name = p_name;
		}
		virtual String ^GetName()
		{
			return this->Name;
		}

		virtual array<array<float>^> ^ PreprocessActionTemplate(array<array<float> ^> ^p_saved_action_definition, array<int> ^p_sdi)
		{
			return p_saved_action_definition;
		}

		virtual void SetFileSaveEnabled(boolean val, double threshold)
		{
			Threshold = threshold;
			//FileSave = true;
		}

		virtual array<double> ^ Apply(array<array<float> ^> ^p_saved_action_definition, array<array<float> ^> ^p_last_actions, array<int> ^p_data_offsets) override
		{
			return nullptr;
		}
		virtual array<ViewSerialEvent> ^Apply(array<ViewSerialEvent> ^p_action) override
		{
			array<ViewSerialEvent> ^p_ret_array = gcnew array<ViewSerialEvent>(p_action->Length - Order);

			float sum_temp;
			for (int i = Order; i < p_action->Length; i++) {
		
					sum_temp = 0;
					for (int j = 0; j < Order; j++) {
						sum_temp += p_action[i - j].pEvent->euler[SensorDataIndex];
					}
					sum_temp = sum_temp / Order;
					ViewSerialEvent view_event(0);
					p_ret_array[i - Order] = view_event;
					p_ret_array[i - Order].pEvent->euler[SensorDataIndex] = sum_temp;
			
			}

			return p_ret_array;
		}
	};

}