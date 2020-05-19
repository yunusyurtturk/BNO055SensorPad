#pragma once


#include "ISignalOp.h"

namespace SignalProcessing
{
	ref class DiffOp : public Operation {
	private:
		String ^name;
		unsigned int Order;
		unsigned int SensorDataIndex;
		double threshold;
	public:
		DiffOp(unsigned int order, unsigned int sensor_data_index){

			SensorDataIndex = sensor_data_index;

			if (Order > 0) {
				Order = order; 
			}
			else {
				Order = 1;
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

		virtual array<array<float>^> ^ PreprocessActionTemplate(array<array<float> ^> ^p_saved_action_definition, array<int> ^p_sdi)
		{
			return p_saved_action_definition;
		}

		virtual virtual void SetFileSaveEnabled(boolean val, double threshold)
		{
			Threshold = threshold;
			//FileSave = true;
		}

		virtual void SetName(String ^p_name)
		{
			this->Name = p_name;
		}
		virtual String ^GetName()
		{
			return this->Name;
		}

		virtual array<double> ^ Apply(array<array<float> ^> ^p_saved_action_definition, array<array<float> ^> ^p_last_actions, array<int> ^p_data_offsets) override
		{
			return nullptr;
		}
		virtual array<ViewSerialEvent> ^Apply(array<ViewSerialEvent> ^p_action) override
		{
			array<ViewSerialEvent> ^p_ret_array = gcnew array<ViewSerialEvent>(p_action->Length - Order);

			for (int order = 0; order < Order; order++) {

				for (int i = Order - order; i < p_action->Length; i++) {

					ViewSerialEvent pEvent(0);
					p_ret_array[i - Order] = pEvent;
					p_ret_array[i - Order].pEvent->euler[SensorDataIndex] = p_action[i].pEvent->euler[SensorDataIndex] - p_action[i - 1].pEvent->euler[SensorDataIndex];
				}
			}

			return p_ret_array;
			

		}
	};

}