#pragma once


#include "ISignalOp.h"

namespace SignalProcessing
{
	ref class DiffOp : public Operation {
	private:
		unsigned int Order;
		unsigned int SensorDataIndex;
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