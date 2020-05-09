#pragma once

#include "ViewTypes.h"
#include "ISignalOp.h"

#define OP_CHAIN_MAX_OPERATION_LENGTH 10


namespace SignalProcessing
{
	ref class OpChain {

		unsigned int cursor;
		array<Operation ^> ^pChain;

	public:
		OpChain() {
			cursor = 0;
			pChain = gcnew array<Operation ^>(OP_CHAIN_MAX_OPERATION_LENGTH);
		}

		void InsertOp(Operation ^p_val) {

			if (cursor < OP_CHAIN_MAX_OPERATION_LENGTH) {

				pChain[cursor] = p_val;
				cursor++;
			}
			
		}


		double Apply(double new_sample, unsigned int sensor_data_type_index, array<ViewSerialEvent> ^p_flow) {

			double ret_val = 0;
			for (int i = 0; i < pChain->Length; i++) {

				if (pChain[i] != nullptr) {

				//	ret_val = pChain[i]->Apply(new_sample, sensor_data_type_index, p_flow);
				}
			}

			return ret_val;

		}
	};

}