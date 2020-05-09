#pragma once

#include "ViewTypes.h"

namespace SignalProcessing
{
	interface class Operation {

		virtual array<double>^ Apply(array<array<float> ^> ^p_saved_action_definition, array<array<float> ^> ^p_last_actions, array<int> ^p_data_offsets);
	};

}