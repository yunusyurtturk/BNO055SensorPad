#pragma once

#include "ViewTypes.h"

namespace SignalProcessing
{
	interface class Operation {

		virtual void SetName(String ^p_name);
		virtual String ^GetName();
		virtual array<array<float>^> ^ PreprocessActionTemplate(array<array<float> ^> ^p_saved_action_definition, array<int> ^p_sdi);
		virtual void SetFileSaveEnabled(boolean val, double threshold);
		virtual array<double>^ Apply(array<array<float> ^> ^p_saved_action_definition, array<array<float> ^> ^p_last_actions, array<int> ^p_data_offsets);
	};

}