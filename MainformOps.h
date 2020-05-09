#pragma once
#include "ViewTypes.h"
#include "ISignalOp.h"
public interface class IMainFormOps
{
public:

	virtual void AddPattern(int action, array<array<float> ^> ^pattern, array<int> ^sensor_data_offsets, double threshold, SignalProcessing::Operation ^p_distance_method);

};