#pragma once
#include "ActionDefinition.h"
#include "StandartDeviation.h"
#include "ZNormalize.h"
public ref class PatternMatcher
{
public:


	static double Match(ActionDefinition ^p_action_definition, array<array<float> ^> ^p_last_actions)
	{
		double distance = 0;
		array<double> ^distances;
		array<double> ^means;
		array<double> ^center_of_masses;
		array<double> ^std_devs;

		array<array<float> ^> ^p_action_original_patterns = p_action_definition->GetOriginalPattern();
		array<array<float> ^> ^p_action_patterns = p_action_definition->GetPattern();
		array<int> ^p_data_offsets = p_action_definition->GetSensorDataIndexs();
		array<int> ^p_sub_last_actions_data_offsets = gcnew array<int>(p_data_offsets->Length);

		unsigned int p_action_length = p_action_original_patterns->Length;
		unsigned int p_last_action_length = p_last_actions->Length;
		array<array<float> ^> ^sub_last_actions =  gcnew array<array<float> ^>(p_action_length);

		bool is_match = false;

		distances = gcnew array<double>(p_data_offsets->Length);
		means = gcnew array<double>(p_data_offsets->Length);
		center_of_masses = gcnew array<double>(p_data_offsets->Length);
		std_devs = gcnew array<double>(p_data_offsets->Length);
		
		//p_last_actions = SignalProcessing::Normalizer::Normalize(p_last_actions, p_data_offsets);

		
		for (int i = 0; i < p_action_original_patterns->Length; i++) {
			sub_last_actions[i] = gcnew array<float>(p_data_offsets->Length);

			for (int j = 0; j < p_data_offsets->Length; j++) {

				sub_last_actions[i][j] = p_last_actions[p_last_action_length - p_action_length + i][p_data_offsets[j]];
			}
		}

		for (int i = 0; i < p_data_offsets->Length; i++) {
			p_sub_last_actions_data_offsets[i] = i;
		}

	//	CalcMeansAndCenterOfMasses(sub_last_actions, p_sub_last_actions_data_offsets, means, center_of_masses, std_devs);

	
	//	ZNormalize::Normalize(sub_last_actions, sub_last_actions, p_sub_last_actions_data_offsets, p_sub_last_actions_data_offsets, means, std_devs);

		CalcMeansAndCenterOfMasses(sub_last_actions, p_sub_last_actions_data_offsets, means, center_of_masses, std_devs);

		if(std_devs[0] > 1){
		//distance = p_action_definition->GetMSEDistancer()->Apply(p_action_patterns, p_last_actions, p_data_offsets);
			distances = p_action_definition->GetDistanceMethod()->Apply(p_action_patterns, sub_last_actions, p_sub_last_actions_data_offsets);
		}
		else {

			for (int i = 0; i < distances->Length; i++) {

				distances[i] = 44;
			}
		}
		if (distances[0] < 0.2) {
			volatile int a = 1;
			a++;
		}


		for (int i = 0; i < p_sub_last_actions_data_offsets->Length; i++) {

			is_match = false;
			if (Math::Abs((p_action_definition->GetStandartDeviation(i) - std_devs[i])) < 0.2) {
				//if (Math::Abs((p_action_definition->GetMean(i) - means[i])) < (p_action_definition->GetMean(i) * 0.15)) {
				is_match = true;
				//}
			}
#if 0
			if (!is_match) {
				distance = (distance * 5);
			}
			else {
				distance = distance;
			}
#endif
			distance = distance + (distances[i] * (1 / (float)p_sub_last_actions_data_offsets->Length));


		}
		
		
		return distance;
	//	return distance * Math::Pow(10, (p_data_offsets->Length - 1)) / Math::Pow(2, (p_data_offsets->Length - 1));

	}
private:
	static void CalcMeansAndCenterOfMasses(array<array<float> ^> ^p_last_actions, array<int> ^p_data_offsets, array<double> ^means, array<double> ^center_of_masses, array<double> ^std_devs)
	{
		double mean;
		array<array<float> ^>^p_pattern = p_last_actions;
		unsigned int data_index = 0;
		unsigned int pattern_length = p_last_actions->Length;
		double cumulative_mass;
		double cumulative_weighted_mass;
		int SensorDataCount = p_data_offsets->Length;

		for (int i = 0; i < SensorDataCount; i++) {

			data_index = p_data_offsets[i];
			MeanAndCenterOffMass::Calculate(p_last_actions, i, i, means, center_of_masses);
			StandartDeviation::Calculate(p_last_actions, i, i, means, std_devs);

		}
	}


};