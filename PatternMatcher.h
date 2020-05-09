#pragma once
#include "ActionDefinition.h"


public ref class PatternMatcher
{
public:


	static double Match(ActionDefinition ^p_action_definition, array<array<float> ^> ^p_last_actions)
	{
		double distance = 0;
		array<double> ^distances;
		array<double> ^means;
		array<double> ^center_of_masses;

		array<array<float> ^> ^p_action_patterns = p_action_definition->GetPattern();
		array<int> ^p_data_offsets = p_action_definition->GetSensorDataIndexs();
		
		unsigned int p_action_length = p_action_patterns->Length;
		unsigned int p_last_action_length = p_last_actions->Length;
		array<array<float> ^> ^sub_last_actions =  gcnew array<array<float> ^>(p_action_length);

		bool is_match = false;

		distances = gcnew array<double>(p_data_offsets->Length);
		means = gcnew array<double>(p_data_offsets->Length);
		center_of_masses = gcnew array<double>(p_data_offsets->Length);
		
		p_last_actions = SignalProcessing::Normalizer::Normalize(p_last_actions, p_data_offsets);

		for (int i = 0; i < p_action_patterns->Length; i++) {
			sub_last_actions[i] = p_last_actions[p_last_action_length - p_action_length + i];
		}
		//distance = p_action_definition->GetMSEDistancer()->Apply(p_action_patterns, p_last_actions, p_data_offsets);
		distances = p_action_definition->GetDistanceMethod()->Apply(p_action_patterns, sub_last_actions, p_data_offsets);

		CalcMeansAndCenterOfMasses(sub_last_actions, p_data_offsets, means, center_of_masses);


		for (int i = 0; i < p_data_offsets->Length; i++) {

			is_match = false;
			if (Math::Abs((p_action_definition->GetCenterOffMass(i) - center_of_masses[i])) < (p_action_length * 0.20)) {
				if (Math::Abs((p_action_definition->GetMean(i) - means[i])) < (p_action_definition->GetMean(i) * 0.15)) {
					is_match = true;
				}
			}
			
			distance = distance + (distances[i] * (1/ (float)p_data_offsets->Length));

			if (!is_match) {
			//	distance = (distance + 1) * 10;
			}
			else {
				distance = distance;
			}
		}
		
		return distance;
	//	return distance * Math::Pow(10, (p_data_offsets->Length - 1)) / Math::Pow(2, (p_data_offsets->Length - 1));

	}
private:
	static void CalcMeansAndCenterOfMasses(array<array<float> ^> ^p_last_actions, array<int> ^p_data_offsets, array<double> ^means, array<double> ^center_of_masses)
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
#if 0
			mean = 0.0f;
			cumulative_mass = 0;
			cumulative_weighted_mass = 0;
			data_index = p_data_offsets[i];

			for (int sample = 0; sample < pattern_length; sample++) {

				mean = mean + p_pattern[sample][data_index];
				cumulative_weighted_mass = cumulative_weighted_mass + sample * p_pattern[sample][data_index];
				cumulative_mass = cumulative_mass + p_pattern[sample][data_index];
			}

			center_of_masses[i] = (cumulative_weighted_mass / cumulative_mass);

			if (center_of_masses[i] < 0) {
				int a = 1;
				a++;
			}

			mean = mean / pattern_length;
			means[i] = mean;
#endif
		}
	}


};