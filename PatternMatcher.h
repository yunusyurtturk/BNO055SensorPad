#pragma once
#include "ActionDefinition.h"
#include "StandartDeviation.h"
#include "ZNormalize.h"
#include "RMS.h"
#include "MeanAndCenterOffMass.h"
public ref class PatternMatcher
{
private:
	static boolean is_eligible_for_matching(array<array<float> ^> ^p_action_patterns, array<array<float> ^> ^sub_last_actions,
		array<int> ^p_data_offsets, array<int> ^p_sub_last_actions_data_offsets)
	{

		boolean mean_check_pass = false;

		array<double> ^means;
		array<double> ^center_of_masses;
		array<double> ^std_devs;
		array<double> ^rms;
		array<double> ^rms_saved;


		array<double> ^saved_means;
		array<double> ^saved_center_of_masses;
		array<double> ^saved_std_devs;


		means = gcnew array<double>(p_data_offsets->Length);
		center_of_masses = gcnew array<double>(p_data_offsets->Length);
		std_devs = gcnew array<double>(p_data_offsets->Length);
		rms = gcnew array<double>(p_data_offsets->Length);
		rms_saved = gcnew array<double>(p_data_offsets->Length);

		saved_means = gcnew array<double>(p_data_offsets->Length);
		saved_center_of_masses = gcnew array<double>(p_data_offsets->Length);
		saved_std_devs = gcnew array<double>(p_data_offsets->Length);


		RMS::Calculate(p_action_patterns, 0, 0, 0, p_action_patterns->Length, rms_saved);
		RMS::Calculate(sub_last_actions,  0, 0, 0, sub_last_actions->Length, rms);


		double rms_ratio = rms_saved[0] / rms[0];
		if (rms_ratio < 1) {
			rms_ratio = (1 / rms_ratio);
		}

		if (rms[0] > 3 || rms_ratio < 2.5) {

			CalcMeansAndCenterOfMasses(sub_last_actions, p_sub_last_actions_data_offsets, 0, sub_last_actions->Length / 2, means, center_of_masses, std_devs);
			CalcMeansAndCenterOfMasses(p_action_patterns, p_data_offsets, 0, p_action_patterns->Length / 2, saved_means, saved_center_of_masses, saved_std_devs);

			RMS::Calculate(p_action_patterns, 0, 0, 0, p_action_patterns->Length / 2, rms_saved);
			RMS::Calculate(sub_last_actions, 0, 0, 0, sub_last_actions->Length / 2, rms);

			double min_mean_limit;
			double max_mean_limit;

			double min_rms_limit;
			double max_rms_limit;

			double min_std_limit;
			double max_std_limit;

			if (Math::Abs(saved_means[0]) < 6) {

				min_mean_limit = saved_means[0] - 3;
				max_mean_limit = saved_means[0] + 3;
			}
			else {
				min_mean_limit = saved_means[0] - Math::Abs(saved_means[0] / 2.0f);
				max_mean_limit = saved_means[0] + Math::Abs(saved_means[0] / 2.0f);
			}

			if (rms_saved[0] < 6) {

				min_rms_limit = rms_saved[0] - 3;
				max_rms_limit = rms_saved[0] + 3;
			}
			else {
				min_rms_limit = rms_saved[0] - (rms_saved[0] / 2.0f);
				max_rms_limit = rms_saved[0] + (rms_saved[0] / 2.0f);
			}

			if (saved_std_devs[0] < 6) {

				min_std_limit = saved_std_devs[0] - 3;
				max_std_limit = saved_std_devs[0] + 3;
			}
			else {
				min_std_limit = saved_std_devs[0] - (saved_std_devs[0] / 2.0f);
				max_std_limit = saved_std_devs[0] + (saved_std_devs[0] / 2.0f);
			}


	

			if (means[0] > min_mean_limit && means[0] < max_mean_limit) {

				if (rms[0] > min_rms_limit && rms[0] < max_rms_limit) {

					if (std_devs[0] > min_std_limit && std_devs[0] < max_std_limit) {
						mean_check_pass = true;
					}
				}
			}
			if (mean_check_pass == true) {
				CalcMeansAndCenterOfMasses(sub_last_actions, p_sub_last_actions_data_offsets, Math::Ceiling((sub_last_actions->Length / 2)), sub_last_actions->Length, means, center_of_masses, std_devs);
				CalcMeansAndCenterOfMasses(p_action_patterns, p_data_offsets, Math::Ceiling((p_action_patterns->Length / 2)), p_action_patterns->Length, saved_means, saved_center_of_masses, saved_std_devs);

				RMS::Calculate(p_action_patterns, 0, 0, Math::Ceiling(p_action_patterns->Length / 2), p_action_patterns->Length, rms_saved);
				RMS::Calculate(sub_last_actions, 0, 0, Math::Ceiling(sub_last_actions->Length / 2), sub_last_actions->Length, rms);

				if (Math::Abs(saved_means[0]) < 6) {

					min_mean_limit = saved_means[0] - 3;
					max_mean_limit = saved_means[0] + 3;
				}
				else {
					min_mean_limit = saved_means[0] - Math::Abs(saved_means[0] / 2.0f);
					max_mean_limit = saved_means[0] + Math::Abs(saved_means[0] / 2.0f);
				}

				if (rms_saved[0] < 6) {

					min_rms_limit = rms_saved[0] - 3;
					max_rms_limit = rms_saved[0] + 3;
				}
				else {
					min_rms_limit = rms_saved[0] - (rms_saved[0] / 2.0f);
					max_rms_limit = rms_saved[0] + (rms_saved[0] / 2.0f);
				}

				if (saved_std_devs[0] < 6) {

					min_std_limit = saved_std_devs[0] - 3;
					max_std_limit = saved_std_devs[0] + 3;
				}
				else {
					min_std_limit = saved_std_devs[0] - (saved_std_devs[0] / 2.0f);
					max_std_limit = saved_std_devs[0] + (saved_std_devs[0] / 2.0f);
				}

				if (means[0] < min_mean_limit || means[0] > max_mean_limit) {
					mean_check_pass = false;
				}

				if (rms[0] < min_rms_limit || rms[0] > max_rms_limit) {
					mean_check_pass = false;
				}

				if (std_devs[0] < min_std_limit || std_devs[0] > max_std_limit) {
					mean_check_pass = false;
				}


			}





			return mean_check_pass;
		}

		return mean_check_pass;
	}
public:


	static double Match(ActionDefinition ^p_action_definition, array<array<float> ^> ^p_last_actions)
	{
		double distance = 0;
		bool is_eligible = false;
		array<double> ^distances;
		array<double> ^means;
		array<double> ^center_of_masses;
		array<double> ^std_devs;
		array<double> ^rms;
		array<double> ^rms_saved;
		

		array<double> ^saved_means;
		array<double> ^saved_center_of_masses;
		array<double> ^saved_std_devs;

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
		rms = gcnew array<double>(p_data_offsets->Length);
		rms_saved = gcnew array<double>(p_data_offsets->Length);
		
		saved_means = gcnew array<double>(p_data_offsets->Length);
		saved_center_of_masses = gcnew array<double>(p_data_offsets->Length);
		saved_std_devs = gcnew array<double>(p_data_offsets->Length);

		for (int i = 0; i < p_action_original_patterns->Length; i++) {
			sub_last_actions[i] = gcnew array<float>(p_data_offsets->Length);

			for (int j = 0; j < p_data_offsets->Length; j++) {

				sub_last_actions[i][j] = p_last_actions[p_last_action_length - p_action_length + i][p_data_offsets[j]];
			}
		}

		for (int i = 0; i < p_data_offsets->Length; i++) {
			p_sub_last_actions_data_offsets[i] = i;
		}



		is_eligible = is_eligible_for_matching(p_action_patterns, sub_last_actions, p_data_offsets, p_sub_last_actions_data_offsets);

		

		for (int i = 0; i < distances->Length; i++) {

			distances[i] = 40;
		}


		if (is_eligible || (FileSave == true))
		{
				distances = p_action_definition->GetDistanceMethod()->Apply(p_action_patterns, sub_last_actions, p_sub_last_actions_data_offsets);

				if (distances[0] < FileSaveThreshold) {
					volatile int a = 1;
					a++;
				}

		}
#if 0
		if(std_devs[0] > 1  && (Math::Abs(saved_std_devs[0] - std_devs[0]) < 2)){
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
#endif

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

	static void CalcMeansAndCenterOfMasses(array<array<float> ^> ^p_action, array<int> ^p_data_offsets, int start_index, int length, array<double> ^means, array<double> ^center_of_masses, array<double> ^std_devs)
	{
		double mean;
		array<array<float> ^>^p_pattern = p_action;
		unsigned int data_index = 0;
		unsigned int pattern_length = p_action->Length;
		double cumulative_mass;
		double cumulative_weighted_mass;
		int SensorDataCount = p_data_offsets->Length;

		

		for (int i = 0; i < SensorDataCount; i++) {

			data_index = p_data_offsets[i];
			MeanAndCenterOffMass::Calculate(p_action, i, i, start_index, length, means, center_of_masses);
			StandartDeviation::Calculate(p_action, i, i, start_index, length, means, std_devs);

		}
	}
	static void CalcMeansAndCenterOfMasses(array<array<float> ^> ^p_last_actions, array<int> ^p_data_offsets, array<double> ^means, array<double> ^center_of_masses, array<double> ^std_devs)
	{
		CalcMeansAndCenterOfMasses(p_last_actions, p_data_offsets, 0, p_last_actions->Length, means, center_of_masses, std_devs);
	}


};