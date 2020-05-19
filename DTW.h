#pragma once
#include "ViewTypes.h"


using namespace System;
using namespace System::Collections::Generic;

ref class DTW
{
public:
	static double Distance(float p_1, float p_2) {
		return Math::Abs(p_1 - p_2);
	}

	static array<double>^ Apply(array<array<float> ^> ^p_saved_action_definition, array<array<float> ^> ^p_last_actions, array<int> ^p_data_offsets)
	{
		double distance = 0;
		unsigned int sensor_data_index_of_saved = 0;
		unsigned int sensor_data_index_of_last = p_data_offsets[0];
		unsigned int diagonal_counter = 0;
		float min_neighbourhood;
		const unsigned int const saved_action_length = p_saved_action_definition->Length;
		const unsigned int const last_action_length = p_last_actions->Length;
		unsigned int i, j;

		array<double> ^distances = gcnew array<double>(p_data_offsets->Length);
		array<double> ^action_original_length = gcnew array<double>(p_data_offsets->Length);

		array<KeyValuePair<int, int> ^> ^pDistancePath = gcnew array<KeyValuePair<int, int> ^>(saved_action_length * last_action_length);
		array<float, 2> ^CMatrix = gcnew array<float, 2>(saved_action_length, last_action_length);
		/*
		for (int i = 0; i < saved_action_length; i++) {
		p_saved_action_definition[i][0] = i;

		p_last_actions[i][0] = 1;
		}*/
		/*
		for (i = 1; i < saved_action_length; i++) {
		for (j = 1; j < saved_action_length; j++) {
		CMatrix[j, i] = 999999;
		}
		}
		*/
		/*
		for (i = Neighborhood + 1, j = Neighborhood + 1; j < saved_action_length, i < last_action_length; i++, j++) {

		CMatrix[j - Neighborhood, i] = 999999;
		CMatrix[j, i - Neighborhood] = 999999;


		}*/

		for (int sdi = 0; sdi < p_data_offsets->Length; sdi++) {

			sensor_data_index_of_last = p_data_offsets[sdi];
			sensor_data_index_of_saved = sdi;

			//sensor_data_index_of_last = sdi;

			CMatrix[0, 0] = Distance(p_saved_action_definition[0][sensor_data_index_of_saved], p_last_actions[0][sensor_data_index_of_last]);

			for (int i = 1; i < last_action_length; i++) {

				CMatrix[0, i] = Distance(p_saved_action_definition[0][sensor_data_index_of_saved], p_last_actions[i][sensor_data_index_of_last]) + CMatrix[0, i - 1];

			}

			for (int j = 1; j < saved_action_length; j++) {

				CMatrix[j, 0] = Distance(p_saved_action_definition[j][sensor_data_index_of_saved], p_last_actions[0][sensor_data_index_of_last]) + CMatrix[j - 1, 0];

			}

			/* C[0, 0] */
			CMatrix[0, 0] = Distance(p_saved_action_definition[0][sensor_data_index_of_saved], p_last_actions[0][sensor_data_index_of_last]);
			/* END - C[0, 0] */

			/* C[1, 1] */
			CMatrix[1, 1] = Distance(p_saved_action_definition[1][sensor_data_index_of_saved], p_last_actions[1][sensor_data_index_of_last]);

			min_neighbourhood = CMatrix[0, 0];

			if (CMatrix[0, 1] < min_neighbourhood) {
				min_neighbourhood = CMatrix[0, 1];
			}
			if (CMatrix[1, 0] < min_neighbourhood)
			{
				min_neighbourhood = CMatrix[1, 0];
			}

			CMatrix[1, 1] = CMatrix[1, 1] + min_neighbourhood;

			/* END - C[1, 1] */

			diagonal_counter = saved_action_length;

			for (j = 1; j < saved_action_length; j++)
			{
				for (i = 1; i < last_action_length; i++)
				{

					CMatrix[j, i] = Distance(p_saved_action_definition[j][sensor_data_index_of_saved], p_last_actions[i][sensor_data_index_of_last]);

					min_neighbourhood = CMatrix[j - 1, i - 1];

					if (CMatrix[j - 1, i] < min_neighbourhood) {
						min_neighbourhood = CMatrix[j - 1, i];
					}
					if (CMatrix[j, i - 1] < min_neighbourhood)
					{
						min_neighbourhood = CMatrix[j, i - 1];
					}

					CMatrix[j, i] = CMatrix[j, i] + min_neighbourhood;

				}
			}

			j = saved_action_length - 1;
			i = last_action_length - 1;

			int distance_counter = 0;
			int temp_i_decrement, temp_j_decrement;
			int dtw_range = saved_action_length + saved_action_length * 0.30f;
			double dtw_distance = 0;

			while (i >= 0 && j >= 0) {

				KeyValuePair<int, int> ^p_kvp = KeyValuePair<int, int>(j + 1, i + 1);
				pDistancePath[distance_counter] = p_kvp;
				distance_counter++;

				if (distance_counter > dtw_range) {

					//distance = 9999999;
					//break;
				}

				action_original_length[sdi] = action_original_length[sdi] + Math::Pow(p_saved_action_definition[j][sensor_data_index_of_saved], 2);

				temp_i_decrement = 1;
				temp_j_decrement = 1;

				//distance = distance + CMatrix[j, i];
				dtw_distance = dtw_distance + Math::Pow((p_saved_action_definition[j][sensor_data_index_of_saved] - p_last_actions[i][sensor_data_index_of_last]), 2);

				if (i != 0 && j != 0) {

					min_neighbourhood = CMatrix[j - 1, i - 1];

					if (CMatrix[j, i - 1] < min_neighbourhood) {
						min_neighbourhood = CMatrix[j, i - 1];
						temp_i_decrement = 1;
						temp_j_decrement = 0;
					}

					if (CMatrix[j - 1, i] < min_neighbourhood) {
						min_neighbourhood = CMatrix[j - 1, i];
						temp_i_decrement = 0;
						temp_j_decrement = 1;
					}
				}
				else if (i == 0 && j == 0) {
					break;
				}
				else if (i == 0) {

					min_neighbourhood = CMatrix[j - 1, i];
					temp_i_decrement = 0;
					temp_j_decrement = 1;

				}
				else if (j == 0) {

					min_neighbourhood = CMatrix[j, i - 1];
					temp_i_decrement = 1;
					temp_j_decrement = 0;
				}





				i = i - temp_i_decrement;
				j = j - temp_j_decrement;

			}

			distance = distance + dtw_distance * ((sdi + 1) / (float)p_data_offsets->Length);



			if (FileSave) {

				String ^dir = gcnew String(Environment::CurrentDirectory + "\\SensorData\\Signals\\" + SaveDirIndex.ToString() + "DTW");
				if (!IO::Directory::Exists(dir)) {

					IO::Directory::CreateDirectory(dir);
				}
				/****************************/
				String ^path = gcnew String(dir + "\\saved_action.txt");

				System::IO::TextWriter ^pTextWriter;
				if (!IO::File::Exists(path)) {

					pTextWriter = gcnew System::IO::StreamWriter(path, true);
					pTextWriter->Flush();

				}
				for (j = 0; j < saved_action_length; j++) {
					pTextWriter->Write(p_saved_action_definition[saved_action_length - j - 1][sensor_data_index_of_saved] + "\t");
				}
				pTextWriter->Flush();
				pTextWriter->Close();

				/****************************/
				path = gcnew String(dir + "\\last_action.txt");


				if (!IO::File::Exists(path)) {

					pTextWriter = gcnew System::IO::StreamWriter(path, true);
					pTextWriter->Flush();

				}
				for (j = 0; j < saved_action_length; j++) {
					pTextWriter->Write(p_last_actions[last_action_length - j - 1][sensor_data_index_of_last] + "\t");
				}
				pTextWriter->Flush();
				pTextWriter->Close();

				/************DISTANCE****************/
				path = gcnew String(dir + "\\distance.txt");


				if (!IO::File::Exists(path)) {

					pTextWriter = gcnew System::IO::StreamWriter(path, true);
					pTextWriter->Flush();

				}
				for (j = 0; j < saved_action_length; j++) {
					pTextWriter->Write(distance + "\t");
				}
				pTextWriter->Flush();
				pTextWriter->Close();

				/****************************/
				path = gcnew String(dir + "\\distance_path_ij.txt");


				if (!IO::File::Exists(path)) {

					pTextWriter = gcnew System::IO::StreamWriter(path, true);
					pTextWriter->Flush();

				}

				for (int i = 0; i < pDistancePath->Length; i++) {
					if (pDistancePath[i] != nullptr) {

						pTextWriter->Write(pDistancePath[i]->Key + "\t");

					}
				}
				pTextWriter->Flush();
				pTextWriter->Close();

				/****************************/
				path = gcnew String(dir + "\\distance_path_ix.txt");


				if (!IO::File::Exists(path)) {

					pTextWriter = gcnew System::IO::StreamWriter(path, true);
					pTextWriter->Flush();

				}

				for (int i = 0; i < pDistancePath->Length; i++) {
					if (pDistancePath[i] != nullptr) {

						pTextWriter->Write(pDistancePath[i]->Value + "\t");

					}
				}
				pTextWriter->Flush();
				pTextWriter->Close();


				/****************************/
				path = gcnew String(dir + "\\CMatrix.txt");

				if (!IO::File::Exists(path)) {

					pTextWriter = gcnew System::IO::StreamWriter(path, true);
					pTextWriter->Flush();

				}
				for (j = 0; j < saved_action_length; j++) {
					for (i = 0; i < saved_action_length; i++) {

						pTextWriter->Write(CMatrix[j, i] + "\t");
					}

					pTextWriter->WriteLine();
				}

				pTextWriter->Flush();
				pTextWriter->Close();
			}

			distances[sdi] = Math::Sqrt(distance);
			distance = 0;
		}
		return distances;
	}


};