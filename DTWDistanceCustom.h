#pragma once

#include "ISignalOp.h"

extern bool FileSave;

namespace SignalProcessing
{
	ref class DTWDistanceCustom : public Operation {
	private:
		unsigned int Neighborhood;
		String ^name;
	private:
		static double Distance(float p_1, float p_2) {
			return Math::Abs(p_1 - p_2);
		}
	public:
		DTWDistanceCustom(unsigned int neighborhood) {

			Neighborhood = neighborhood;
			this->Name = "DTWDistanceCustom";
		}

		property String ^Name {
			String ^get() {
				return name;
			}
			void set(String ^val) {
				name = val;
			}
		}



		virtual array<ViewSerialEvent> ^Apply(array<ViewSerialEvent> ^p_action)
		{
			return nullptr;
		}
		virtual array<double>^ Apply(array<array<float> ^> ^p_saved_action_definition, array<array<float> ^> ^p_last_actions, array<int> ^p_data_offsets) override
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

			array<KeyValuePair<int, int> ^> ^pDistancePath = gcnew array<KeyValuePair<int, int> ^>(saved_action_length * 2);
			array<float, 2> ^CMatrix = gcnew array<float, 2>(saved_action_length, saved_action_length);
/*
			for (int i = 0; i < saved_action_length; i++) {
				p_saved_action_definition[i][0] = i;

				p_last_actions[i][0] = 1;
			}*/
			
			for (i = 1; i < saved_action_length; i++) {
				for (j = 1; j < saved_action_length; j++) {
					CMatrix[j, i] = 999999;
				}
			}
			/*
			for (i = Neighborhood + 1, j = Neighborhood + 1; j < saved_action_length, i < last_action_length; i++, j++) {

				CMatrix[j - Neighborhood, i] = 999999;
				CMatrix[j, i - Neighborhood] = 999999;


			}*/

			for (int sdi = 0; sdi < p_data_offsets->Length; sdi++) {

				sensor_data_index_of_last = p_data_offsets[sdi];
				sensor_data_index_of_saved = sdi;

				CMatrix[0, 0] = Distance(p_saved_action_definition[0][sensor_data_index_of_saved], p_last_actions[0][sensor_data_index_of_last]);

				for (int i = 1; i < saved_action_length; i++) {

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

				for (diagonal_counter = 2; diagonal_counter < saved_action_length; diagonal_counter++) {


					for (j = diagonal_counter - 1; ((diagonal_counter - j < Neighborhood) && (j > 0) ); j--)
					{

						CMatrix[j, diagonal_counter] = Distance(p_saved_action_definition[j][sensor_data_index_of_saved], p_last_actions[diagonal_counter][sensor_data_index_of_last]);


						min_neighbourhood = CMatrix[j - 1, diagonal_counter - 1];

						if (CMatrix[j - 1, diagonal_counter] < min_neighbourhood) {
							min_neighbourhood = CMatrix[j - 1, diagonal_counter];
						}
						if (CMatrix[j, diagonal_counter - 1] < min_neighbourhood)
						{
							min_neighbourhood = CMatrix[j, diagonal_counter - 1];
						}

						CMatrix[j, diagonal_counter] = CMatrix[j, diagonal_counter] + min_neighbourhood;


					}

					for (i = diagonal_counter - 1; ((diagonal_counter - i < Neighborhood) && (i > 0)); i--) {

						CMatrix[diagonal_counter, i] = Distance(p_saved_action_definition[diagonal_counter][sensor_data_index_of_saved], p_last_actions[i][sensor_data_index_of_last]);


						min_neighbourhood = CMatrix[diagonal_counter - 1, i - 1];

						if (CMatrix[diagonal_counter - 1, i] < min_neighbourhood) {
							min_neighbourhood = CMatrix[diagonal_counter - 1, i];
						}
						if (CMatrix[diagonal_counter, i - 1] < min_neighbourhood)
						{
							min_neighbourhood = CMatrix[diagonal_counter, i - 1];
						}

						CMatrix[diagonal_counter, i] = CMatrix[diagonal_counter, i] + min_neighbourhood;

					}

					CMatrix[diagonal_counter, diagonal_counter] = Distance(p_saved_action_definition[diagonal_counter][sensor_data_index_of_saved], p_last_actions[diagonal_counter][sensor_data_index_of_last]);

					min_neighbourhood = CMatrix[diagonal_counter - 1, diagonal_counter - 1];

					if (CMatrix[diagonal_counter - 1, diagonal_counter] < min_neighbourhood) {
						min_neighbourhood = CMatrix[diagonal_counter - 1, diagonal_counter];
					}
					if (CMatrix[diagonal_counter, diagonal_counter - 1] < min_neighbourhood)
					{
						min_neighbourhood = CMatrix[diagonal_counter, diagonal_counter - 1];
					}

					CMatrix[diagonal_counter, diagonal_counter] = CMatrix[diagonal_counter, diagonal_counter] + min_neighbourhood;

				}
#if 0
					for (j = diagonal_counter - 1; j > 0; j--)
					{
						for (i = diagonal_counter -1; i > 0; i--) {

							min_neighbourhood = 999999.0f;

							

							min_neighbourhood = CMatrix[j - 1, i - 1];

							if (CMatrix[j - 1, i] < min_neighbourhood) {
								min_neighbourhood = CMatrix[j - 1, i];
							}
							if (CMatrix[j, i - 1] < min_neighbourhood)
							{
								min_neighbourhood = CMatrix[j, i - 1];
							}

							CMatrix[diagonal_counter, diagonal_counter] = CMatrix[diagonal_counter, diagonal_counter] + min_neighbourhood;

						}
					}
#endif


					j = saved_action_length - 1;
					i = saved_action_length - 1;

					int distance_counter = 0;
					int temp_i_decrement, temp_j_decrement;

					while (i >= 0 && j >= 0) {

						KeyValuePair<int, int> ^p_kvp = KeyValuePair<int, int>(j + 1, i + 1);
						pDistancePath[distance_counter] = p_kvp;
						distance_counter++;

						action_original_length[sdi] = action_original_length[sdi] + Math::Pow(p_saved_action_definition[j][sensor_data_index_of_saved], 2);

						temp_i_decrement = 1;
						temp_j_decrement = 1;

						//distance = distance + CMatrix[j, i];
						distance = distance + Math::Pow((p_saved_action_definition[j][sensor_data_index_of_saved] - p_last_actions[i][sensor_data_index_of_last]), 2);

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



					if (FileSave) {
						static int file = 0;
						FileSave = false;
						file++;


						String ^dir = gcnew String(Environment::CurrentDirectory + "\\SensorData\\Signals\\" + file.ToString());
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
				
				distances[sdi] = Math::Sqrt(distance / action_original_length[sdi]);
				distance = 0;
			}
			return distances;
		}
	};

}