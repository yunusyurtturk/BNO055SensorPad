#pragma once

#include "ISignalOp.h"

extern bool FileSave;
extern unsigned int SaveDirIndex;


using namespace System;

namespace SignalProcessing
{
	ref class SymbolicDiff : public Operation {
	private:
		unsigned int Order;
		String ^name;
		double threshold;


	public:
		SymbolicDiff() {

			Order = 4;
			this->Name = "SymbolicDiff";


		}
		property String ^Name {
			String ^get() {
				return name;
			}
			void set(String ^val) {
				name = val;
			}
		}

		property double Threshold {
			double get() {
				return threshold;
			}
			void set(double val) {
				threshold = val;
			}
		}

		virtual void SetName(String ^p_name)
		{
			this->Name = p_name;
		}
		virtual String ^GetName()
		{
			return this->Name;
		}

		virtual virtual void SetFileSaveEnabled(boolean val, double threshold)
		{
			Threshold = threshold;
			FileSave = true;
		}

		virtual array<array<float>^> ^ PreprocessActionTemplate(array<array<float> ^> ^p_saved_action_definition, array<int> ^p_sdi)
		{
			p_saved_action_definition = SymbolicDiff::Normalize(p_saved_action_definition, p_sdi, -1, 1);

			return SymbolicDiff::Symbolize(p_saved_action_definition, p_sdi, 4);
		}

		static array<array<float> ^> ^ Normalize(array<array<float> ^> ^p_saved_action_definition, array<int> ^p_sdi, double norm_min, double norm_max)
		{
			array<array<float> ^> ^p_retval = gcnew array<array<float> ^>(p_saved_action_definition->Length);
			double max = -999999.0f;
			double min = 999999.0f;

			array<double> ^maximums = gcnew array<double>(p_sdi->Length);
			array<double> ^minimums = gcnew array<double>(p_sdi->Length);

			unsigned int saved_action_length = p_saved_action_definition->Length;

			for (int i = 0; i < p_saved_action_definition->Length; i++) {

				p_retval[i] = gcnew array<float>(p_sdi->Length);
			}

			for (int sdi = 0; sdi < p_sdi->Length; sdi++)
			{
				max = -999999.0f;
				min = 999999.0f;
				for (int i = 0; i < saved_action_length; i++) {

					if (p_saved_action_definition[i][sdi] < min) {
						min = p_saved_action_definition[i][sdi];
					}
					if (p_saved_action_definition[i][sdi] > max) {
						max = p_saved_action_definition[i][sdi];
					}
					
				}

				maximums[sdi] = max;
				minimums[sdi] = min;
			}

			for (int sdi = 0; sdi < p_sdi->Length; sdi++)
			{
				for (int i = 0; i < saved_action_length; i++) {

					p_retval[i][sdi] = norm_min + (p_saved_action_definition[i][sdi] - minimums[sdi]) * ((norm_max - norm_min) / (maximums[sdi] - minimums[sdi]));
				}

			}

			return p_retval;
		}


		static array<array<float>^> ^ Symbolize(array<array<float> ^> ^p_saved_action_definition, array<int> ^p_sdi, unsigned int order)
		{
			Collections::Generic::List<array<float>^ > ^p_symbolic_generic_list = gcnew Collections::Generic::List<array<float>^ >();
			array<array<float>^> ^p_symbolic_list = gcnew array<array<float>^>(Math::Ceiling(p_saved_action_definition->Length / order));

			p_saved_action_definition = SymbolicDiff::Normalize(p_saved_action_definition, p_sdi, -1, 1);
			float conv_constant = (3 / (float)(1 - (-1)));

			unsigned int saved_action_length = p_saved_action_definition->Length;
			int current_symbol_angle;

			for (int i = 0; i < p_symbolic_list->Length; i++) {
				p_symbolic_list[i] = gcnew array<float>(p_sdi->Length);
			}

			
			unsigned int symbolic_index = 0;
				

			for (int i = 0; (i + order) < saved_action_length; i += order) {

				for (int sdi = 0; sdi < p_sdi->Length; sdi++)
				{

					current_symbol_angle = Math::Round((p_saved_action_definition[i + order - 1][sdi] - p_saved_action_definition[i][sdi])  * conv_constant);
					p_symbolic_list[symbolic_index][sdi] = current_symbol_angle;
					symbolic_index++;

				}


			}


			array<float> ^new_symbol = gcnew array<float>(p_sdi->Length);
			for (int sdi = 0; sdi < p_sdi->Length; sdi++)
			{
				new_symbol[sdi] = p_symbolic_list[0][sdi];
			}

			p_symbolic_generic_list->Add(new_symbol);


			for (int j = 1; j < p_symbolic_list->Length; j++) {

				
				
				int diff = p_symbolic_list[j][0] - p_symbolic_list[j - 1][0];

				if (0 == diff) {
					new_symbol = gcnew array<float>(p_sdi->Length);
					new_symbol[0] = p_symbolic_list[j][0];

					p_symbolic_generic_list->Add(new_symbol);
				}
				else {
					if (diff > 0) {
						for (int i = 0; i < diff; i++) {

							new_symbol = gcnew array<float>(p_sdi->Length);
							new_symbol[0] = p_symbolic_list[j - 1][0] + 1 + i;

							p_symbolic_generic_list->Add(new_symbol);
						}
					}
					else {
						for (int i = 0; i > diff; i--) {

							new_symbol = gcnew array<float>(p_sdi->Length);
							new_symbol[0] = p_symbolic_list[j - 1][0] - 1 + i;

							p_symbolic_generic_list->Add(new_symbol);
						}
					}
				}



			}

			return p_symbolic_generic_list->ToArray();;
		}




		virtual array<ViewSerialEvent> ^Apply(array<ViewSerialEvent> ^p_action)
		{
			return nullptr;
		}
		virtual array<double>^ Apply(array<array<float> ^> ^p_saved_action_definition, array<array<float> ^> ^p_last_actions, array<int> ^p_data_offsets) override
		{
			array<double> ^distances = gcnew array<double>(p_data_offsets->Length);
			array<double> ^ mse = gcnew array<double>(p_data_offsets->Length);

			unsigned int sensor_data_index_of_saved = 0;
			unsigned int sensor_data_index_of_last = p_data_offsets[0];
			unsigned int saved_action_length = p_saved_action_definition->Length;
			unsigned int last_action_length;


			p_last_actions = SymbolicDiff::Normalize(p_last_actions, p_data_offsets, -1, 1);

			array<array<float>^> ^symbolic_list = SymbolicDiff::Symbolize(p_last_actions, p_data_offsets, 4);

			last_action_length = symbolic_list->Length;



			double sum_of_squares = 0.0f;

			for (int sdi = 0; sdi < p_data_offsets->Length; sdi++) {

				sum_of_squares = 0;

				sensor_data_index_of_last = p_data_offsets[sdi];

				for (int i = 0; i < saved_action_length && i < last_action_length; i++) {


					sum_of_squares = sum_of_squares + Math::Pow((symbolic_list[last_action_length - i - 1][sensor_data_index_of_last] - p_saved_action_definition[saved_action_length - i - 1][sdi]), 2);
				}

				mse[sdi] = sum_of_squares;

				distances[sdi] = Math::Sqrt(mse[sdi]);
			}

			if (distances[0] < threshold) {
				FileSave = true;
			}


			if (FileSave) {

				String ^dir = gcnew String(Environment::CurrentDirectory + "\\SensorData\\Signals\\" + SaveDirIndex.ToString() + this->Name);
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
				for (int j = 0; j < saved_action_length; j++) {
					pTextWriter->Write(p_saved_action_definition[saved_action_length - j - 1][sensor_data_index_of_saved] + "\t");
				}
				pTextWriter->Flush();
				pTextWriter->Close();

				/****************************/
				path = gcnew String(dir + "\\symbolic_last_actions.txt");


				if (!IO::File::Exists(path)) {

					pTextWriter = gcnew System::IO::StreamWriter(path, true);
					pTextWriter->Flush();

				}
				for (int j = 0; j < symbolic_list->Length; j++) {
					pTextWriter->Write(symbolic_list[j][sensor_data_index_of_last] + "\t");
				}
				pTextWriter->Flush();
				pTextWriter->Close();

				/****************************/
				path = gcnew String(dir + "\\last_actions.txt");


				if (!IO::File::Exists(path)) {

					pTextWriter = gcnew System::IO::StreamWriter(path, true);
					pTextWriter->Flush();

				}
				for (int j = 0; j < p_last_actions->Length; j++) {
					pTextWriter->Write(p_last_actions[j][sensor_data_index_of_last] + "\t");
				}
				pTextWriter->Flush();
				pTextWriter->Close();


				/************DISTANCE****************/
				path = gcnew String(dir + "\\euclidian_distance.txt");


				if (!IO::File::Exists(path)) {

					pTextWriter = gcnew System::IO::StreamWriter(path, true);
					pTextWriter->Flush();

				}
				pTextWriter->Write(distances[0] + "\t");

				pTextWriter->Flush();
				pTextWriter->Close();

			}

			return distances;
		}
	};

}