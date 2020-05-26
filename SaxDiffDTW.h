#pragma once

#include "ISignalOp.h"
#include "DTW.h"

extern bool FileSave;
extern unsigned int SaveDirIndex;


using namespace System;

namespace SignalProcessing
{
	ref class SaxDiffDTW : public Operation {
	private:
		unsigned int Order;
		String ^name;
		double threshold;


	public:
		SaxDiffDTW() {

			Order = 4;
			this->Name = "SaxDiffDTW";


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
			p_saved_action_definition = SaxDiffDTW::Normalize(p_saved_action_definition, p_sdi, -1, 1);

			return SaxDiffDTW::Symbolize(p_saved_action_definition, p_sdi, 4);
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
			array<array<float>^> ^p_symbolic_list = gcnew array<array<float>^>(Math::Ceiling(p_saved_action_definition->Length / (order - 1)) + 1);

			p_saved_action_definition = SaxDiffDTW::Normalize(p_saved_action_definition, p_sdi, -1, 1);
			float conv_constant = (3 / (float)(1 - (-1)));

			unsigned int saved_action_length = p_saved_action_definition->Length;
			int current_symbol_angle;

			for (int i = 0; i < p_symbolic_list->Length; i++) {
				p_symbolic_list[i] = gcnew array<float>(p_sdi->Length);
			}


			for (int sdi = 0; sdi < p_sdi->Length; sdi++)
			{
				current_symbol_angle = Math::Round((p_saved_action_definition[0][sdi] * conv_constant));
				p_symbolic_list[0][sdi] = current_symbol_angle;
			}

			unsigned int symbolic_index = 1;

			double current_mean;
			for (int i = 0; (i + order - 1) <= saved_action_length; i += (order - 1)) {

				current_mean = 0;
				for (int sdi = 0; sdi < p_sdi->Length; sdi++)
				{
					current_mean = 0;

					for (int index = i; index < (i + order); index++) {

						current_mean += p_saved_action_definition[index][sdi];
					}

					current_mean = current_mean / order;

					current_symbol_angle = Math::Round((current_mean  * conv_constant));
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


			p_last_actions = SaxDiffDTW::Normalize(p_last_actions, p_data_offsets, -1, 1);

			array<array<float>^> ^symbolic_list = SaxDiffDTW::Symbolize(p_last_actions, p_data_offsets, 4);

			last_action_length = symbolic_list->Length;

			distances = DTW::Apply(p_saved_action_definition, symbolic_list, p_data_offsets);

			if (distances[0] > 1 && distances[0] < 2) {
			//	FileSave = true;
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
					pTextWriter->Write(p_saved_action_definition[j][sensor_data_index_of_saved] + "\t");
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