#pragma once

#include "ISignalOp.h"
#include "DTW.h"
#include "DTWSax.h"
#include "Normalizer.h"
#include "SaxDiffDTW.h"
#include "SymbolicDiffDTW.h"

extern bool FileSave;
extern unsigned int SaveDirIndex;
extern double FileSaveThreshold;

using namespace System;

namespace SignalProcessing
{
	ref class SymbolicDiffVectoredDTW : public Operation {
	private:
		unsigned int Order;
		String ^name;
		double threshold;


	public:
		SymbolicDiffVectoredDTW() {

			Order = 4;
			this->Name = "SymbolicDiffVectoredDTW";


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
			return p_saved_action_definition;
		}

		static array<array<float> ^> ^ Normalize(array<array<float> ^> ^p_saved_action_definition, array<int> ^p_sdi, double norm_min, double norm_max)
		{
			return Normalizer::NormalizeBetween(p_saved_action_definition, p_sdi, norm_min, norm_max);
		}


		static array<array<float>^> ^ Symbolize(array<array<float> ^> ^p_saved_action_definition, array<int> ^p_sdi, unsigned int order)
		{
			Collections::Generic::List<array<float>^ > ^p_symbolic_generic_list = gcnew Collections::Generic::List<array<float>^ >();
			array<array<float>^> ^p_symbolic_list = gcnew array<array<float>^>(Math::Ceiling(p_saved_action_definition->Length / (order - 1)) + 1);

			p_saved_action_definition = SymbolicDiffVectoredDTW::Normalize(p_saved_action_definition, p_sdi, -1, 1);
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
			array<double> ^distances_sax = gcnew array<double>(p_data_offsets->Length);
			array<double> ^distances_sax_angle = gcnew array<double>(p_data_offsets->Length);
			array<double> ^ mse = gcnew array<double>(p_data_offsets->Length);

			unsigned int saved_sax_length;
			unsigned int last_action_sax_length; 

			unsigned int saved_sax_angle_length;
			unsigned int last_action_sax_angle_length;

			array<array<float>^> ^p_saved_sax = SaxDiffDTW::Normalize(p_saved_action_definition, p_data_offsets, -1, 1);
			p_saved_sax = SaxDiffDTW::Symbolize(p_saved_sax, p_data_offsets, 4);
			saved_sax_length = p_saved_sax->Length;

			array<array<float>^> ^p_last_actions_sax = SaxDiffDTW::Normalize(p_last_actions, p_data_offsets, -1, 1);
			p_last_actions_sax = SaxDiffDTW::Symbolize(p_last_actions_sax, p_data_offsets, 4);
			last_action_sax_length = p_last_actions_sax->Length;

			array<array<float>^> ^p_saved_sax_angle = SaxDiffDTW::Normalize(p_saved_action_definition, p_data_offsets, -1, 1);
			p_saved_sax_angle = SymbolicDiffDTW::Symbolize(p_saved_sax_angle, p_data_offsets, 4);
			saved_sax_angle_length = p_saved_sax_angle->Length;

			array<array<float>^> ^p_last_actions_sax_angle = SaxDiffDTW::Normalize(p_last_actions, p_data_offsets, -1, 1);
			p_last_actions_sax_angle = SymbolicDiffDTW::Symbolize(p_last_actions_sax_angle, p_data_offsets, 4);
			last_action_sax_angle_length = p_last_actions_sax_angle->Length;

			//distances_sax = DTW::Apply(p_saved_sax, p_last_actions_sax, p_data_offsets);
			distances_sax_angle = DTWSax::Apply(p_saved_sax_angle, p_last_actions_sax_angle, p_data_offsets, 1);


			for (int i = 0; i < p_data_offsets->Length; i++) {

				distances[i] = distances_sax[i] + distances_sax_angle[i];
			}

			if (distances[0] != distances[0]) {
				volatile int a = 1;
				a++;
			}
			if (distances[0] < FileSaveThreshold) {
					FileSave = true;
			}

			if (FileSave) {

				String ^dir = gcnew String(Environment::CurrentDirectory + "\\SensorData\\Signals\\" + SaveDirIndex.ToString() + this->Name);
				if (!IO::Directory::Exists(dir)) {

					IO::Directory::CreateDirectory(dir);
				}
				/****************************/

				String ^path = gcnew String(dir + "\\org_saved.txt");

				System::IO::TextWriter ^pTextWriter;
				if (!IO::File::Exists(path)) {

					pTextWriter = gcnew System::IO::StreamWriter(path, true);
					pTextWriter->Flush();

				}
				for (int j = 0; j < p_saved_action_definition->Length; j++) {
					pTextWriter->Write(p_saved_action_definition[j][0] + "\t");
				}
				pTextWriter->Flush();
				pTextWriter->Close();

				/****************************/
				path = gcnew String(dir + "\\sax_saved.txt");


				if (!IO::File::Exists(path)) {

					pTextWriter = gcnew System::IO::StreamWriter(path, true);
					pTextWriter->Flush();

				}
				for (int j = 0; j < p_saved_sax->Length; j++) {
					pTextWriter->Write(p_saved_sax[j][0] + "\t");
				}
				pTextWriter->Flush();
				pTextWriter->Close();

				/****************************/
				path = gcnew String(dir + "\\sax_angle_saved.txt");


				if (!IO::File::Exists(path)) {

					pTextWriter = gcnew System::IO::StreamWriter(path, true);
					pTextWriter->Flush();

				}
				for (int j = 0; j < p_saved_sax_angle->Length; j++) {
					pTextWriter->Write(p_saved_sax_angle[j][0] + "\t");
				}
				pTextWriter->Flush();
				pTextWriter->Close();

				/****************************/
				path = gcnew String(dir + "\\org_last.txt");


				if (!IO::File::Exists(path)) {

					pTextWriter = gcnew System::IO::StreamWriter(path, true);
					pTextWriter->Flush();

				}
				for (int j = 0; j < p_last_actions->Length; j++) {
					pTextWriter->Write(p_last_actions[j][0] + "\t");
				}
				pTextWriter->Flush();
				pTextWriter->Close();


				/****************************/
				path = gcnew String(dir + "\\sax_last.txt");


				if (!IO::File::Exists(path)) {

					pTextWriter = gcnew System::IO::StreamWriter(path, true);
					pTextWriter->Flush();

				}
				for (int j = 0; j < p_last_actions_sax->Length; j++) {
					pTextWriter->Write(p_last_actions_sax[j][0] + "\t");
				}
				pTextWriter->Flush();
				pTextWriter->Close();

				/****************************/
				path = gcnew String(dir + "\\sax_angle_last.txt");


				if (!IO::File::Exists(path)) {

					pTextWriter = gcnew System::IO::StreamWriter(path, true);
					pTextWriter->Flush();

				}
				for (int j = 0; j < p_last_actions_sax_angle->Length; j++) {
					pTextWriter->Write(p_last_actions_sax_angle[j][0] + "\t");
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