#pragma once

#include "ISignalOp.h"

extern bool FileSave;
extern unsigned int SaveDirIndex;


using namespace System;

namespace SignalProcessing
{
	ref class MSE3D : public Operation {
	private:
		unsigned int Order;
		String ^name;
		double threshold;


	public:
		MSE3D() {

			this->Name = "MSE3D";


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

		virtual array<array<float>^> ^ PreprocessActionTemplate(array<array<float> ^> ^p_saved_action_definition, array<int> ^p_sdi)
		{
			return p_saved_action_definition;
		}

		virtual virtual void SetFileSaveEnabled(boolean val, double threshold)
		{
			Threshold = threshold;
			FileSave = true;
		}

		virtual array<ViewSerialEvent> ^Apply(array<ViewSerialEvent> ^p_action)
		{
			return nullptr;
		}
		virtual array<double>^ Apply(array<array<float> ^> ^p_saved_action_definition, array<array<float> ^> ^p_last_actions, array<int> ^p_data_offsets) override
		{
			array<double> ^distances = gcnew array<double>(p_data_offsets->Length);
			array<double> ^ mse = gcnew array<double>(p_data_offsets->Length);
			double action_original_length = 0;
			unsigned int sensor_data_index_of_saved = 0;
			unsigned int sensor_data_index_of_last = p_data_offsets[0];
			const unsigned int const saved_action_length	= p_saved_action_definition->Length;
			const unsigned int const last_action_length		= p_last_actions->Length;

	
			double sum_of_squares = 0.0f;

			for (int sdi = 0; sdi < p_data_offsets->Length; sdi++) {

				sum_of_squares = 0;

				action_original_length = 0;

				sensor_data_index_of_last = p_data_offsets[sdi];

				for (int i = 0; i < saved_action_length; i++) {

					
					sum_of_squares = sum_of_squares + Math::Pow((p_last_actions[last_action_length - i - 1][sensor_data_index_of_last] - p_saved_action_definition[saved_action_length - i - 1][sdi]), 2);
					
					action_original_length = action_original_length + Math::Pow(p_saved_action_definition[saved_action_length - i - 1][sensor_data_index_of_saved], 2);

				}

				mse[sdi] = sum_of_squares;
				
			//	distances[sdi] = Math::Sqrt(mse[sdi] / action_original_length);

				distances[sdi] = Math::Sqrt(mse[sdi]);

			
			}

			if (distances[0] < threshold) {
				FileSave = true;
			}


			if (FileSave) {

				String ^dir = gcnew String(Environment::CurrentDirectory + "\\SensorData\\Signals\\"+ SaveDirIndex.ToString() + this->Name);
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
				path = gcnew String(dir + "\\last_action.txt");


				if (!IO::File::Exists(path)) {

					pTextWriter = gcnew System::IO::StreamWriter(path, true);
					pTextWriter->Flush();

				}
				for (int j = 0; j < saved_action_length; j++) {
					pTextWriter->Write(p_last_actions[last_action_length - j - 1][sensor_data_index_of_last] + "\t");
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