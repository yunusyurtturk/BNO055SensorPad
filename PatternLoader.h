#pragma once
#include "ActionBuffer.h"
#include "MainformOps.h"
#include "COpChain.h"
#include "AvgOp.h"
#include "DiffOp.h"
#include "DataComponent.h"
#include "ActionArrayBuffer.h"

#include "DTWDistance.h"
#include "DTWDistanceCustom.h"
#include "MSE3D.h"

extern bool FileSave;

namespace BNOControl {

	using namespace System;
	using namespace System::Windows::Forms;
	using namespace System::ComponentModel;
	using namespace System::Collections;
	using namespace System::Windows::Forms;
	using namespace System::Data;
	using namespace System::Drawing;
	using namespace MathNet::Numerics;
	
	/// <summary>
	/// Summary for PatternLoader
	/// </summary>
	public ref class PatternLoader : public System::Windows::Forms::Form
	{
	private:
		
		IMainFormOps ^pMainFormOps;
	private: System::Windows::Forms::ComboBox^  cbAction;
	private: System::Windows::Forms::CheckBox^  cbDiff;

	private: System::Windows::Forms::GroupBox^  groupBox1;
	private: System::Windows::Forms::CheckBox^  cbAvg;
			 ActionArrayBuffer ^pActionBuffer;


			
	private: System::Windows::Forms::TextBox^  tbMeaE;
	private: System::Windows::Forms::TextBox^  tbEstE;
	private: System::Windows::Forms::TextBox^  tbQ;

	float _err_measure;
	float _err_estimate;
	float _q;
	float _current_estimate;
	float _last_estimate;
	private: System::Windows::Forms::CheckedListBox^  clbSensorTypes;
	private: System::Windows::Forms::TextBox^  tbThresholdDistance;

	private: System::Windows::Forms::Label^  label2;
	private: System::Windows::Forms::ComboBox^  cbDistanceAlgorithm;


			 float _kalman_gain;
	
	public:
		PatternLoader(void)
		{
			InitializeComponent();

			cbDistanceAlgorithm->Items->Add(gcnew SignalProcessing::DTWDistance(5));
			cbDistanceAlgorithm->Items->Add(gcnew SignalProcessing::DTWDistanceCustom(5));
			cbDistanceAlgorithm->Items->Add(gcnew SignalProcessing::MSE3D());
			cbDistanceAlgorithm->SelectedIndex = 0;
		
		}

		IMainFormOps ^GetMainFormOps()
		{
			return this->pMainFormOps;
		}

		void SetMainFormOps(IMainFormOps ^p_val)
		{
			this->pMainFormOps = p_val;
		}

	protected:
		/// <summary>
		/// Clean up any resources being used.
		/// </summary>
		~PatternLoader()
		{
			if (components)
			{
				delete components;
			}
		}
	private: System::Windows::Forms::Button^  button1;
	private: System::Windows::Forms::DataVisualization::Charting::Chart^  cPattern;
	protected:

	private: System::Windows::Forms::TextBox^  tbFirstSampleNo;
	private: System::Windows::Forms::TextBox^  tbLastSampleNo;

	private: System::Windows::Forms::Label^  label1;
	private: System::Windows::Forms::Label^  lLastSampleNo;
	private: System::Windows::Forms::OpenFileDialog^  ofdSelectPatternFile;

	private: System::Windows::Forms::Button^  bUseSelectedPattern;

	private:
		/// <summary>
		/// Required designer variable.
		/// </summary>
		System::ComponentModel::Container ^components;

#pragma region Windows Form Designer generated code
		/// <summary>
		/// Required method for Designer support - do not modify
		/// the contents of this method with the code editor.
		/// </summary>
		void InitializeComponent(void)
		{
			System::Windows::Forms::DataVisualization::Charting::ChartArea^  chartArea1 = (gcnew System::Windows::Forms::DataVisualization::Charting::ChartArea());
			System::Windows::Forms::DataVisualization::Charting::Legend^  legend1 = (gcnew System::Windows::Forms::DataVisualization::Charting::Legend());
			System::Windows::Forms::DataVisualization::Charting::Series^  series1 = (gcnew System::Windows::Forms::DataVisualization::Charting::Series());
			System::Windows::Forms::DataVisualization::Charting::Series^  series2 = (gcnew System::Windows::Forms::DataVisualization::Charting::Series());
			this->button1 = (gcnew System::Windows::Forms::Button());
			this->cPattern = (gcnew System::Windows::Forms::DataVisualization::Charting::Chart());
			this->tbFirstSampleNo = (gcnew System::Windows::Forms::TextBox());
			this->tbLastSampleNo = (gcnew System::Windows::Forms::TextBox());
			this->label1 = (gcnew System::Windows::Forms::Label());
			this->lLastSampleNo = (gcnew System::Windows::Forms::Label());
			this->ofdSelectPatternFile = (gcnew System::Windows::Forms::OpenFileDialog());
			this->bUseSelectedPattern = (gcnew System::Windows::Forms::Button());
			this->cbAction = (gcnew System::Windows::Forms::ComboBox());
			this->cbDiff = (gcnew System::Windows::Forms::CheckBox());
			this->groupBox1 = (gcnew System::Windows::Forms::GroupBox());
			this->cbAvg = (gcnew System::Windows::Forms::CheckBox());
			this->tbMeaE = (gcnew System::Windows::Forms::TextBox());
			this->tbEstE = (gcnew System::Windows::Forms::TextBox());
			this->tbQ = (gcnew System::Windows::Forms::TextBox());
			this->clbSensorTypes = (gcnew System::Windows::Forms::CheckedListBox());
			this->tbThresholdDistance = (gcnew System::Windows::Forms::TextBox());
			this->label2 = (gcnew System::Windows::Forms::Label());
			this->cbDistanceAlgorithm = (gcnew System::Windows::Forms::ComboBox());
			(cli::safe_cast<System::ComponentModel::ISupportInitialize^>(this->cPattern))->BeginInit();
			this->groupBox1->SuspendLayout();
			this->SuspendLayout();
			// 
			// button1
			// 
			this->button1->Anchor = static_cast<System::Windows::Forms::AnchorStyles>((System::Windows::Forms::AnchorStyles::Bottom | System::Windows::Forms::AnchorStyles::Left));
			this->button1->Location = System::Drawing::Point(112, 426);
			this->button1->Name = L"button1";
			this->button1->Size = System::Drawing::Size(139, 32);
			this->button1->TabIndex = 0;
			this->button1->Text = L"Select Pattern File";
			this->button1->UseVisualStyleBackColor = true;
			this->button1->Click += gcnew System::EventHandler(this, &PatternLoader::button1_Click);
			// 
			// cPattern
			// 
			this->cPattern->Anchor = static_cast<System::Windows::Forms::AnchorStyles>((((System::Windows::Forms::AnchorStyles::Top | System::Windows::Forms::AnchorStyles::Bottom)
				| System::Windows::Forms::AnchorStyles::Left)
				| System::Windows::Forms::AnchorStyles::Right));
			chartArea1->Name = L"ChartArea1";
			this->cPattern->ChartAreas->Add(chartArea1);
			legend1->Name = L"Legend1";
			this->cPattern->Legends->Add(legend1);
			this->cPattern->Location = System::Drawing::Point(186, 13);
			this->cPattern->Name = L"cPattern";
			series1->ChartArea = L"ChartArea1";
			series1->ChartType = System::Windows::Forms::DataVisualization::Charting::SeriesChartType::Line;
			series1->Legend = L"Legend1";
			series1->Name = L"Series1";
			series2->ChartArea = L"ChartArea1";
			series2->ChartType = System::Windows::Forms::DataVisualization::Charting::SeriesChartType::Line;
			series2->Legend = L"Legend1";
			series2->Name = L"Series2";
			this->cPattern->Series->Add(series1);
			this->cPattern->Series->Add(series2);
			this->cPattern->Size = System::Drawing::Size(769, 407);
			this->cPattern->TabIndex = 1;
			this->cPattern->Text = L"chPattern";
			// 
			// tbFirstSampleNo
			// 
			this->tbFirstSampleNo->Anchor = static_cast<System::Windows::Forms::AnchorStyles>((System::Windows::Forms::AnchorStyles::Bottom | System::Windows::Forms::AnchorStyles::Left));
			this->tbFirstSampleNo->Location = System::Drawing::Point(112, 469);
			this->tbFirstSampleNo->Name = L"tbFirstSampleNo";
			this->tbFirstSampleNo->Size = System::Drawing::Size(100, 20);
			this->tbFirstSampleNo->TabIndex = 2;
			this->tbFirstSampleNo->Text = L"22";
			// 
			// tbLastSampleNo
			// 
			this->tbLastSampleNo->Anchor = static_cast<System::Windows::Forms::AnchorStyles>((System::Windows::Forms::AnchorStyles::Bottom | System::Windows::Forms::AnchorStyles::Left));
			this->tbLastSampleNo->Location = System::Drawing::Point(112, 495);
			this->tbLastSampleNo->Name = L"tbLastSampleNo";
			this->tbLastSampleNo->Size = System::Drawing::Size(100, 20);
			this->tbLastSampleNo->TabIndex = 3;
			this->tbLastSampleNo->Text = L"110";
			// 
			// label1
			// 
			this->label1->Anchor = static_cast<System::Windows::Forms::AnchorStyles>((System::Windows::Forms::AnchorStyles::Bottom | System::Windows::Forms::AnchorStyles::Left));
			this->label1->AutoSize = true;
			this->label1->Location = System::Drawing::Point(32, 475);
			this->label1->Name = L"label1";
			this->label1->Size = System::Drawing::Size(74, 13);
			this->label1->TabIndex = 4;
			this->label1->Text = L"First Sample #";
			// 
			// lLastSampleNo
			// 
			this->lLastSampleNo->Anchor = static_cast<System::Windows::Forms::AnchorStyles>((System::Windows::Forms::AnchorStyles::Bottom | System::Windows::Forms::AnchorStyles::Left));
			this->lLastSampleNo->AutoSize = true;
			this->lLastSampleNo->Location = System::Drawing::Point(32, 498);
			this->lLastSampleNo->Name = L"lLastSampleNo";
			this->lLastSampleNo->Size = System::Drawing::Size(75, 13);
			this->lLastSampleNo->TabIndex = 5;
			this->lLastSampleNo->Text = L"Last Sample #";
			// 
			// ofdSelectPatternFile
			// 
			this->ofdSelectPatternFile->FileName = L"openFileDialog1";
			this->ofdSelectPatternFile->RestoreDirectory = true;
			// 
			// bUseSelectedPattern
			// 
			this->bUseSelectedPattern->Anchor = static_cast<System::Windows::Forms::AnchorStyles>((System::Windows::Forms::AnchorStyles::Bottom | System::Windows::Forms::AnchorStyles::Right));
			this->bUseSelectedPattern->Location = System::Drawing::Point(859, 433);
			this->bUseSelectedPattern->Name = L"bUseSelectedPattern";
			this->bUseSelectedPattern->Size = System::Drawing::Size(96, 77);
			this->bUseSelectedPattern->TabIndex = 7;
			this->bUseSelectedPattern->Text = L"Use Selected Pattern";
			this->bUseSelectedPattern->UseVisualStyleBackColor = true;
			this->bUseSelectedPattern->Click += gcnew System::EventHandler(this, &PatternLoader::bUseSelectedPattern_Click);
			// 
			// cbAction
			// 
			this->cbAction->Anchor = static_cast<System::Windows::Forms::AnchorStyles>((System::Windows::Forms::AnchorStyles::Bottom | System::Windows::Forms::AnchorStyles::Right));
			this->cbAction->FormattingEnabled = true;
			this->cbAction->Items->AddRange(gcnew cli::array< System::Object^  >(8) { L"0", L"1", L"2", L"3", L"4", L"5", L"6", L"7" });
			this->cbAction->Location = System::Drawing::Point(218, 467);
			this->cbAction->Name = L"cbAction";
			this->cbAction->Size = System::Drawing::Size(137, 21);
			this->cbAction->TabIndex = 8;
			this->cbAction->Text = L"0";
			// 
			// cbDiff
			// 
			this->cbDiff->AutoSize = true;
			this->cbDiff->Location = System::Drawing::Point(6, 19);
			this->cbDiff->Name = L"cbDiff";
			this->cbDiff->Size = System::Drawing::Size(42, 17);
			this->cbDiff->TabIndex = 9;
			this->cbDiff->Text = L"Diff";
			this->cbDiff->UseVisualStyleBackColor = true;
			// 
			// groupBox1
			// 
			this->groupBox1->Anchor = static_cast<System::Windows::Forms::AnchorStyles>((System::Windows::Forms::AnchorStyles::Bottom | System::Windows::Forms::AnchorStyles::Right));
			this->groupBox1->Controls->Add(this->cbAvg);
			this->groupBox1->Controls->Add(this->cbDiff);
			this->groupBox1->Location = System::Drawing::Point(577, 433);
			this->groupBox1->Name = L"groupBox1";
			this->groupBox1->Size = System::Drawing::Size(140, 82);
			this->groupBox1->TabIndex = 10;
			this->groupBox1->TabStop = false;
			this->groupBox1->Text = L"groupBox1";
			// 
			// cbAvg
			// 
			this->cbAvg->AutoSize = true;
			this->cbAvg->Location = System::Drawing::Point(6, 36);
			this->cbAvg->Name = L"cbAvg";
			this->cbAvg->Size = System::Drawing::Size(45, 17);
			this->cbAvg->TabIndex = 10;
			this->cbAvg->Text = L"Avg";
			this->cbAvg->UseVisualStyleBackColor = true;
			// 
			// tbMeaE
			// 
			this->tbMeaE->Anchor = static_cast<System::Windows::Forms::AnchorStyles>((System::Windows::Forms::AnchorStyles::Bottom | System::Windows::Forms::AnchorStyles::Right));
			this->tbMeaE->Location = System::Drawing::Point(724, 465);
			this->tbMeaE->Name = L"tbMeaE";
			this->tbMeaE->Size = System::Drawing::Size(32, 20);
			this->tbMeaE->TabIndex = 11;
			this->tbMeaE->Text = L"0.5";
			// 
			// tbEstE
			// 
			this->tbEstE->Anchor = static_cast<System::Windows::Forms::AnchorStyles>((System::Windows::Forms::AnchorStyles::Bottom | System::Windows::Forms::AnchorStyles::Right));
			this->tbEstE->Location = System::Drawing::Point(769, 464);
			this->tbEstE->Name = L"tbEstE";
			this->tbEstE->Size = System::Drawing::Size(32, 20);
			this->tbEstE->TabIndex = 12;
			this->tbEstE->Text = L"0.5";
			// 
			// tbQ
			// 
			this->tbQ->Anchor = static_cast<System::Windows::Forms::AnchorStyles>((System::Windows::Forms::AnchorStyles::Bottom | System::Windows::Forms::AnchorStyles::Right));
			this->tbQ->Location = System::Drawing::Point(812, 464);
			this->tbQ->Name = L"tbQ";
			this->tbQ->Size = System::Drawing::Size(32, 20);
			this->tbQ->TabIndex = 13;
			this->tbQ->Text = L"0.1";
			// 
			// clbSensorTypes
			// 
			this->clbSensorTypes->CheckOnClick = true;
			this->clbSensorTypes->FormattingEnabled = true;
			this->clbSensorTypes->Location = System::Drawing::Point(13, 13);
			this->clbSensorTypes->Name = L"clbSensorTypes";
			this->clbSensorTypes->Size = System::Drawing::Size(167, 409);
			this->clbSensorTypes->TabIndex = 14;
			this->clbSensorTypes->ItemCheck += gcnew System::Windows::Forms::ItemCheckEventHandler(this, &PatternLoader::clbSensorTypes_ItemCheck);
			// 
			// tbThresholdDistance
			// 
			this->tbThresholdDistance->Anchor = static_cast<System::Windows::Forms::AnchorStyles>((System::Windows::Forms::AnchorStyles::Bottom | System::Windows::Forms::AnchorStyles::Right));
			this->tbThresholdDistance->Location = System::Drawing::Point(323, 494);
			this->tbThresholdDistance->Name = L"tbThresholdDistance";
			this->tbThresholdDistance->Size = System::Drawing::Size(32, 20);
			this->tbThresholdDistance->TabIndex = 15;
			this->tbThresholdDistance->Text = L"1";
			// 
			// label2
			// 
			this->label2->Anchor = static_cast<System::Windows::Forms::AnchorStyles>((System::Windows::Forms::AnchorStyles::Bottom | System::Windows::Forms::AnchorStyles::Left));
			this->label2->AutoSize = true;
			this->label2->Location = System::Drawing::Point(218, 498);
			this->label2->Name = L"label2";
			this->label2->Size = System::Drawing::Size(99, 13);
			this->label2->TabIndex = 16;
			this->label2->Text = L"Threshold Distance";
			// 
			// cbDistanceAlgorithm
			// 
			this->cbDistanceAlgorithm->FormattingEnabled = true;
			this->cbDistanceAlgorithm->Location = System::Drawing::Point(435, 437);
			this->cbDistanceAlgorithm->Name = L"cbDistanceAlgorithm";
			this->cbDistanceAlgorithm->Size = System::Drawing::Size(121, 21);
			this->cbDistanceAlgorithm->TabIndex = 17;
			// 
			// PatternLoader
			// 
			this->AutoScaleDimensions = System::Drawing::SizeF(6, 13);
			this->AutoScaleMode = System::Windows::Forms::AutoScaleMode::Font;
			this->ClientSize = System::Drawing::Size(981, 522);
			this->Controls->Add(this->cbDistanceAlgorithm);
			this->Controls->Add(this->label2);
			this->Controls->Add(this->tbThresholdDistance);
			this->Controls->Add(this->clbSensorTypes);
			this->Controls->Add(this->tbQ);
			this->Controls->Add(this->tbEstE);
			this->Controls->Add(this->tbMeaE);
			this->Controls->Add(this->groupBox1);
			this->Controls->Add(this->cbAction);
			this->Controls->Add(this->bUseSelectedPattern);
			this->Controls->Add(this->lLastSampleNo);
			this->Controls->Add(this->label1);
			this->Controls->Add(this->tbLastSampleNo);
			this->Controls->Add(this->tbFirstSampleNo);
			this->Controls->Add(this->cPattern);
			this->Controls->Add(this->button1);
			this->Name = L"PatternLoader";
			this->Text = L"PatternLoader";
			(cli::safe_cast<System::ComponentModel::ISupportInitialize^>(this->cPattern))->EndInit();
			this->groupBox1->ResumeLayout(false);
			this->groupBox1->PerformLayout();
			this->ResumeLayout(false);
			this->PerformLayout();

		}
#pragma endregion
	private: System::Void button1_Click(System::Object^  sender, System::EventArgs^  e) {
		System::Windows::Forms::DialogResult result =  ofdSelectPatternFile->ShowDialog();

		if (result == System::Windows::Forms::DialogResult::OK) {

			String ^file_path = ofdSelectPatternFile->FileName;
			IO::Stream ^p_file_stream = ofdSelectPatternFile->OpenFile();

			IO::StreamReader ^reader = gcnew IO::StreamReader(p_file_stream);


			pActionBuffer = gcnew ActionArrayBuffer(150);
			String ^line;
			
			unsigned int sensor_data_type_length = clbSensorTypes->Items->Count;

			while ((line =reader->ReadLine()) != nullptr) {
				array<float> ^ sensor_data = gcnew array<float>(sensor_data_type_length + 1);
				array<String ^> ^sensor_str_data = line->Split();

				

				for (int i = 0; i < sensor_str_data->Length && i < sensor_data_type_length + 1; i++) {

					if (sensor_str_data[i] != "") {
						sensor_data[i] = Convert::ToSingle(sensor_str_data[i]);
					}
					
				}
				

				pActionBuffer->Push(sensor_data);
			}

		}

		array<array<float> ^> ^p_events = pActionBuffer->GetQueue()->ToArray();

		for (int i = 0; i < p_events->Length; i++) {

			for (int j = 1; j < p_events[i]->Length; j++) {
				//cPattern->Series[j - 1]->Points->AddXY(p_events[i][0], p_events[i][j]); // Discard timestamp
				cPattern->Series[j - 1]->Points->AddXY(i, p_events[i][j]); // Discard timestamp
			}
			
		}
		cPattern->ChartAreas[0]->AxisY->MaximumAutoSize = true;
		cPattern->Update();

	}
public:
	void SetSensorTypes(System::Windows::Forms::CheckedListBox::ObjectCollection ^object_collection)
	{
		cPattern->Series->Clear();
		for (int i = 0; i < object_collection->Count; i++) {

			Object ^obj = (Object ^)object_collection[i];
			DataComponent ^p_component = (DataComponent ^)obj;
			clbSensorTypes->Items->Add(p_component);

			DataVisualization::Charting::Series ^series = cPattern->Series->Add(p_component->Name);
			series->ChartType = DataVisualization::Charting::SeriesChartType::FastLine;
			series->Enabled = false;
			series->YValuesPerPoint = SENSOR_DATA_TYPE_COUNT;
			series->ToolTip = "#VALX{0.0} #VAL{0.0}";
			
		}



	


	}
private:
	float updateEstimate(float mea)
	{
		_kalman_gain = _err_estimate / (_err_estimate + _err_measure);
		_current_estimate = _last_estimate + _kalman_gain * (mea - _last_estimate);
		_err_estimate = (1.0 - _kalman_gain)*_err_estimate + Math::Abs(_last_estimate - _current_estimate)*_q;
		_last_estimate = _current_estimate;

		return _current_estimate;
	}
private: System::Void cbSensorDataType_SelectedIndexChanged(System::Object^  sender, System::EventArgs^  e) {


}
private: System::Void bUseSelectedPattern_Click(System::Object^  sender, System::EventArgs^  e) {

	unsigned int start	= Convert::ToUInt32(tbFirstSampleNo->Text);
	unsigned int end	= Convert::ToUInt32(tbLastSampleNo->Text);

	String ^selected_action_str = cbAction->SelectedItem->ToString();
	int selected_action = Convert::ToInt32(selected_action_str);

	int extracted_pattern_length = (end - start + 1);

	double threshold = Convert::ToDouble(tbThresholdDistance->Text);

	array<int> ^p_selected_sensor_datas = gcnew array<int>(clbSensorTypes->CheckedIndices->Count);

	array<array<float> ^> ^extracted_pattern = gcnew array<array<float> ^>(extracted_pattern_length);

	//System::Array::Copy(pActionBuffer->GetQueue()->ToArray(), start, extracted_pattern, 0, extracted_pattern_length);

	array<array<float> ^> ^ p_action_buffer = pActionBuffer->GetQueue()->ToArray();
	int checked_counter = 0;

	System::Windows::Forms::CheckedListBox::CheckedItemCollection ^p_checked_items = clbSensorTypes->CheckedItems;
	for each(Object ^obj in p_checked_items) {
		DataComponent ^p_component = (DataComponent ^)obj;
		p_selected_sensor_datas[checked_counter] = p_component->Offset;
		checked_counter++;

	}

	for (int i = 0; i < extracted_pattern_length; i++) {

		extracted_pattern[i] = gcnew array<float>(clbSensorTypes->CheckedIndices->Count);

		

		checked_counter = 0;
		for each(Object ^obj in p_checked_items) {
			DataComponent ^p_component = (DataComponent ^)obj;

			extracted_pattern[i][checked_counter] = p_action_buffer[start + i][p_component->Offset + 1]; // +1 , ilk elemanin timestamp olmasindan dolayi ekleniyor
			checked_counter++;

		}




	}

	SignalProcessing::Operation ^p_distance_operator = ((SignalProcessing::Operation ^)cbDistanceAlgorithm->Items[cbDistanceAlgorithm->SelectedIndex]);


	GetMainFormOps()->AddPattern(selected_action, extracted_pattern, p_selected_sensor_datas, threshold, p_distance_operator);
	
	checked_counter = 0;
	

}
private: System::Void clbSensorTypes_ItemCheck(System::Object^  sender, System::Windows::Forms::ItemCheckEventArgs^  e) {
	DataComponent ^p_component = (DataComponent ^)clbSensorTypes->Items[e->Index];

	if (nullptr != cPattern->Series && cPattern->Series->Count > 0) {
		cPattern->Series[e->Index]->Enabled = (System::Windows::Forms::CheckState::Checked == e->NewValue);
		cPattern->ChartAreas[0]->RecalculateAxesScale();
	}

	
}
};
}
