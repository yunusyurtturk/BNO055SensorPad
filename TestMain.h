#pragma once
#include "DataComponent.h"
#include "ViewTypes.h"
#include "DTWDistance.h"
#include "MSE3D.h"


#define ALGORITHM_TEST_MAX_SAMPLE_COUNT 400
namespace BNOControl {

	using namespace System;
	using namespace System::ComponentModel;
	using namespace System::Collections;
	using namespace System::Windows::Forms;
	using namespace System::Data;
	using namespace System::Drawing;

	/// <summary>
	/// Summary for TestMain
	/// </summary>
	public ref class TestMain : public System::Windows::Forms::Form
	{
	private:
		array<array<float> ^>^ pActionArrayBuffer;
		unsigned int LoadedSampleLength;
	private: System::Windows::Forms::GroupBox^  groupBox3;
	private: System::Windows::Forms::Label^  lDistanceResult;
	private: System::Windows::Forms::Label^  label2;
	private: System::Windows::Forms::Button^  button3;
			 DataVisualization::Charting::Series ^pCrossComponentSeries;
	public:
		TestMain(void)
		{
			InitializeComponent();
			LoadedSampleLength = 0;
			pActionArrayBuffer = gcnew array<array<float>^>(ALGORITHM_TEST_MAX_SAMPLE_COUNT);

			cbDistanceAlgorithm->Items->Add(gcnew SignalProcessing::DTWDistance(5));
			cbDistanceAlgorithm->Items->Add(gcnew SignalProcessing::MSE3D());
			cbDistanceAlgorithm->SelectedIndex = 0;

		}

	protected:
		/// <summary>
		/// Clean up any resources being used.
		/// </summary>
		~TestMain()
		{
			if (components)
			{
				delete components;
			}
		}
	private: System::Windows::Forms::Button^  button1;
	protected:
	private: System::Windows::Forms::GroupBox^  groupBox1;


	private: System::Windows::Forms::TextBox^  tbLastSample;
	private: System::Windows::Forms::TextBox^  tbFirstSample;
	private: System::Windows::Forms::Label^  lLoadedSampleLength;

	private: System::Windows::Forms::Label^  label1;
	private: System::Windows::Forms::GroupBox^  groupBox2;
	private: System::Windows::Forms::Label^  label3;
	private: System::Windows::Forms::ComboBox^  cbDistanceAlgorithm;

	private: System::Windows::Forms::Label^  label5;
	private: System::Windows::Forms::Label^  label4;
	private: System::Windows::Forms::OpenFileDialog^  ofdSelectPatternFile;
	private: System::Windows::Forms::CheckedListBox^  clbSensorTypes;
	private: System::Windows::Forms::DataVisualization::Charting::Chart^  cPattern;
	private: System::Windows::Forms::Button^  button2;
	private: System::Windows::Forms::Label^  label9;
	private: System::Windows::Forms::Label^  label8;
	private: System::Windows::Forms::Label^  label6;
	private: System::Windows::Forms::Label^  label7;
	private: System::Windows::Forms::TextBox^  textBox1;
	private: System::Windows::Forms::TextBox^  textBox2;
	private: System::Windows::Forms::CheckBox^  cbCompareLastActions;



	private: System::Windows::Forms::TextBox^  tbLastCompareSample;

	private: System::Windows::Forms::TextBox^  tbFirstCompareSample;






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
			System::Windows::Forms::DataVisualization::Charting::ChartArea^  chartArea3 = (gcnew System::Windows::Forms::DataVisualization::Charting::ChartArea());
			System::Windows::Forms::DataVisualization::Charting::ChartArea^  chartArea4 = (gcnew System::Windows::Forms::DataVisualization::Charting::ChartArea());
			System::Windows::Forms::DataVisualization::Charting::Legend^  legend2 = (gcnew System::Windows::Forms::DataVisualization::Charting::Legend());
			System::Windows::Forms::DataVisualization::Charting::Series^  series3 = (gcnew System::Windows::Forms::DataVisualization::Charting::Series());
			System::Windows::Forms::DataVisualization::Charting::Series^  series4 = (gcnew System::Windows::Forms::DataVisualization::Charting::Series());
			this->button1 = (gcnew System::Windows::Forms::Button());
			this->groupBox1 = (gcnew System::Windows::Forms::GroupBox());
			this->label9 = (gcnew System::Windows::Forms::Label());
			this->label8 = (gcnew System::Windows::Forms::Label());
			this->label6 = (gcnew System::Windows::Forms::Label());
			this->label7 = (gcnew System::Windows::Forms::Label());
			this->tbLastCompareSample = (gcnew System::Windows::Forms::TextBox());
			this->textBox1 = (gcnew System::Windows::Forms::TextBox());
			this->tbFirstCompareSample = (gcnew System::Windows::Forms::TextBox());
			this->textBox2 = (gcnew System::Windows::Forms::TextBox());
			this->label5 = (gcnew System::Windows::Forms::Label());
			this->label4 = (gcnew System::Windows::Forms::Label());
			this->tbLastSample = (gcnew System::Windows::Forms::TextBox());
			this->tbFirstSample = (gcnew System::Windows::Forms::TextBox());
			this->lLoadedSampleLength = (gcnew System::Windows::Forms::Label());
			this->label1 = (gcnew System::Windows::Forms::Label());
			this->groupBox2 = (gcnew System::Windows::Forms::GroupBox());
			this->label3 = (gcnew System::Windows::Forms::Label());
			this->cbDistanceAlgorithm = (gcnew System::Windows::Forms::ComboBox());
			this->ofdSelectPatternFile = (gcnew System::Windows::Forms::OpenFileDialog());
			this->clbSensorTypes = (gcnew System::Windows::Forms::CheckedListBox());
			this->cPattern = (gcnew System::Windows::Forms::DataVisualization::Charting::Chart());
			this->button2 = (gcnew System::Windows::Forms::Button());
			this->cbCompareLastActions = (gcnew System::Windows::Forms::CheckBox());
			this->groupBox3 = (gcnew System::Windows::Forms::GroupBox());
			this->lDistanceResult = (gcnew System::Windows::Forms::Label());
			this->label2 = (gcnew System::Windows::Forms::Label());
			this->button3 = (gcnew System::Windows::Forms::Button());
			this->groupBox1->SuspendLayout();
			this->groupBox2->SuspendLayout();
			(cli::safe_cast<System::ComponentModel::ISupportInitialize^>(this->cPattern))->BeginInit();
			this->groupBox3->SuspendLayout();
			this->SuspendLayout();
			// 
			// button1
			// 
			this->button1->Location = System::Drawing::Point(6, 29);
			this->button1->Name = L"button1";
			this->button1->Size = System::Drawing::Size(97, 51);
			this->button1->TabIndex = 0;
			this->button1->Text = L"Veri Dosyasý Yükle";
			this->button1->UseVisualStyleBackColor = true;
			this->button1->Click += gcnew System::EventHandler(this, &TestMain::button1_Click);
			// 
			// groupBox1
			// 
			this->groupBox1->Controls->Add(this->button3);
			this->groupBox1->Controls->Add(this->label9);
			this->groupBox1->Controls->Add(this->label8);
			this->groupBox1->Controls->Add(this->label6);
			this->groupBox1->Controls->Add(this->label7);
			this->groupBox1->Controls->Add(this->tbLastCompareSample);
			this->groupBox1->Controls->Add(this->textBox1);
			this->groupBox1->Controls->Add(this->tbFirstCompareSample);
			this->groupBox1->Controls->Add(this->textBox2);
			this->groupBox1->Controls->Add(this->label5);
			this->groupBox1->Controls->Add(this->label4);
			this->groupBox1->Controls->Add(this->tbLastSample);
			this->groupBox1->Controls->Add(this->tbFirstSample);
			this->groupBox1->Controls->Add(this->lLoadedSampleLength);
			this->groupBox1->Controls->Add(this->label1);
			this->groupBox1->Controls->Add(this->button1);
			this->groupBox1->Location = System::Drawing::Point(195, 19);
			this->groupBox1->Name = L"groupBox1";
			this->groupBox1->Size = System::Drawing::Size(332, 120);
			this->groupBox1->TabIndex = 1;
			this->groupBox1->TabStop = false;
			this->groupBox1->Text = L"Örnek Ayarlarý";
			// 
			// label9
			// 
			this->label9->AutoSize = true;
			this->label9->Location = System::Drawing::Point(220, 29);
			this->label9->Name = L"label9";
			this->label9->Size = System::Drawing::Size(36, 13);
			this->label9->TabIndex = 13;
			this->label9->Text = L"Örnek";
			// 
			// label8
			// 
			this->label8->AutoSize = true;
			this->label8->Location = System::Drawing::Point(153, 43);
			this->label8->Name = L"label8";
			this->label8->Size = System::Drawing::Size(37, 13);
			this->label8->TabIndex = 12;
			this->label8->Text = L"Eksen";
			// 
			// label6
			// 
			this->label6->AutoSize = true;
			this->label6->Location = System::Drawing::Point(114, 97);
			this->label6->Name = L"label6";
			this->label6->Size = System::Drawing::Size(33, 13);
			this->label6->TabIndex = 11;
			this->label6->Text = L"Maks";
			// 
			// label7
			// 
			this->label7->AutoSize = true;
			this->label7->Location = System::Drawing::Point(124, 65);
			this->label7->Name = L"label7";
			this->label7->Size = System::Drawing::Size(24, 13);
			this->label7->TabIndex = 10;
			this->label7->Text = L"Min";
			// 
			// tbLastCompareSample
			// 
			this->tbLastCompareSample->Location = System::Drawing::Point(243, 94);
			this->tbLastCompareSample->Name = L"tbLastCompareSample";
			this->tbLastCompareSample->Size = System::Drawing::Size(39, 20);
			this->tbLastCompareSample->TabIndex = 15;
			this->tbLastCompareSample->TextChanged += gcnew System::EventHandler(this, &TestMain::tbLastCompareSample_TextChanged);
			// 
			// textBox1
			// 
			this->textBox1->Location = System::Drawing::Point(153, 94);
			this->textBox1->Name = L"textBox1";
			this->textBox1->Size = System::Drawing::Size(39, 20);
			this->textBox1->TabIndex = 9;
			this->textBox1->TextChanged += gcnew System::EventHandler(this, &TestMain::textBox1_TextChanged);
			// 
			// tbFirstCompareSample
			// 
			this->tbFirstCompareSample->Location = System::Drawing::Point(198, 94);
			this->tbFirstCompareSample->Name = L"tbFirstCompareSample";
			this->tbFirstCompareSample->Size = System::Drawing::Size(39, 20);
			this->tbFirstCompareSample->TabIndex = 14;
			this->tbFirstCompareSample->TextChanged += gcnew System::EventHandler(this, &TestMain::tbFirstCompareSample_TextChanged);
			// 
			// textBox2
			// 
			this->textBox2->Location = System::Drawing::Point(153, 59);
			this->textBox2->Name = L"textBox2";
			this->textBox2->Size = System::Drawing::Size(39, 20);
			this->textBox2->TabIndex = 8;
			this->textBox2->TextChanged += gcnew System::EventHandler(this, &TestMain::textBox2_TextChanged);
			// 
			// label5
			// 
			this->label5->AutoSize = true;
			this->label5->Location = System::Drawing::Point(251, 41);
			this->label5->Name = L"label5";
			this->label5->Size = System::Drawing::Size(26, 13);
			this->label5->TabIndex = 7;
			this->label5->Text = L"Son";
			// 
			// label4
			// 
			this->label4->AutoSize = true;
			this->label4->Location = System::Drawing::Point(209, 41);
			this->label4->Name = L"label4";
			this->label4->Size = System::Drawing::Size(18, 13);
			this->label4->TabIndex = 6;
			this->label4->Text = L"Ýlk";
			// 
			// tbLastSample
			// 
			this->tbLastSample->Location = System::Drawing::Point(243, 58);
			this->tbLastSample->Name = L"tbLastSample";
			this->tbLastSample->Size = System::Drawing::Size(39, 20);
			this->tbLastSample->TabIndex = 4;
			this->tbLastSample->TextChanged += gcnew System::EventHandler(this, &TestMain::tbLastSample_TextChanged);
			// 
			// tbFirstSample
			// 
			this->tbFirstSample->Location = System::Drawing::Point(198, 59);
			this->tbFirstSample->Name = L"tbFirstSample";
			this->tbFirstSample->Size = System::Drawing::Size(39, 20);
			this->tbFirstSample->TabIndex = 3;
			this->tbFirstSample->TextChanged += gcnew System::EventHandler(this, &TestMain::tbFirstSample_TextChanged);
			// 
			// lLoadedSampleLength
			// 
			this->lLoadedSampleLength->AutoSize = true;
			this->lLoadedSampleLength->Location = System::Drawing::Point(182, 29);
			this->lLoadedSampleLength->Name = L"lLoadedSampleLength";
			this->lLoadedSampleLength->Size = System::Drawing::Size(10, 13);
			this->lLoadedSampleLength->TabIndex = 2;
			this->lLoadedSampleLength->Text = L"-";
			// 
			// label1
			// 
			this->label1->AutoSize = true;
			this->label1->Location = System::Drawing::Point(110, 29);
			this->label1->Name = L"label1";
			this->label1->Size = System::Drawing::Size(66, 13);
			this->label1->TabIndex = 1;
			this->label1->Text = L"Örnek Sayýsý";
			// 
			// groupBox2
			// 
			this->groupBox2->Controls->Add(this->label3);
			this->groupBox2->Controls->Add(this->cbDistanceAlgorithm);
			this->groupBox2->Location = System::Drawing::Point(533, 19);
			this->groupBox2->Name = L"groupBox2";
			this->groupBox2->Size = System::Drawing::Size(211, 72);
			this->groupBox2->TabIndex = 2;
			this->groupBox2->TabStop = false;
			this->groupBox2->Text = L"Benzerlik";
			// 
			// label3
			// 
			this->label3->AutoSize = true;
			this->label3->Location = System::Drawing::Point(6, 37);
			this->label3->Name = L"label3";
			this->label3->Size = System::Drawing::Size(50, 13);
			this->label3->TabIndex = 6;
			this->label3->Text = L"Algoritma";
			// 
			// cbDistanceAlgorithm
			// 
			this->cbDistanceAlgorithm->FormattingEnabled = true;
			this->cbDistanceAlgorithm->Location = System::Drawing::Point(71, 34);
			this->cbDistanceAlgorithm->Name = L"cbDistanceAlgorithm";
			this->cbDistanceAlgorithm->Size = System::Drawing::Size(134, 21);
			this->cbDistanceAlgorithm->TabIndex = 6;
			// 
			// ofdSelectPatternFile
			// 
			this->ofdSelectPatternFile->FileName = L"openFileDialog1";
			// 
			// clbSensorTypes
			// 
			this->clbSensorTypes->Anchor = static_cast<System::Windows::Forms::AnchorStyles>(((System::Windows::Forms::AnchorStyles::Top | System::Windows::Forms::AnchorStyles::Bottom)
				| System::Windows::Forms::AnchorStyles::Left));
			this->clbSensorTypes->CheckOnClick = true;
			this->clbSensorTypes->FormattingEnabled = true;
			this->clbSensorTypes->Location = System::Drawing::Point(12, 19);
			this->clbSensorTypes->Name = L"clbSensorTypes";
			this->clbSensorTypes->Size = System::Drawing::Size(167, 499);
			this->clbSensorTypes->TabIndex = 15;
			this->clbSensorTypes->ItemCheck += gcnew System::Windows::Forms::ItemCheckEventHandler(this, &TestMain::clbSensorTypes_ItemCheck);
			// 
			// cPattern
			// 
			this->cPattern->Anchor = static_cast<System::Windows::Forms::AnchorStyles>((((System::Windows::Forms::AnchorStyles::Top | System::Windows::Forms::AnchorStyles::Bottom)
				| System::Windows::Forms::AnchorStyles::Left)
				| System::Windows::Forms::AnchorStyles::Right));
			chartArea3->Name = L"ChartArea1";
			chartArea4->Name = L"ChartArea2";
			this->cPattern->ChartAreas->Add(chartArea3);
			this->cPattern->ChartAreas->Add(chartArea4);
			legend2->Name = L"Legend1";
			this->cPattern->Legends->Add(legend2);
			this->cPattern->Location = System::Drawing::Point(195, 145);
			this->cPattern->Name = L"cPattern";
			series3->ChartArea = L"ChartArea1";
			series3->Legend = L"Legend1";
			series3->Name = L"Series1";
			series4->ChartArea = L"ChartArea2";
			series4->Legend = L"Legend1";
			series4->Name = L"Series2";
			this->cPattern->Series->Add(series3);
			this->cPattern->Series->Add(series4);
			this->cPattern->Size = System::Drawing::Size(902, 382);
			this->cPattern->TabIndex = 16;
			this->cPattern->Text = L"chart1";
			// 
			// button2
			// 
			this->button2->Anchor = static_cast<System::Windows::Forms::AnchorStyles>((System::Windows::Forms::AnchorStyles::Top | System::Windows::Forms::AnchorStyles::Right));
			this->button2->Location = System::Drawing::Point(987, 25);
			this->button2->Name = L"button2";
			this->button2->Size = System::Drawing::Size(97, 66);
			this->button2->TabIndex = 17;
			this->button2->Text = L"TEST";
			this->button2->UseVisualStyleBackColor = true;
			this->button2->Click += gcnew System::EventHandler(this, &TestMain::button2_Click);
			// 
			// cbCompareLastActions
			// 
			this->cbCompareLastActions->AutoSize = true;
			this->cbCompareLastActions->Checked = true;
			this->cbCompareLastActions->CheckState = System::Windows::Forms::CheckState::Checked;
			this->cbCompareLastActions->Location = System::Drawing::Point(533, 108);
			this->cbCompareLastActions->Name = L"cbCompareLastActions";
			this->cbCompareLastActions->Size = System::Drawing::Size(70, 17);
			this->cbCompareLastActions->TabIndex = 18;
			this->cbCompareLastActions->Text = L"Anlýk Veri";
			this->cbCompareLastActions->UseVisualStyleBackColor = true;
			this->cbCompareLastActions->CheckedChanged += gcnew System::EventHandler(this, &TestMain::cbLastActions_CheckedChanged);
			// 
			// groupBox3
			// 
			this->groupBox3->Controls->Add(this->lDistanceResult);
			this->groupBox3->Controls->Add(this->label2);
			this->groupBox3->Location = System::Drawing::Point(750, 25);
			this->groupBox3->Name = L"groupBox3";
			this->groupBox3->Size = System::Drawing::Size(200, 100);
			this->groupBox3->TabIndex = 19;
			this->groupBox3->TabStop = false;
			this->groupBox3->Text = L"Sonuç";
			// 
			// lDistanceResult
			// 
			this->lDistanceResult->AutoSize = true;
			this->lDistanceResult->Location = System::Drawing::Point(67, 23);
			this->lDistanceResult->Name = L"lDistanceResult";
			this->lDistanceResult->Size = System::Drawing::Size(10, 13);
			this->lDistanceResult->TabIndex = 1;
			this->lDistanceResult->Text = L"-";
			// 
			// label2
			// 
			this->label2->AutoSize = true;
			this->label2->Location = System::Drawing::Point(16, 23);
			this->label2->Name = L"label2";
			this->label2->Size = System::Drawing::Size(45, 13);
			this->label2->TabIndex = 0;
			this->label2->Text = L"Uzaklýk:";
			// 
			// button3
			// 
			this->button3->Location = System::Drawing::Point(288, 94);
			this->button3->Name = L"button3";
			this->button3->Size = System::Drawing::Size(21, 20);
			this->button3->TabIndex = 16;
			this->button3->Text = L"+";
			this->button3->UseVisualStyleBackColor = true;
			this->button3->Click += gcnew System::EventHandler(this, &TestMain::button3_Click);
			// 
			// TestMain
			// 
			this->AutoScaleDimensions = System::Drawing::SizeF(6, 13);
			this->AutoScaleMode = System::Windows::Forms::AutoScaleMode::Font;
			this->ClientSize = System::Drawing::Size(1096, 539);
			this->Controls->Add(this->groupBox3);
			this->Controls->Add(this->cbCompareLastActions);
			this->Controls->Add(this->button2);
			this->Controls->Add(this->cPattern);
			this->Controls->Add(this->clbSensorTypes);
			this->Controls->Add(this->groupBox2);
			this->Controls->Add(this->groupBox1);
			this->Name = L"TestMain";
			this->Text = L"TestMain";
			this->Load += gcnew System::EventHandler(this, &TestMain::TestMain_Load);
			this->groupBox1->ResumeLayout(false);
			this->groupBox1->PerformLayout();
			this->groupBox2->ResumeLayout(false);
			this->groupBox2->PerformLayout();
			(cli::safe_cast<System::ComponentModel::ISupportInitialize^>(this->cPattern))->EndInit();
			this->groupBox3->ResumeLayout(false);
			this->groupBox3->PerformLayout();
			this->ResumeLayout(false);
			this->PerformLayout();

		}
#pragma endregion
	private: System::Void button1_Click(System::Object^  sender, System::EventArgs^  e) 
	{
		System::Windows::Forms::DialogResult result = ofdSelectPatternFile->ShowDialog();

		if (result == System::Windows::Forms::DialogResult::OK) {

			String ^file_path = ofdSelectPatternFile->FileName;
			IO::Stream ^p_file_stream = ofdSelectPatternFile->OpenFile();

			IO::StreamReader ^reader = gcnew IO::StreamReader(p_file_stream);

			String ^line;

			int i = 0;
			while ((line = reader->ReadLine()) != nullptr) {
				array<float> ^ sensor_data = gcnew array<float>(clbSensorTypes->Items->Count + 1);
				array<String ^> ^sensor_str_data = line->Split();

				for (int i = 0; i < sensor_str_data->Length; i++) {

					if (sensor_str_data[i] != "") {
						sensor_data[i] = Convert::ToSingle(sensor_str_data[i]);
					}

				}

				if (i >= ALGORITHM_TEST_MAX_SAMPLE_COUNT) {
					break;
				}
				else {
					pActionArrayBuffer[i] = sensor_data;
					i++;
				}
			}

			LoadedSampleLength = (i - 1);
			lLoadedSampleLength->Text = LoadedSampleLength.ToString();

			for (int i = 0; i < LoadedSampleLength; i++) {

				for (int j = 1; j < pActionArrayBuffer[i]->Length; j++) {
					//cPattern->Series[j - 1]->Points->AddXY(p_events[i][0], p_events[i][j]); // Discard timestamp
					cPattern->Series[j - 1]->Points->AddY(pActionArrayBuffer[i][j]); // Discard timestamp
				}

			}
			cPattern->ChartAreas[0]->AxisY->MaximumAutoSize = true;
			cPattern->Update();




		}
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
			series->YValuesPerPoint = 1;

		}

		pCrossComponentSeries = cPattern->Series->Add("CrossComponent");
		pCrossComponentSeries->ChartArea = L"ChartArea2";
		pCrossComponentSeries->ChartType = DataVisualization::Charting::SeriesChartType::FastLine;
		pCrossComponentSeries->Enabled = true;
		pCrossComponentSeries->YValuesPerPoint = 1;





	}
private: System::Void TestMain_Load(System::Object^  sender, System::EventArgs^  e) {
}
private: System::Void textBox1_TextChanged(System::Object^  sender, System::EventArgs^  e) {

	double max_val;
	if (Double::TryParse(((TextBox ^)sender)->Text, max_val)) {

		if (max_val > cPattern->ChartAreas[0]->AxisY->Minimum) {
			cPattern->ChartAreas[0]->AxisY->Maximum = max_val;
			cPattern->ChartAreas[1]->AxisY->Maximum = max_val;
		}
	}
}
private: System::Void textBox2_TextChanged(System::Object^  sender, System::EventArgs^  e) 
{
	double min_val;
	if (Double::TryParse(((TextBox ^)sender)->Text, min_val)) {
		if (min_val < cPattern->ChartAreas[0]->AxisY->Maximum) {
			cPattern->ChartAreas[0]->AxisY->Minimum = min_val;
			cPattern->ChartAreas[1]->AxisY->Minimum = min_val;

		}
	}
}
private: System::Void cbLastActions_CheckedChanged(System::Object^  sender, System::EventArgs^  e) {
	if (cbCompareLastActions->Checked) {
		tbFirstCompareSample->Enabled = false;
		tbLastCompareSample->Enabled = false;
	}
	else {

		tbFirstCompareSample->Enabled = true;
		tbLastCompareSample->Enabled = true;
	}
}
private: System::Void clbSensorTypes_ItemCheck(System::Object^  sender, System::Windows::Forms::ItemCheckEventArgs^  e) 
{
	DataComponent ^p_component = (DataComponent ^)clbSensorTypes->Items[e->Index];

	for (int ix = 0; ix < clbSensorTypes->Items->Count; ++ix) {
		if (ix != e->Index) {
			clbSensorTypes->SetItemChecked(ix, false);
		}
	}


	if (nullptr != cPattern->Series && cPattern->Series->Count > 0) {
		cPattern->Series[e->Index]->Enabled = (System::Windows::Forms::CheckState::Checked == e->NewValue);
		cPattern->ChartAreas[0]->RecalculateAxesScale();


		RefreshCrossSeries(cPattern->Series[e->Index]);
	}


	
}
private: void RefreshCrossSeries(DataVisualization::Charting::Series ^p_current_series)
{
	pCrossComponentSeries->Points->Clear();

	for (int i = 0; i < p_current_series->Points->Count; i++) {

		pCrossComponentSeries->Points->AddXY(p_current_series->Points[i]->XValue, p_current_series->Points[i]->YValues[0]);
	}
	cPattern->ChartAreas[1]->RecalculateAxesScale();


}
private: System::Void tbFirstSample_TextChanged(System::Object^  sender, System::EventArgs^  e) {

	double val;
	if (Double::TryParse(((TextBox ^)sender)->Text, val)) {

		if (val < cPattern->ChartAreas[0]->AxisX->Maximum) {
			cPattern->ChartAreas[0]->AxisX->Minimum = val;
			cPattern->ChartAreas[0]->RecalculateAxesScale();
		}
	}
}
private: System::Void tbLastSample_TextChanged(System::Object^  sender, System::EventArgs^  e) 
{
	double val;
	if (Double::TryParse(((TextBox ^)sender)->Text, val)) {

		if (val > cPattern->ChartAreas[0]->AxisX->Minimum) {
			cPattern->ChartAreas[0]->AxisX->Maximum = val;
			cPattern->ChartAreas[0]->RecalculateAxesScale();
		}
	}
}
private: System::Void tbFirstCompareSample_TextChanged(System::Object^  sender, System::EventArgs^  e) 
{
	double val;
	if (Double::TryParse(((TextBox ^)sender)->Text, val)) {

		if (val < cPattern->ChartAreas[1]->AxisX->Maximum) {
			cPattern->ChartAreas[1]->AxisX->Minimum = val;
			cPattern->ChartAreas[1]->RecalculateAxesScale();
		}
	}
}
private: System::Void tbLastCompareSample_TextChanged(System::Object^  sender, System::EventArgs^  e) 
{
	double val;
	if (Double::TryParse(((TextBox ^)sender)->Text, val)) {

		if (val > cPattern->ChartAreas[1]->AxisX->Minimum) {
			cPattern->ChartAreas[1]->AxisX->Maximum = val;
			cPattern->ChartAreas[1]->RecalculateAxesScale();
		}
	}
}
private: System::Void button2_Click(System::Object^  sender, System::EventArgs^  e) 
{
	unsigned int first_sample = Convert::ToUInt32(tbFirstSample->Text);
	unsigned int last_sample = Convert::ToUInt32(tbLastSample->Text);
	unsigned int first_cross_sample = Convert::ToUInt32(tbFirstCompareSample->Text);
	unsigned int last_cross_sample = Convert::ToUInt32(tbLastCompareSample->Text);

	unsigned int sample_length = (last_sample - first_sample) + 1;
	unsigned int cross_sample_length = (last_cross_sample - first_cross_sample) + 1;

	array<int> ^sensor_data_type_index = gcnew array<int>(1);
	sensor_data_type_index[0] = 0;

	DataVisualization::Charting::Series ^p_selected_series;

	array<array<float> ^>^sample = gcnew array<array<float>^>(sample_length);
	array<array<float> ^>^cross_sample = gcnew array<array<float>^>(cross_sample_length);

	
	for each(DataVisualization::Charting::Series ^p_series in cPattern->Series) {

		if (p_series->Enabled) {
			p_selected_series = p_series;
			break;
		}
	}

	for (int i = 0; i < sample_length; i++) {
		array<float> ^p_sample = gcnew array<float>(1);
		p_sample[0] = p_selected_series->Points[first_sample + i]->YValues[0];
		sample[i] = p_sample;
	}

	for (int i = 0; i < cross_sample_length; i++) {
		array<float> ^p_sample = gcnew array<float>(1);
		p_sample[0] = p_selected_series->Points[first_cross_sample + i]->YValues[0];
		cross_sample[i] = p_sample;
	}

	array<double> ^ distance = ((SignalProcessing::Operation ^)cbDistanceAlgorithm->Items[cbDistanceAlgorithm->SelectedIndex])->Apply(sample, cross_sample, sensor_data_type_index);;


	lDistanceResult->Text = Convert::ToString(distance[0]);
}
private: System::Void button3_Click(System::Object^  sender, System::EventArgs^  e) 
{
	unsigned int first_cross_sample = Convert::ToUInt32(tbFirstCompareSample->Text);
	unsigned int last_cross_sample = Convert::ToUInt32(tbLastCompareSample->Text);

	first_cross_sample++;
	last_cross_sample++;

	tbFirstCompareSample->Text = Convert::ToString(first_cross_sample);
	tbLastCompareSample->Text = Convert::ToString(last_cross_sample);

	button2_Click(nullptr, nullptr);
}
};
}
