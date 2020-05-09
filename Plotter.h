#pragma once

namespace BNOControl {

	using namespace System;
	using namespace System::ComponentModel;
	using namespace System::Collections;
	using namespace System::Windows::Forms;
	using namespace System::Data;
	using namespace System::Drawing;

	/// <summary>
	/// Summary for Plotter
	/// </summary>
	public ref class Plotter : public System::Windows::Forms::Form
	{
	private:
		float Rate;
		unsigned int ReceivedMessageCount;
		unsigned int TotalReceivedMessageCount;
		bool FlushRater;
		static const unsigned int  buffer = 512;
	private: System::Windows::Forms::Timer^  timerBandwidthRater;
	private: System::Windows::Forms::Label^  label1;
	private: System::Windows::Forms::Label^  lRate;
			 long MilliSeconds;
			 ArrayList ^pYValuesArray;
	private: System::Windows::Forms::Label^  lPatternMSaaa;
	private: System::Windows::Forms::Label^  lPatternMS;
	private: System::Windows::Forms::Label^  label4;
	private: System::Windows::Forms::Label^  lActionMS;
	private: System::Windows::Forms::Panel^  panelTrigger;

			 ArrayList ^pXValuesArray;
	public:
		Plotter(void)
		{
			InitializeComponent();
			ReceivedMessageCount = 0;
			TotalReceivedMessageCount = 0;
			pChart->ResetAutoValues();
			pXValuesArray = gcnew ArrayList();
			pYValuesArray = gcnew ArrayList();
			//
			//TODO: Add the constructor code here
			//
		}

	protected:
		/// <summary>
		/// Clean up any resources being used.
		/// </summary>
		~Plotter()
		{
			if (components)
			{
				delete components;
			}
		}
	private: System::Windows::Forms::DataVisualization::Charting::Chart^  pChart;
	private: System::ComponentModel::IContainer^  components;
	protected:

	protected:

	private:
		/// <summary>
		/// Required designer variable.
		/// </summary>


#pragma region Windows Form Designer generated code
		/// <summary>
		/// Required method for Designer support - do not modify
		/// the contents of this method with the code editor.
		/// </summary>
		void InitializeComponent(void)
		{
			this->components = (gcnew System::ComponentModel::Container());
			System::Windows::Forms::DataVisualization::Charting::ChartArea^  chartArea1 = (gcnew System::Windows::Forms::DataVisualization::Charting::ChartArea());
			System::Windows::Forms::DataVisualization::Charting::Legend^  legend1 = (gcnew System::Windows::Forms::DataVisualization::Charting::Legend());
			System::Windows::Forms::DataVisualization::Charting::Series^  series1 = (gcnew System::Windows::Forms::DataVisualization::Charting::Series());
			this->pChart = (gcnew System::Windows::Forms::DataVisualization::Charting::Chart());
			this->timerBandwidthRater = (gcnew System::Windows::Forms::Timer(this->components));
			this->label1 = (gcnew System::Windows::Forms::Label());
			this->lRate = (gcnew System::Windows::Forms::Label());
			this->lPatternMSaaa = (gcnew System::Windows::Forms::Label());
			this->lPatternMS = (gcnew System::Windows::Forms::Label());
			this->label4 = (gcnew System::Windows::Forms::Label());
			this->lActionMS = (gcnew System::Windows::Forms::Label());
			this->panelTrigger = (gcnew System::Windows::Forms::Panel());
			(cli::safe_cast<System::ComponentModel::ISupportInitialize^>(this->pChart))->BeginInit();
			this->SuspendLayout();
			// 
			// pChart
			// 
			this->pChart->Anchor = static_cast<System::Windows::Forms::AnchorStyles>((((System::Windows::Forms::AnchorStyles::Top | System::Windows::Forms::AnchorStyles::Bottom)
				| System::Windows::Forms::AnchorStyles::Left)
				| System::Windows::Forms::AnchorStyles::Right));
			chartArea1->Name = L"ChartArea1";
			this->pChart->ChartAreas->Add(chartArea1);
			legend1->Name = L"Legend1";
			this->pChart->Legends->Add(legend1);
			this->pChart->Location = System::Drawing::Point(0, 0);
			this->pChart->Name = L"pChart";
			series1->ChartArea = L"ChartArea1";
			series1->ChartType = System::Windows::Forms::DataVisualization::Charting::SeriesChartType::Spline;
			series1->Legend = L"Legend1";
			series1->Name = L"Series1";
			this->pChart->Series->Add(series1);
			this->pChart->Size = System::Drawing::Size(677, 350);
			this->pChart->TabIndex = 0;
			this->pChart->Text = L"chart1";
			// 
			// timerBandwidthRater
			// 
			this->timerBandwidthRater->Enabled = true;
			this->timerBandwidthRater->Interval = 1000;
			this->timerBandwidthRater->Tick += gcnew System::EventHandler(this, &Plotter::timerBandwidthRater_Tick);
			// 
			// label1
			// 
			this->label1->Anchor = static_cast<System::Windows::Forms::AnchorStyles>((System::Windows::Forms::AnchorStyles::Bottom | System::Windows::Forms::AnchorStyles::Right));
			this->label1->AutoSize = true;
			this->label1->Location = System::Drawing::Point(620, 297);
			this->label1->Name = L"label1";
			this->label1->Size = System::Drawing::Size(44, 13);
			this->label1->TabIndex = 1;
			this->label1->Text = L"mesaj/s";
			// 
			// lRate
			// 
			this->lRate->Anchor = static_cast<System::Windows::Forms::AnchorStyles>((System::Windows::Forms::AnchorStyles::Bottom | System::Windows::Forms::AnchorStyles::Right));
			this->lRate->AutoSize = true;
			this->lRate->Location = System::Drawing::Point(592, 297);
			this->lRate->Name = L"lRate";
			this->lRate->Size = System::Drawing::Size(10, 13);
			this->lRate->TabIndex = 2;
			this->lRate->Text = L"-";
			// 
			// lPatternMSaaa
			// 
			this->lPatternMSaaa->AutoSize = true;
			this->lPatternMSaaa->Location = System::Drawing::Point(551, 218);
			this->lPatternMSaaa->Name = L"lPatternMSaaa";
			this->lPatternMSaaa->Size = System::Drawing::Size(60, 13);
			this->lPatternMSaaa->TabIndex = 3;
			this->lPatternMSaaa->Text = L"PatternMS:";
			// 
			// lPatternMS
			// 
			this->lPatternMS->AutoSize = true;
			this->lPatternMS->Location = System::Drawing::Point(629, 218);
			this->lPatternMS->Name = L"lPatternMS";
			this->lPatternMS->Size = System::Drawing::Size(10, 13);
			this->lPatternMS->TabIndex = 4;
			this->lPatternMS->Text = L"-";
			// 
			// label4
			// 
			this->label4->AutoSize = true;
			this->label4->Location = System::Drawing::Point(555, 240);
			this->label4->Name = L"label4";
			this->label4->Size = System::Drawing::Size(56, 13);
			this->label4->TabIndex = 5;
			this->label4->Text = L"ActionMS:";
			// 
			// lActionMS
			// 
			this->lActionMS->AutoSize = true;
			this->lActionMS->Location = System::Drawing::Point(629, 240);
			this->lActionMS->Name = L"lActionMS";
			this->lActionMS->Size = System::Drawing::Size(10, 13);
			this->lActionMS->TabIndex = 6;
			this->lActionMS->Text = L"-";
			// 
			// panelTrigger
			// 
			this->panelTrigger->Location = System::Drawing::Point(558, 135);
			this->panelTrigger->Name = L"panelTrigger";
			this->panelTrigger->Size = System::Drawing::Size(106, 70);
			this->panelTrigger->TabIndex = 7;
			// 
			// Plotter
			// 
			this->AutoScaleDimensions = System::Drawing::SizeF(6, 13);
			this->AutoScaleMode = System::Windows::Forms::AutoScaleMode::Font;
			this->ClientSize = System::Drawing::Size(676, 347);
			this->Controls->Add(this->panelTrigger);
			this->Controls->Add(this->lActionMS);
			this->Controls->Add(this->label4);
			this->Controls->Add(this->lPatternMS);
			this->Controls->Add(this->lPatternMSaaa);
			this->Controls->Add(this->lRate);
			this->Controls->Add(this->label1);
			this->Controls->Add(this->pChart);
			this->Name = L"Plotter";
			this->Text = L"Plotter";
			(cli::safe_cast<System::ComponentModel::ISupportInitialize^>(this->pChart))->EndInit();
			this->ResumeLayout(false);
			this->PerformLayout();

		}
#pragma endregion
	public:
		void ResetChart()
		{
			while (pChart->Series->Count > 0) {
				pChart->Series->RemoveAt(0);
			}
			
		}
		void IncrementReceivedMessageCount()
		{
			ReceivedMessageCount++;
			TotalReceivedMessageCount++;
#if 0
			if (TotalReceivedMessageCount > 512) {
				pChart->ChartAreas[0]->AxisX->ScaleView->Position = pChart->Series[0]->Points[pChart->Series[0]->Points->Count - 350]->XValue;
				pChart->ChartAreas[0]->AxisX->ScaleView->Size = 5;
				//pChart->ChartAreas[0]->AxisX->


			}
#endif
		}
		void SetFormTitle(String ^title)
		{
			this->Text = title;
		}
		
		System::Windows::Forms::DataVisualization::Charting::Series ^ AddNewSeries(String ^series_name) {

			System::Windows::Forms::DataVisualization::Charting::Series^  series = (gcnew System::Windows::Forms::DataVisualization::Charting::Series());
			series->ChartArea = L"ChartArea1";
			series->Legend = L"Legend1";
			series->Name = series_name;
			series->ChartType = System::Windows::Forms::DataVisualization::Charting::SeriesChartType::Line;
			pChart->Series->Add(series);

			return series;

		}

		/*
		System::Windows::Forms::DataVisualization::Charting::Series ^ AddNewSeries(String ^series_name) {

			System::Windows::Forms::DataVisualization::Charting::Series^  series = (gcnew System::Windows::Forms::DataVisualization::Charting::Series());
			series->ChartArea = L"ChartArea1";
			series->Legend = L"Legend1";
			series->Name = series_name;
			series->ChartType = System::Windows::Forms::DataVisualization::Charting::SeriesChartType::Line;
			pChart->Series->Add(series);

			Collections::Generic::List<float> ^p_xval_ref = gcnew Collections::Generic::List<float>(buffer);
			Collections::Generic::List<float> ^p_yval_ref = gcnew Collections::Generic::List<float>(buffer);

			for (int j = 0; j < buffer; j++) {
				p_xval_ref->Add(0);
				p_yval_ref->Add(0);
			}

			pXValuesArray->Add(p_xval_ref);
			pYValuesArray->Add(p_yval_ref);

			
			series->Points->DataBindXY(p_xval_ref,  p_yval_ref);
			

			return series;
			
		}*/
		void ClearPoints()
		{
			for each(System::Windows::Forms::DataVisualization::Charting::Series ^series in pChart->Series)
			{
				series->Points->Clear();
				ReceivedMessageCount = 0;
				TotalReceivedMessageCount = 0;
			}
		}
		void AddPointXYBulk(float y_values[], float time, unsigned short start_at, unsigned short count)
		{
			time = time / 1000;
			pChart->Series->SuspendUpdates();
			for (int i = start_at; i < start_at + count; i++) {

				if (nullptr != pChart->Series[i]) {
					/*
					Collections::Generic::List<float> ^p_xval_ref = ((Collections::Generic::List<float> ^)pXValuesArray[i]);
					Collections::Generic::List<float> ^p_yval_ref = ((Collections::Generic::List<float> ^)pYValuesArray[i]);

					p_xval_ref[TotalReceivedMessageCount%buffer] = time;
					p_yval_ref[TotalReceivedMessageCount % buffer] = y_values[i];
					*/
					/*
					if (pChart->Series[i]->Points->Count > 512) {
						pChart->Series[i]->Points->RemoveAt(0);
					}*/
					pChart->Series[i]->Points->AddXY(time, y_values[i]);
					
				}

			}
			
			pChart->Series->ResumeUpdates();
			pChart->Update();
		}
		void AddPointXYBulk(float y_values[], float time, unsigned short count)
		{
			//pChart->Series->SuspendUpdates();

			for (int i = 0; i < count; i++) {
				int y= TotalReceivedMessageCount % buffer;

				if (nullptr != pChart->Series[i]) {
					/*
					Collections::Generic::List<float> ^p_xval_ref = ((Collections::Generic::List<float> ^)pXValuesArray[i]);
					Collections::Generic::List<float> ^p_yval_ref = ((Collections::Generic::List<float> ^)pYValuesArray[i]);
					
					p_xval_ref[TotalReceivedMessageCount % buffer] = time;
					p_yval_ref[TotalReceivedMessageCount % buffer] = y_values[i];
					*/

					pChart->Series[i]->Points->AddXY(time, y_values[i]);
					

					/*
					if (pChart->Series[i]->Points->Count >= 512) {
						pChart->Series[i]->Points[y]->SetValueY(y_values[i]);
						pChart->Series[i]->Points->Invalidate();
					
					}
					else {
						pChart->Series[i]->Points->AddXY(time, y_values[i]);
					}
					*/

				}

			}

			//pChart->Series->ResumeUpdates();
			
		}

		void AddPointXYBulk(System::Collections::Generic::List<float> ^y_values, float time)
		{
			for (int i = 0; i < y_values->Count; i++) {

				if (nullptr != pChart->Series[i]) {
					((Collections::Generic::List<float> ^)pXValuesArray[i])[TotalReceivedMessageCount] = y_values[i];
					((Collections::Generic::List<float> ^)pYValuesArray[i])[TotalReceivedMessageCount] = time;

					//pChart->Series[i]->Points->AddXY(y_values[i], time);
				}
				
			}
		}
		System::Windows::Forms::DataVisualization::Charting::Series ^GetSeriesByName(String ^name)
		{
			return pChart->Series[name];
		}
		System::Windows::Forms::DataVisualization::Charting::Chart^ GetChart()
		{
			return pChart;
		}
		public: void SetCorrelationOfCurrentFlow(double val) {

			lActionMS->Text = val.ToString();
		}
				

		public: void SetCorrelationOfPattern(double val) {

			lPatternMS->Text = val.ToString();
		}

		public: void SetTrigger(bool val) {

			if (val) {
				panelTrigger->BackColor = System::Drawing::Color::Green;
			}
			else {
				panelTrigger->BackColor = System::Drawing::Color::Crimson;
			}
		}
		
	private: System::Void timerBandwidthRater_Tick(System::Object^  sender, System::EventArgs^  e) {
		Rate = ReceivedMessageCount / (timerBandwidthRater->Interval / 1000);
		ReceivedMessageCount = 0;
		lRate->Text = Rate.ToString();

		

	}
};
}
