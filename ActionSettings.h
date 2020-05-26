#pragma once

namespace BNOControl {

	using namespace System;
	using namespace System::ComponentModel;
	using namespace System::Collections;
	using namespace System::Windows::Forms;
	using namespace System::Data;
	using namespace System::Drawing;

	/// <summary>
	/// Summary for ActionSettings
	/// </summary>
	public ref class ActionSettings : public System::Windows::Forms::Form
	{
	public:
		delegate void UpdateEventDelegate(unsigned int target_action, double threshold);
		UpdateEventDelegate ^UpdateEvent;

		delegate void FileSaveThresholdDelegate(unsigned int target_action, bool is_save_enabled, double threshold);
		FileSaveThresholdDelegate ^FileSaveThresholdMethod;

		delegate void ExportActionPatternDelegate(unsigned int target_action);
		ExportActionPatternDelegate ^ExportActionPattern;
	private:
		double threshold;
	private: System::Windows::Forms::Label^  label2;
	private: System::Windows::Forms::Label^  lActionNo;
	private: System::Windows::Forms::Label^  label3;
	private: System::Windows::Forms::TextBox^  tbFileSaveThreshold;
	private: System::Windows::Forms::CheckBox^  cbFileSaveEnabled;
	private: System::Windows::Forms::Button^  bFileSaveApply;
	private: System::Windows::Forms::Button^  button1;
			 unsigned int TargetAction;
	public:
		ActionSettings(unsigned int target_action)
		{
			InitializeComponent();
			
			TargetAction = target_action;

			lActionNo->Text = target_action.ToString();
		}
		property double Threshold{
			double get() {
				return threshold;
			}
			void set(double val) {
				threshold = val;
				this->tbActionThreshold->Text = val.ToString();
			}
		}

	protected:
		/// <summary>
		/// Clean up any resources being used.
		/// </summary>
		~ActionSettings()
		{
			if (components)
			{
				delete components;
			}
		}
	private: System::Windows::Forms::Label^  label1;
	private: System::Windows::Forms::TextBox^  tbActionThreshold;
	private: System::Windows::Forms::Button^  bActionThreshold;

	protected:

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
			this->label1 = (gcnew System::Windows::Forms::Label());
			this->tbActionThreshold = (gcnew System::Windows::Forms::TextBox());
			this->bActionThreshold = (gcnew System::Windows::Forms::Button());
			this->label2 = (gcnew System::Windows::Forms::Label());
			this->lActionNo = (gcnew System::Windows::Forms::Label());
			this->label3 = (gcnew System::Windows::Forms::Label());
			this->tbFileSaveThreshold = (gcnew System::Windows::Forms::TextBox());
			this->cbFileSaveEnabled = (gcnew System::Windows::Forms::CheckBox());
			this->bFileSaveApply = (gcnew System::Windows::Forms::Button());
			this->button1 = (gcnew System::Windows::Forms::Button());
			this->SuspendLayout();
			// 
			// label1
			// 
			this->label1->AutoSize = true;
			this->label1->Location = System::Drawing::Point(38, 48);
			this->label1->Name = L"label1";
			this->label1->Size = System::Drawing::Size(87, 13);
			this->label1->TabIndex = 0;
			this->label1->Text = L"Action Threshold";
			// 
			// tbActionThreshold
			// 
			this->tbActionThreshold->Location = System::Drawing::Point(31, 64);
			this->tbActionThreshold->Name = L"tbActionThreshold";
			this->tbActionThreshold->Size = System::Drawing::Size(100, 20);
			this->tbActionThreshold->TabIndex = 1;
			// 
			// bActionThreshold
			// 
			this->bActionThreshold->Location = System::Drawing::Point(31, 91);
			this->bActionThreshold->Name = L"bActionThreshold";
			this->bActionThreshold->Size = System::Drawing::Size(100, 23);
			this->bActionThreshold->TabIndex = 2;
			this->bActionThreshold->Text = L"Update";
			this->bActionThreshold->UseVisualStyleBackColor = true;
			this->bActionThreshold->Click += gcnew System::EventHandler(this, &ActionSettings::bActionThreshold_Click);
			// 
			// label2
			// 
			this->label2->AutoSize = true;
			this->label2->Location = System::Drawing::Point(41, 13);
			this->label2->Name = L"label2";
			this->label2->Size = System::Drawing::Size(40, 13);
			this->label2->TabIndex = 3;
			this->label2->Text = L"Action:";
			// 
			// lActionNo
			// 
			this->lActionNo->AutoSize = true;
			this->lActionNo->Location = System::Drawing::Point(87, 13);
			this->lActionNo->Name = L"lActionNo";
			this->lActionNo->Size = System::Drawing::Size(10, 13);
			this->lActionNo->TabIndex = 4;
			this->lActionNo->Text = L"-";
			// 
			// label3
			// 
			this->label3->AutoSize = true;
			this->label3->Location = System::Drawing::Point(191, 48);
			this->label3->Name = L"label3";
			this->label3->Size = System::Drawing::Size(101, 13);
			this->label3->TabIndex = 5;
			this->label3->Text = L"File Save Threshold";
			// 
			// tbFileSaveThreshold
			// 
			this->tbFileSaveThreshold->Location = System::Drawing::Point(298, 45);
			this->tbFileSaveThreshold->Name = L"tbFileSaveThreshold";
			this->tbFileSaveThreshold->Size = System::Drawing::Size(100, 20);
			this->tbFileSaveThreshold->TabIndex = 6;
			// 
			// cbFileSaveEnabled
			// 
			this->cbFileSaveEnabled->AutoSize = true;
			this->cbFileSaveEnabled->Location = System::Drawing::Point(206, 71);
			this->cbFileSaveEnabled->Name = L"cbFileSaveEnabled";
			this->cbFileSaveEnabled->RightToLeft = System::Windows::Forms::RightToLeft::Yes;
			this->cbFileSaveEnabled->Size = System::Drawing::Size(106, 17);
			this->cbFileSaveEnabled->TabIndex = 7;
			this->cbFileSaveEnabled->Text = L"File Save Enable";
			this->cbFileSaveEnabled->UseVisualStyleBackColor = true;
			// 
			// bFileSaveApply
			// 
			this->bFileSaveApply->Location = System::Drawing::Point(323, 91);
			this->bFileSaveApply->Name = L"bFileSaveApply";
			this->bFileSaveApply->Size = System::Drawing::Size(75, 23);
			this->bFileSaveApply->TabIndex = 8;
			this->bFileSaveApply->Text = L"Apply";
			this->bFileSaveApply->UseVisualStyleBackColor = true;
			this->bFileSaveApply->Click += gcnew System::EventHandler(this, &ActionSettings::bFileSaveApply_Click);
			// 
			// button1
			// 
			this->button1->Location = System::Drawing::Point(298, 134);
			this->button1->Name = L"button1";
			this->button1->Size = System::Drawing::Size(100, 55);
			this->button1->TabIndex = 9;
			this->button1->Text = L"Save Action Pattern";
			this->button1->UseVisualStyleBackColor = true;
			this->button1->Click += gcnew System::EventHandler(this, &ActionSettings::button1_Click);
			// 
			// ActionSettings
			// 
			this->AutoScaleDimensions = System::Drawing::SizeF(6, 13);
			this->AutoScaleMode = System::Windows::Forms::AutoScaleMode::Font;
			this->ClientSize = System::Drawing::Size(441, 201);
			this->Controls->Add(this->button1);
			this->Controls->Add(this->bFileSaveApply);
			this->Controls->Add(this->cbFileSaveEnabled);
			this->Controls->Add(this->tbFileSaveThreshold);
			this->Controls->Add(this->label3);
			this->Controls->Add(this->lActionNo);
			this->Controls->Add(this->label2);
			this->Controls->Add(this->bActionThreshold);
			this->Controls->Add(this->tbActionThreshold);
			this->Controls->Add(this->label1);
			this->Name = L"ActionSettings";
			this->Text = L"ActionSettings";
			this->ResumeLayout(false);
			this->PerformLayout();

		}
#pragma endregion
	private: System::Void bActionThreshold_Click(System::Object^  sender, System::EventArgs^  e);
	private: System::Void button1_Click(System::Object^  sender, System::EventArgs^  e) {
		
		

		ExportActionPattern(TargetAction);
	}
private: System::Void bFileSaveApply_Click(System::Object^  sender, System::EventArgs^  e) {

	double threshold;
	if (Double::TryParse(tbFileSaveThreshold->Text, threshold))
	{
		bool is_save_enabled = cbFileSaveEnabled->Checked;
		FileSaveThresholdMethod(TargetAction, is_save_enabled, threshold);
	}

}
};
}
