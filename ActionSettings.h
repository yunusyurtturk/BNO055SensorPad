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
	private:
		double threshold;
		unsigned int TargetAction;
	public:
		ActionSettings(unsigned int target_action)
		{
			InitializeComponent();
			
			TargetAction = target_action;
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
			// ActionSettings
			// 
			this->AutoScaleDimensions = System::Drawing::SizeF(6, 13);
			this->AutoScaleMode = System::Windows::Forms::AutoScaleMode::Font;
			this->ClientSize = System::Drawing::Size(160, 148);
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
	};
}
