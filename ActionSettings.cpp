
#include "ActionSettings.h"

System::Void BNOControl::ActionSettings::bActionThreshold_Click(System::Object^  sender, System::EventArgs^  e) {

	double threshold = Convert::ToDouble(tbActionThreshold->Text);

	UpdateEvent(TargetAction, threshold);
}