#include "MainForm.h"

using namespace System;
using namespace System::Windows::Forms;

extern bool FileSave;

[STAThread]
int Main(array < System::String^>^ args)
{
	try {
		Application::EnableVisualStyles();
		Application::SetCompatibleTextRenderingDefault(0);
		BNOControl::MainForm form;
		Application::Run(%form);
	}
	catch (System::OverflowException ^e) {

		int a = 0;
		a++;
	}
	
		return 0;
	


}