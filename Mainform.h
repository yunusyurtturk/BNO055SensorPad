#pragma once

#include <stdio.h>

#include "ViewTypes.h"
#include "SerialPort\SerialControl.h"
#include "Plotter.h"
#include "ActionControlTrainerTypes.h"
#include "SensorValueOps.h"
#include "RelativeMovement.h"
#include "PatternLoader.h"
#include "MainformOps.h"
#include "ActionDefinition.h"
#include"PatternMatcher.h"
#include "KalmanEstimator.h"
#include "DataComponent.h"
#include "MSE3D.h"
#include "MadgwickAHRS.h"

#include <sensor_processing_lib.h>
#include <quaternion.h>
#include "Math3D.h"
#include "UISettingsSaver.h"
#include "ActionArrayBuffer.h"
#include "TestMain.h"
#include "ActionSettings.h"
#include "SensorDataTypes.h"


#define ACTION_BUFFER_ARRAY_LENGTH			100
#define		DEGREE_TO_RADIAN_CONSTANT		(0.0174532925)
#define	SENSOR_ACC_DATA_AVG_SIZE	8


bool FileSave = false;
unsigned int SaveDirIndex = 0;
double FileSaveThreshold = -1;

namespace BNOControl {

	using namespace System;
	using namespace System::ComponentModel;
	using namespace System::Collections;
	using namespace System::Windows::Forms;
	using namespace System::Data;
	using namespace System::Drawing;
	using namespace BNOUserControls;
	using namespace System::Net::Sockets;

	using namespace vJoyInterfaceWrap;
	/// <summary>
	/// Summary for MainForm
	/// </summary>

	

	public ref class MainForm : public System::Windows::Forms::Form, public IMainFormOps
	{
	private:
		static int counter = 0;
		vJoy ^pJoystick;
		vJoy::JoystickState ^iReport;
		long long MaxAxisVal;
		int FoundDirectionID;
		UdpClient ^pUDPClient;
		Quaternion *pRefQuaternion;
		Quaternion *pDirectionQuaternions;
		Quaternion *pStandpointQuaternion;
		array<Control ^> ^pDirectionQuatControls;
		array<Control ^>^pJoyButtonList;
		array<boolean> ^pJoyButtons;
		delegate void SerialDelegate(ViewSerialEvent p_event);
		Madgwick *pMadgwick;
		unsigned int MaxSampleSize;
		array<SignalProcessing::KalmanEstimator ^>^pKalmanEstimators;
		Dictionary<unsigned int, ActionDefinition ^> ^pActions;
		List<SignalProcessing::OpChain ^> ^pOpChains;
		ActionBuffer ^pActionBuffer;
		ActionArrayBuffer ^pActionArrayBuffer;
		ActionBuffer ^pFilteredActionBuffer;
		bool SamplingInProgress;
		IO::FileStream ^pFileStream;
		System::IO::TextWriter ^pTextWriter;

		array<float> ^PreviousSensorDataArray;

		CSerialControl ^SerialControl;
		double LastSumRoot;
		double CurrentSumRoot;
		unsigned int message_count = 0;
		array<Byte> ^SerialReadBuffer;
		int SerialBufferDataInputMarker = 0;
		int LastSerialBufferDataInputMarkerPoint = 0;
		int SerialBufferDataReadMarker = 0;
		int BufferReset = 0;
		unsigned int TotalReadBytesCount = 0;
		unsigned int BufferSizeDecreased = 0;
		array<float> ^LastSamples;
		bool IMUNavigationReset = false;
		array<array<float> ^>^pLastAccValues;
		boolean addDataPointsFlag;
		property boolean AddDataPointsFlag {
			boolean get() {
				return addDataPointsFlag;
			}
			void set(boolean val) {
				addDataPointsFlag = val;
				cbAddDataPointsFlag->Checked = val;
			}
		}

	


		float * ned_abs_pos_3d;
		float * ned_abs_vel_3d;

		int offset_counter;
		double total_acc_x;
		double total_acc_y;
		double total_acc_z;
		bool AccOffsetCalcInProgress;

	
		RelativeMovement ^pRelativePositon;
		RelativeMovement ^pRelativeVelocity;
		List<array<ViewSerialEvent> ^> ^Patterns;
		List<BNOControl::Plotter ^> ^PatternPlotters;
		List<int> ^PatternSensorDataIndex;
		List<System::Drawing::Color> ^ActionColors;

		float *acc_offset;
		System::Random ^pRandom;
		IO::StreamWriter ^GnuPlotSW;
		IO::StreamReader ^GnuPlotSR;

	private: System::Windows::Forms::GroupBox^  groupBox1;
	private: System::Windows::Forms::Label^  label3;
	private: System::Windows::Forms::Label^  label2;
	private: System::Windows::Forms::Label^  label1;
	private: System::Windows::Forms::TextBox^  tbAccZ;

	private: System::Windows::Forms::TextBox^  tbAccY;

	private: System::Windows::Forms::TextBox^  tbAccX;
	private: System::Windows::Forms::GroupBox^  groupBox2;
	private: System::Windows::Forms::Label^  label4;
	private: System::Windows::Forms::Label^  label5;
	private: System::Windows::Forms::Label^  label6;
	private: System::Windows::Forms::TextBox^  orientZ;

	private: System::Windows::Forms::TextBox^  orientY;

	private: System::Windows::Forms::TextBox^  orientX;
	private: System::Windows::Forms::GroupBox^  groupBox3;
	private: System::Windows::Forms::Label^  label7;
	private: System::Windows::Forms::Label^  label8;
	private: System::Windows::Forms::Label^  label9;
	private: System::Windows::Forms::TextBox^  tbMagneticZ;

	private: System::Windows::Forms::TextBox^  tbMagneticY;

	private: System::Windows::Forms::TextBox^  tbMagneticX;
	private: System::Windows::Forms::GroupBox^  groupBox4;
	private: System::Windows::Forms::Label^  label10;
	private: System::Windows::Forms::Label^  label11;
	private: System::Windows::Forms::Label^  label12;
	private: System::Windows::Forms::TextBox^  tbGyroZ;

	private: System::Windows::Forms::TextBox^  tbGyroY;

	private: System::Windows::Forms::TextBox^  tbGyroX;


	private: System::Windows::Forms::TextBox^  tbVersion;
	private: System::Windows::Forms::TextBox^  tbSensorID;
	private: System::Windows::Forms::TextBox^  tbType;



	private: System::Windows::Forms::TextBox^  tbTimestamp;
	private: System::Windows::Forms::Label^  label14;
	private: System::Windows::Forms::Label^  label15;
	private: System::Windows::Forms::Label^  label16;
	private: System::Windows::Forms::Label^  label17;
	private: System::Windows::Forms::TextBox^  tbInvalidMessage;
	private: System::Windows::Forms::GroupBox^  groupBox5;
	private: System::Windows::Forms::Label^  label18;
	private: System::Windows::Forms::TextBox^  tbQuatW;

	private: System::Windows::Forms::Label^  label19;
	private: System::Windows::Forms::Label^  label20;
	private: System::Windows::Forms::Label^  label21;
	private: System::Windows::Forms::TextBox^  tbQuatZ;

	private: System::Windows::Forms::TextBox^  tbQuatY;

	private: System::Windows::Forms::TextBox^  tbQuatX;
	private: System::Windows::Forms::DataVisualization::Charting::Chart^  sensorFlowChart;
	private: System::Windows::Forms::Button^  button1;



	private: System::Windows::Forms::Label^  label22;
	private: System::Windows::Forms::Label^  lTotalMessageCount;
	private: System::Windows::Forms::Timer^  timerSerialRead;
	private: System::IO::Ports::SerialPort^  serialPortBNO055;
	private: System::Windows::Forms::Label^  lBufferReset;
	private: System::Windows::Forms::Label^  label24;
	private: System::Windows::Forms::Label^  lTotalReceivedBytes;
	private: System::Windows::Forms::Label^  label25;
	private: System::Windows::Forms::Label^  lDifferenceBetweenReceivedBytes;
	private: System::Windows::Forms::Label^  label27;
	private: System::Windows::Forms::Label^  lBufferSizeDecreased;
	private: System::Windows::Forms::Label^  label26;
	private: System::Windows::Forms::Label^  lSerialBufferReadSize;
	private: System::Windows::Forms::Label^  label28;








			CIMUControl		^IMUControl;
			BNOControl::Plotter ^p_acc_plotter;
			BNOControl::Plotter ^p_quat_plotter;
			BNOControl::Plotter ^p_mag_plotter;
			BNOControl::Plotter ^p_position_plotter;
			BNOControl::Plotter ^p_velocity_plotter;
private: System::Windows::Forms::GroupBox^  groupBox6;
private: System::Windows::Forms::Label^  lNextAction;
private: System::Windows::Forms::Label^  label30;
private: System::Windows::Forms::Label^  lRemainingTimeToNextAction;
private: System::Windows::Forms::Label^  label23;
private: System::Windows::Forms::Timer^  tTimeToNextAction;
private: System::Windows::Forms::Panel^  panel1;
private: System::Windows::Forms::Button^  bToggleTraining;
private: System::Windows::Forms::Label^  lWorkInProgress;
private: System::Windows::Forms::Label^  lCurrentAction;
private: System::Windows::Forms::Button^  bOpenTrainingDirectory;



private: System::Windows::Forms::Button^  bLimitlessDataFlow;
private: System::Windows::Forms::Button^  button2;
private: System::Windows::Forms::GroupBox^  groupBox7;



private: System::Windows::Forms::GroupBox^  groupBox8;
private: System::Windows::Forms::Label^  label13;
private: System::Windows::Forms::Label^  label32;
private: System::Windows::Forms::Label^  label33;
private: System::Windows::Forms::TextBox^  tbAccD;
private: System::Windows::Forms::TextBox^  tbAccE;
private: System::Windows::Forms::TextBox^  tbAccN;
private: System::Windows::Forms::Button^  bSetAccOffset;
private: System::Windows::Forms::GroupBox^  groupBox9;
private: System::Windows::Forms::GroupBox^  groupBox10;
private: System::Windows::Forms::CheckedListBox^  clbDataComponents;

private: System::Windows::Forms::Label^  label31;
private: System::Windows::Forms::TextBox^  tbDataCount;
private: System::Windows::Forms::TextBox^  tbChartMin;
private: System::Windows::Forms::TextBox^  tbChartMax;
private: System::Windows::Forms::Label^  label34;
private: System::Windows::Forms::Label^  label35;
private: System::Windows::Forms::Label^  label36;
private: System::Windows::Forms::TextBox^  tbSystemCal;

private: System::Windows::Forms::GroupBox^  groupBox11;
private: System::Windows::Forms::Label^  label37;
private: System::Windows::Forms::Label^  label40;
private: System::Windows::Forms::TextBox^  tbMagCal;

private: System::Windows::Forms::Label^  label39;
private: System::Windows::Forms::TextBox^  tbAccCal;

private: System::Windows::Forms::Label^  label38;
private: System::Windows::Forms::TextBox^  gbGyroCal;
private: System::Windows::Forms::Label^  lTimeDiffMs;
private: System::Windows::Forms::Label^  label42;
private: System::Windows::Forms::Button^  tbRefreshVelPos;
private: System::Windows::Forms::GroupBox^  groupBox12;
private: System::Windows::Forms::Label^  label45;
private: System::Windows::Forms::TextBox^  tbQDistanceZ;

private: System::Windows::Forms::Label^  label44;
private: System::Windows::Forms::TextBox^  tbQDistanceY;

private: System::Windows::Forms::Label^  label43;
private: System::Windows::Forms::TextBox^  tbQDistanceX;

private: System::Windows::Forms::Label^  label41;
private: System::Windows::Forms::TextBox^  tbQDistanceW;
private: System::Windows::Forms::Label^  lQuaternionDistance;
private: System::Windows::Forms::GroupBox^  groupBox13;
private: System::Windows::Forms::Panel^  pDirectionQuat0;
















private: System::Windows::Forms::Label^  label46;
private: System::Windows::Forms::Panel^  pDirectionQuat8;

private: System::Windows::Forms::Label^  label52;
private: System::Windows::Forms::Panel^  pDirectionQuat5;

private: System::Windows::Forms::Label^  label49;
private: System::Windows::Forms::Panel^  pDirectionQuat7;

private: System::Windows::Forms::Label^  label53;
private: System::Windows::Forms::Panel^  pDirectionQuat2;

private: System::Windows::Forms::Label^  label48;
private: System::Windows::Forms::Panel^  pDirectionQuat6;

private: System::Windows::Forms::Label^  label54;
private: System::Windows::Forms::Panel^  pDirectionQuat4;

private: System::Windows::Forms::Label^  label50;
private: System::Windows::Forms::Panel^  pDirectionQuat1;

private: System::Windows::Forms::Label^  label47;
private: System::Windows::Forms::Panel^  pDirectionQuat3;

private: System::Windows::Forms::Label^  label51;
private: System::Windows::Forms::Button^  button3;
private: System::Windows::Forms::Panel^  pJoyButton1;

private: System::Windows::Forms::Label^  label55;
private: System::Windows::Forms::Label^  label56;
private: System::Windows::Forms::Panel^  pJoyButton2;

private: System::Windows::Forms::Label^  label57;
private: System::Windows::Forms::Panel^  pJoyButton3;

private: System::Windows::Forms::Label^  label58;
private: System::Windows::Forms::Panel^  pJoyButton4;

private: System::Windows::Forms::Label^  label59;
private: System::Windows::Forms::Panel^  pJoyButton5;

private: System::Windows::Forms::Label^  label60;
private: System::Windows::Forms::Panel^  pJoyButton6;
private: System::Windows::Forms::Panel^  pJoyButton8;


private: System::Windows::Forms::Label^  label61;
private: System::Windows::Forms::Panel^  pJoyButton7;

private: System::Windows::Forms::Label^  label62;
private: System::Windows::Forms::Button^  bAlgorithmTest;
private: System::Windows::Forms::Button^  bSaveSnapshot;
private: System::Windows::Forms::CheckBox^  cbAccOffset;
private: System::Windows::Forms::CheckBox^  cbAddDataPointsFlag;





private: System::Windows::Forms::Label^  label29;

	public:
		void InitUDPClient()
		{
			pUDPClient = gcnew UdpClient();
			
		}
		/*
		virtual void AddPattern(int action, array<int> ^sensor_type_data_index)
		{
			ActionDefinition ^pActionDefinition = nullptr;

			if (!pActions->ContainsKey(action)) {
				pActionDefinition = gcnew ActionDefinition();
				pActions->Add(action, pActionDefinition);



				SignalProcessing::OpChain ^p_op_chain = gcnew SignalProcessing::OpChain();
				p_op_chain->InsertOp(gcnew SignalProcessing::MSE3D());

				pActionDefinition->AddOpChain(p_op_chain);

			}
			else {
				pActionDefinition = pActions[action];
			}

		}*/
		virtual void AddPattern(int action, array<array<float> ^> ^pattern, array<int> ^sensor_data_offsets, double threshold, SignalProcessing::Operation ^p_distance_method) override
		{
			ActionDefinition ^pActionDefinition = nullptr;

			if (!pActions->ContainsKey(action)) {
				pActionDefinition = gcnew ActionDefinition();
				pActions->Add(action, pActionDefinition);
				pActionDefinition->SetSensorDataIndexs(sensor_data_offsets);
				pActionDefinition->SetOriginalPattern(pattern);
				pActionDefinition->SetThreshold(threshold);
				pActionDefinition->SetActionNo(action);
				pActionDefinition->SetDistanceMethod(p_distance_method);
				

				pActionDefinition->Init();
			}
			else {
				pActionDefinition = pActions[action];
			}
		}

		 
	public:
		MainForm(void)
		{
			InitializeComponent();

			InitializeVJOY();
			
			FoundDirectionID = -1;
			pRefQuaternion = new Quaternion();
			pStandpointQuaternion = new Quaternion();
			pDirectionQuaternions = new Quaternion[9];
			pDirectionQuatControls = gcnew array<Control ^>(9);

			pLastAccValues = gcnew array<array<float> ^ > (SENSOR_ACC_DATA_AVG_SIZE);
			for (int i = 0; i < SENSOR_ACC_DATA_AVG_SIZE; i++) 
			{
				pLastAccValues[i] = gcnew array<float>(3);

			}

			addDataPointsFlag = true;
			AddDataPointsFlag = true;

			ned_abs_pos_3d = new float[3];
			ned_abs_vel_3d = new float[3];

			ned_abs_vel_3d[0] = 0;
			ned_abs_vel_3d[1] = 0;
			ned_abs_vel_3d[2] = 0;

			ned_abs_pos_3d[0] = 0;
			ned_abs_pos_3d[1] = 0;
			ned_abs_pos_3d[2] = 0;

			pDirectionQuatControls[0] = pDirectionQuat0;
			pDirectionQuatControls[1] = pDirectionQuat1;
			pDirectionQuatControls[2] = pDirectionQuat2;
			pDirectionQuatControls[3] = pDirectionQuat3;
			pDirectionQuatControls[4] = pDirectionQuat4;
			pDirectionQuatControls[5] = pDirectionQuat5;
			pDirectionQuatControls[6] = pDirectionQuat6;
			pDirectionQuatControls[7] = pDirectionQuat7;
			pDirectionQuatControls[8] = pDirectionQuat8;

			pStandpointQuaternion->a = -1;
			pStandpointQuaternion->b = 0;
			pStandpointQuaternion->c = 0;
			pStandpointQuaternion->d = 0;


			pRefQuaternion->a = 0.5;
			pRefQuaternion->b = 0.5;
			pRefQuaternion->c = 0.5;
			pRefQuaternion->d = 0.5;

			for (int i = 0; i < 9; i++) {
				pDirectionQuaternions[i] = *pRefQuaternion;
			}


			pRefQuaternion->a = 0.5;
			pRefQuaternion->b = 0.5;
			pRefQuaternion->c = 0.5;
			pRefQuaternion->d = 0.5;
			MaxSampleSize = ACTION_BUFFER_ARRAY_LENGTH * 2;
			pMadgwick = new Madgwick();
			pMadgwick->begin(100.0f);

			InitUDPClient();

			pJoyButtonList = gcnew array<Control ^>(8);
			pJoyButtonList[0] = pJoyButton1;
			pJoyButtonList[1] = pJoyButton2;
			pJoyButtonList[2] = pJoyButton3;
			pJoyButtonList[3] = pJoyButton4;
			pJoyButtonList[4] = pJoyButton5;
			pJoyButtonList[5] = pJoyButton6;
			pJoyButtonList[6] = pJoyButton7;
			pJoyButtonList[7] = pJoyButton8;

			pJoyButtons = gcnew array<boolean>(8);
			pJoyButtons[0] = false;
			pJoyButtons[1] = false;
			pJoyButtons[2] = false;
			pJoyButtons[3] = false;
			pJoyButtons[4] = false;
			pJoyButtons[5] = false;
			pJoyButtons[6] = false;
			pJoyButtons[7] = false;

			





			pUISettingsSaver = gcnew UISettingsSaver("settings.xml");

			clbDataComponents->Items->Add(gcnew DataComponent("Euler 1", 0), false);
			clbDataComponents->Items->Add(gcnew DataComponent("Euler 2", 1), false);
			clbDataComponents->Items->Add(gcnew DataComponent("Euler 3", 2), false);

			clbDataComponents->Items->Add(gcnew DataComponent("Acc 1", 3), false);
			clbDataComponents->Items->Add(gcnew DataComponent("Acc 2", 4), false);
			clbDataComponents->Items->Add(gcnew DataComponent("Acc 3", 5), false);

			clbDataComponents->Items->Add(gcnew DataComponent("Mag 1", 6));
			clbDataComponents->Items->Add(gcnew DataComponent("Mag 2", 7));
			clbDataComponents->Items->Add(gcnew DataComponent("Mag 3", 8));

			clbDataComponents->Items->Add(gcnew DataComponent("Gyro 1 (rads/s)", 9), false);
			clbDataComponents->Items->Add(gcnew DataComponent("Gyro 2 (rads/s)", 10), false);
			clbDataComponents->Items->Add(gcnew DataComponent("Gyro 3 (rads/s)", 11), false);

			clbDataComponents->Items->Add(gcnew DataComponent("Quat w", 12), false);
			clbDataComponents->Items->Add(gcnew DataComponent("Quat x", 13), false);
			clbDataComponents->Items->Add(gcnew DataComponent("Quat y", 14), false);
			clbDataComponents->Items->Add(gcnew DataComponent("Quat z", 15), false);

			clbDataComponents->Items->Add(gcnew DataComponent("SysCal z", SENSOR_DATA_TYPE_COUNT + SENSOR_DATA_TYPE_CALIBRATION_STATUS_SYSTEM)	, false);
			clbDataComponents->Items->Add(gcnew DataComponent("GyroCal z", SENSOR_DATA_TYPE_COUNT + SENSOR_DATA_TYPE_CALIBRATION_STATUS_GYRO)	, false);
			clbDataComponents->Items->Add(gcnew DataComponent("AccCal z", SENSOR_DATA_TYPE_COUNT + SENSOR_DATA_TYPE_CALIBRATION_STATUS_ACC)	, false);
			clbDataComponents->Items->Add(gcnew DataComponent("MagCal z", SENSOR_DATA_TYPE_COUNT + SENSOR_DATA_TYPE_CALIBRATION_STATUS_MAG)	, false);

			clbDataComponents->Items->Add(gcnew DataComponent("NedAccX", SENSOR_DATA_TYPE_COUNT + SENSOR_DATA_TYPE_RELATIVE_ACCELERATION_NED_X), false);
			clbDataComponents->Items->Add(gcnew DataComponent("NedAccy", SENSOR_DATA_TYPE_COUNT + SENSOR_DATA_TYPE_RELATIVE_ACCELERATION_NED_Y), false);
			clbDataComponents->Items->Add(gcnew DataComponent("NedAccZ", SENSOR_DATA_TYPE_COUNT + SENSOR_DATA_TYPE_RELATIVE_ACCELERATION_NED_Z), false);

			clbDataComponents->Items->Add(gcnew DataComponent("VelN", SENSOR_DATA_TYPE_COUNT + SENSOR_DATA_TYPE_RELATIVE_VELOCITY_NED_X), false);
			clbDataComponents->Items->Add(gcnew DataComponent("VelE", SENSOR_DATA_TYPE_COUNT + SENSOR_DATA_TYPE_RELATIVE_VELOCITY_NED_Y), false);
			clbDataComponents->Items->Add(gcnew DataComponent("VelD", SENSOR_DATA_TYPE_COUNT + SENSOR_DATA_TYPE_RELATIVE_VELOCITY_NED_Z), false);

			clbDataComponents->Items->Add(gcnew DataComponent("PosN", SENSOR_DATA_TYPE_COUNT + SENSOR_DATA_TYPE_RELATIVE_POSITION_NED_X), false);
			clbDataComponents->Items->Add(gcnew DataComponent("PosE", SENSOR_DATA_TYPE_COUNT + SENSOR_DATA_TYPE_RELATIVE_POSITION_NED_Y), false);
			clbDataComponents->Items->Add(gcnew DataComponent("PosD", SENSOR_DATA_TYPE_COUNT + SENSOR_DATA_TYPE_RELATIVE_POSITION_NED_Z), false);


			/* Button/Action 'larin hesaplanma uzakligi*/
			clbDataComponents->Items->Add(gcnew DataComponent("DistanceOfACtion0", SENSOR_DATA_TYPE_COUNT + SENSOR_DATA_TYPE_DISTANCE_ACTION_0), false);
			clbDataComponents->Items->Add(gcnew DataComponent("DistanceOfACtion1", SENSOR_DATA_TYPE_COUNT + SENSOR_DATA_TYPE_DISTANCE_ACTION_1), false);
			clbDataComponents->Items->Add(gcnew DataComponent("DistanceOfACtion2", SENSOR_DATA_TYPE_COUNT + SENSOR_DATA_TYPE_DISTANCE_ACTION_2), false);
			clbDataComponents->Items->Add(gcnew DataComponent("DistanceOfACtion3", SENSOR_DATA_TYPE_COUNT + SENSOR_DATA_TYPE_DISTANCE_ACTION_3), false);
			clbDataComponents->Items->Add(gcnew DataComponent("DistanceOfACtion4", SENSOR_DATA_TYPE_COUNT + SENSOR_DATA_TYPE_DISTANCE_ACTION_4), false);
			clbDataComponents->Items->Add(gcnew DataComponent("DistanceOfACtion5", SENSOR_DATA_TYPE_COUNT + SENSOR_DATA_TYPE_DISTANCE_ACTION_5), false);
			clbDataComponents->Items->Add(gcnew DataComponent("DistanceOfACtion6", SENSOR_DATA_TYPE_COUNT + SENSOR_DATA_TYPE_DISTANCE_ACTION_6), false);
			clbDataComponents->Items->Add(gcnew DataComponent("DistanceOfACtion7", SENSOR_DATA_TYPE_COUNT + SENSOR_DATA_TYPE_DISTANCE_ACTION_7), false);

			clbDataComponents->Items->Add(gcnew DataComponent("RollCompansedQuatW", SENSOR_DATA_TYPE_COUNT + SENSOR_DATA_TYPE_ROLL_COMPANSED_QUATERNION_W), false);	// Sensor'e gore Z ekseninde/asagi dogru Pitch ve Roll etkisinden kurtulmus ivme
			clbDataComponents->Items->Add(gcnew DataComponent("RollCompansedQuatX", SENSOR_DATA_TYPE_COUNT + SENSOR_DATA_TYPE_ROLL_COMPANSED_QUATERNION_X), false);	// Sensor'e gore X ekseninde/ileri dogru pitch etkisinden kurtulmus  ivme
			clbDataComponents->Items->Add(gcnew DataComponent("RollCompansedQuatY", SENSOR_DATA_TYPE_COUNT + SENSOR_DATA_TYPE_ROLL_COMPANSED_QUATERNION_Y), false);	// Sensor'e gore Y ekseninde/yana dogru roll etkisinden kurtulmus ivme
			clbDataComponents->Items->Add(gcnew DataComponent("RollCompansedQuatZ", SENSOR_DATA_TYPE_COUNT + SENSOR_DATA_TYPE_ROLL_COMPANSED_QUATERNION_Z), false);	// Sensor'e gore Y ekseninde/yana dogru roll etkisinden kurtulmus ivme


			clbDataComponents->Items->Add(gcnew DataComponent("FreeSlot", SENSOR_DATA_TYPE_COUNT + SENSOR_DATA_TYPE_FREE_SLOT), false);	// Sensor'e gore Z ekseninde/asagi dogru Pitch ve Roll etkisinden kurtulmus ivme
			clbDataComponents->Items->Add(gcnew DataComponent("SensorRightAcc", SENSOR_DATA_TYPE_COUNT + SENSOR_DATA_TYPE_ROLL_COMPANSED_RIGHT_ACC), false);	// Sensor'e gore X ekseninde/ileri dogru pitch etkisinden kurtulmus  ivme
			clbDataComponents->Items->Add(gcnew DataComponent("SensorDownAcc", SENSOR_DATA_TYPE_COUNT + SENSOR_DATA_TYPE_ROLL_AND_PITCH_COMPANSED_DOWN_ACC), false);	// Sensor'e gore Y ekseninde/yana dogru roll etkisinden kurtulmus ivme


			clbDataComponents->Items->Add(gcnew DataComponent("TimeDiffms", SENSOR_DATA_TYPE_COUNT + SENSOR_DATA_TYPE_TIME_DIFF), false);	// Bir onceki sensor verisi ile fark
			clbDataComponents->Items->Add(gcnew DataComponent("QDistance", SENSOR_DATA_TYPE_COUNT + SENSOR_DATA_TYPE_QUATERNION_DISTANCE_TO_REFERANCE), false);		// Mevcut Quat'in referans quat'a uzakligi


			clbDataComponents->Items->Add(gcnew DataComponent("AccXFiltered",		SENSOR_DATA_TYPE_COUNT + SENSOR_DATA_TYPE_FILTERED_ACC_NED_X), false);
			clbDataComponents->Items->Add(gcnew DataComponent("AccYFiltered",		SENSOR_DATA_TYPE_COUNT + SENSOR_DATA_TYPE_FILTERED_ACC_NED_Y), false);
			clbDataComponents->Items->Add(gcnew DataComponent("AccZFiltered",		SENSOR_DATA_TYPE_COUNT + SENSOR_DATA_TYPE_FILTERED_ACC_NED_Z), false);
			

			clbDataComponents->Items->Add(gcnew DataComponent("MadgYaw",   SENSOR_DATA_TYPE_COUNT + SENSOR_DATA_TYPE_CALCULATED_MADGWICK_YAW), false);
			clbDataComponents->Items->Add(gcnew DataComponent("MadgPitch", SENSOR_DATA_TYPE_COUNT + SENSOR_DATA_TYPE_CALCULATED_MADGWICK_PITCH), false);
			clbDataComponents->Items->Add(gcnew DataComponent("MadgRoll",  SENSOR_DATA_TYPE_COUNT + SENSOR_DATA_TYPE_CALCULATED_MADGWICK_ROLL), false);

			clbDataComponents->Items->Add(gcnew DataComponent("PosZDiff", SENSOR_DATA_TYPE_COUNT + SENSOR_DATA_TYPE_RELATIVE_POSITION_NED_Z_DIFF), false);
			clbDataComponents->Items->Add(gcnew DataComponent("PosZFiltered", SENSOR_DATA_TYPE_COUNT + SENSOR_DATA_TYPE_RELATIVE_POSITION_NED_Z_FILTERED), false);


			clbDataComponents->Items->Add(gcnew DataComponent("AbsPosX", SENSOR_DATA_TYPE_COUNT + SENSOR_DATA_TYPE_ABSOLUTE_POSITION_NED_X), false);
			clbDataComponents->Items->Add(gcnew DataComponent("AbsPosY", SENSOR_DATA_TYPE_COUNT + SENSOR_DATA_TYPE_ABSOLUTE_POSITION_NED_Y), false);
			clbDataComponents->Items->Add(gcnew DataComponent("AbsPosZ", SENSOR_DATA_TYPE_COUNT + SENSOR_DATA_TYPE_ABSOLUTE_POSITION_NED_Z), false);

			clbDataComponents->Items->Add(gcnew DataComponent("AbsVelX", SENSOR_DATA_TYPE_COUNT + SENSOR_DATA_TYPE_ABSOLUTE_VELOCITY_NED_X), false);
			clbDataComponents->Items->Add(gcnew DataComponent("AbsVelY", SENSOR_DATA_TYPE_COUNT + SENSOR_DATA_TYPE_ABSOLUTE_VELOCITY_NED_Y), false);
			clbDataComponents->Items->Add(gcnew DataComponent("AbsVelZ", SENSOR_DATA_TYPE_COUNT + SENSOR_DATA_TYPE_ABSOLUTE_VELOCITY_NED_Z), false);

			clbDataComponents->Items->Add(gcnew DataComponent("Euler2Diff", SENSOR_DATA_TYPE_COUNT + SENSOR_DATA_TYPE_TEMP_EULER_2_DIFF), false);
			clbDataComponents->Items->Add(gcnew DataComponent("Euler3Diff", SENSOR_DATA_TYPE_COUNT + SENSOR_DATA_TYPE_TEMP_EULER_3_DIFF), false);
			clbDataComponents->Items->Add(gcnew DataComponent("DownAccDiff", SENSOR_DATA_TYPE_COUNT + SENSOR_DATA_TYPE_TEMP_ACC_DOWN_DIFF), false);

			clbDataComponents->Items->Add(gcnew DataComponent("DownAccDiffCrossing", SENSOR_DATA_TYPE_COUNT + SENSOR_DATA_TYPE_TEMP_ACC_DOWN_DIFF_CROSSING), false);
			clbDataComponents->Items->Add(gcnew DataComponent("RMSAccFilteredDown", SENSOR_DATA_TYPE_COUNT + SENSOR_DATA_FILTERED_NED_ACC_DOWN_RMS), false);


			

			
			for (int i = 0; i < clbDataComponents->Items->Count;  i++) {

				Object ^obj = (Object ^)clbDataComponents->Items[i];
				String ^item_name = clbDataComponents->GetItemText(obj);
				bool is_checked = pUISettingsSaver->DataSourceSelected(item_name);
				clbDataComponents->SetItemCheckState(i, is_checked?System::Windows::Forms::CheckState::Checked: System::Windows::Forms::CheckState::Unchecked);

			}


			acc_offset = new float[3];
			acc_offset[0] = 0;
			acc_offset[1] = 0;
			acc_offset[2] = 0;

			pKalmanEstimators = gcnew array<SignalProcessing::KalmanEstimator ^>(SENSOR_DATA_TYPE_COUNT + CUSTOM_SENSOR_DATA_TYPE_COUNT);
			for (int i = 0; i < pKalmanEstimators->Length; i++) {
				pKalmanEstimators[i] = gcnew SignalProcessing::KalmanEstimator();
			}
			pActions = gcnew Dictionary<unsigned int, ActionDefinition ^>();
			pOpChains = gcnew List<SignalProcessing::OpChain ^>();

			

			pKalmanEstimators[4]->SetParams(0.5, 0.5, 0.5);

			pKalmanEstimators[SENSOR_DATA_TYPE_COUNT + SENSOR_DATA_TYPE_FILTERED_ACC_NED_X]->SetParams(0.5, 0.5, 0.5);
			pKalmanEstimators[SENSOR_DATA_TYPE_COUNT + SENSOR_DATA_TYPE_FILTERED_ACC_NED_Y]->SetParams(0.5, 0.5, 0.5);
			pKalmanEstimators[SENSOR_DATA_TYPE_COUNT + SENSOR_DATA_TYPE_FILTERED_ACC_NED_Z]->SetParams(0.5, 0.5, 0.5);





			PreviousSensorDataArray = gcnew array<float>(SENSOR_DATA_TYPE_COUNT + CUSTOM_SENSOR_DATA_TYPE_COUNT);
			ActionColors = gcnew List<System::Drawing::Color>();
			pActionBuffer = gcnew ActionBuffer(120);
			pActionArrayBuffer = gcnew ActionArrayBuffer(ACTION_BUFFER_ARRAY_LENGTH);
			pFilteredActionBuffer = gcnew ActionBuffer(120);
			Patterns = gcnew List<array<ViewSerialEvent> ^>();
			PatternSensorDataIndex = gcnew List<int>();
			PatternPlotters = gcnew List<BNOControl::Plotter ^>();
			pRelativePositon = gcnew RelativeMovement();
			pRelativeVelocity = gcnew RelativeMovement();
			SamplingInProgress = false;
			pFileStream = nullptr;
			pTextWriter = nullptr;

			pRandom = gcnew System::Random();
			ElapsedTimeMsSinceBeginning = 0;

			LastSumRoot = 0;
			CurrentSumRoot = 0;
			SerialReadBuffer = gcnew array<Byte>(4096);
			LastSamples = gcnew array<float>(8);
			serialPortBNO055 = gcnew IO::Ports::SerialPort("COM3", 250000, IO::Ports::Parity::None, 8, IO::Ports::StopBits::One);
			serialPortBNO055->ReadTimeout = 4000;
			serialPortBNO055->WriteTimeout = 4000;
			
			try{
			serialPortBNO055->Open();
			serialPortBNO055->DiscardInBuffer();
			serialPortBNO055->DiscardOutBuffer();
			}
			catch (Exception ^e) {

			}

			p_acc_plotter = gcnew BNOControl::Plotter();
			
			p_acc_plotter->SetFormTitle("İvme");
			p_acc_plotter->ResetChart();
			
			p_acc_plotter->AddNewSeries("AccX");
			p_acc_plotter->AddNewSeries("AccY");
			p_acc_plotter->AddNewSeries("AccZ");

			p_acc_plotter->AddNewSeries("AccSumOfSquares");
			p_acc_plotter->AddNewSeries("Diff");
			
			p_acc_plotter->Show();





			p_position_plotter = gcnew BNOControl::Plotter();

			p_position_plotter->SetFormTitle("Position");
			p_position_plotter->ResetChart();
			p_position_plotter->AddNewSeries("N");
			p_position_plotter->AddNewSeries("E");
			p_position_plotter->AddNewSeries("D");

			p_position_plotter->Show();

			p_velocity_plotter = gcnew BNOControl::Plotter();

			p_velocity_plotter->SetFormTitle("Velocity");
			p_velocity_plotter->ResetChart();
			p_velocity_plotter->AddNewSeries("N");
			p_velocity_plotter->AddNewSeries("E");
			p_velocity_plotter->AddNewSeries("D");

			p_velocity_plotter->Show();





			p_quat_plotter = gcnew BNOControl::Plotter();

			p_quat_plotter->SetFormTitle("Quaternion");
			p_quat_plotter->ResetChart();

			p_quat_plotter->AddNewSeries("W");
			p_quat_plotter->AddNewSeries("X");
			p_quat_plotter->AddNewSeries("Y");
			p_quat_plotter->AddNewSeries("Z");
			p_quat_plotter->AddNewSeries("AccSumOfSquares");
			p_quat_plotter->AddNewSeries("Diff");

			p_quat_plotter->Show();

			p_mag_plotter = gcnew BNOControl::Plotter();

			p_mag_plotter->SetFormTitle("Magnetic");
			p_mag_plotter->ResetChart();
			/*
			p_mag_plotter->GetChart()->ChartAreas[0]->AxisX->Minimum = -50;
			p_mag_plotter->GetChart()->ChartAreas[0]->AxisX->Maximum = 50;
			p_mag_plotter->GetChart()->ChartAreas[0]->AxisY->Minimum = -50;
			p_mag_plotter->GetChart()->ChartAreas[0]->AxisY->Maximum = 50;
			*/
			p_mag_plotter->AddNewSeries("X");
			p_mag_plotter->AddNewSeries("Y");
			p_mag_plotter->AddNewSeries("Z");

			p_mag_plotter->Show();

			pRelativePositon->Reset();
			pRelativeVelocity->Reset();

			ActionColors->Add(System::Drawing::Color::Crimson);
			ActionColors->Add(System::Drawing::Color::Green);
			ActionColors->Add(System::Drawing::Color::Blue);
			ActionColors->Add(System::Drawing::Color::Aqua);
			ActionColors->Add(System::Drawing::Color::Yellow);
			ActionColors->Add(System::Drawing::Color::Chocolate);

		}
	private: System::Windows::Forms::Button^  bInitImu;
	private: System::Windows::Forms::TextBox^  tbConsole;
	public:
	private:
		UISettingsSaver ^pUISettingsSaver;
		float RemainingTimeMsToNextAction = 0;
		
		
		int ElapsedTimeMsSinceBeginning;
		boolean InitializeVJOY()
		{
			int id = 1;
			UInt32 DllVer, DrvVer;
			MaxAxisVal = 0;
			pJoystick = gcnew vJoy();
			iReport = gcnew vJoy::JoystickState();
			VjdStat status = pJoystick->GetVJDStatus(1);

			bool AxisX = pJoystick->GetVJDAxisExist(id, HID_USAGES::HID_USAGE_X);
			bool AxisY = pJoystick->GetVJDAxisExist(id, HID_USAGES::HID_USAGE_Y);
			bool AxisZ = pJoystick->GetVJDAxisExist(id, HID_USAGES::HID_USAGE_Z);
			bool AxisRX = pJoystick->GetVJDAxisExist(id, HID_USAGES::HID_USAGE_RX);
			bool AxisRZ = pJoystick->GetVJDAxisExist(id, HID_USAGES::HID_USAGE_RZ);

			
			bool match = pJoystick->DriverMatch(DllVer, DrvVer);


			if ((status == VjdStat::VJD_STAT_OWN) || ((status == VjdStat::VJD_STAT_FREE) && (!pJoystick->AcquireVJD(id))))
			{
				
				return false;
			}
			else {

				pJoystick->GetVJDAxisMax(id, HID_USAGES::HID_USAGE_X, MaxAxisVal);
				pJoystick->GetVJDAxisMax(id, HID_USAGES::HID_USAGE_Y, MaxAxisVal);


				bool res;
				pJoystick->ResetVJD(id);
			}

			return true;
		}
		E_ACTION_TYPES	GetRandomAction()
		{

			return (E_ACTION_TYPES)pRandom->Next(E_ACTION_ZERO + 1, E_ACTION_LAST);
		}

	public:
		float Distance(int x1, int x2) {
			float abs_diff = Math::Abs(x2 - x1);
			if (abs_diff > 180) {

				return 360 - abs_diff;
			}
			else {
				return abs_diff;
			}

		}

	protected:
		/// <summary>
		/// Clean up any resources being used.
		/// </summary>
		~MainForm()
		{
			if (components)
			{
				delete components;
			}
		}
private: System::ComponentModel::IContainer^  components;
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
			this->bInitImu = (gcnew System::Windows::Forms::Button());
			this->tbConsole = (gcnew System::Windows::Forms::TextBox());
			this->groupBox1 = (gcnew System::Windows::Forms::GroupBox());
			this->label3 = (gcnew System::Windows::Forms::Label());
			this->label2 = (gcnew System::Windows::Forms::Label());
			this->label1 = (gcnew System::Windows::Forms::Label());
			this->tbAccZ = (gcnew System::Windows::Forms::TextBox());
			this->tbAccY = (gcnew System::Windows::Forms::TextBox());
			this->tbAccX = (gcnew System::Windows::Forms::TextBox());
			this->groupBox2 = (gcnew System::Windows::Forms::GroupBox());
			this->label4 = (gcnew System::Windows::Forms::Label());
			this->label5 = (gcnew System::Windows::Forms::Label());
			this->label6 = (gcnew System::Windows::Forms::Label());
			this->orientZ = (gcnew System::Windows::Forms::TextBox());
			this->orientY = (gcnew System::Windows::Forms::TextBox());
			this->orientX = (gcnew System::Windows::Forms::TextBox());
			this->groupBox3 = (gcnew System::Windows::Forms::GroupBox());
			this->label7 = (gcnew System::Windows::Forms::Label());
			this->label8 = (gcnew System::Windows::Forms::Label());
			this->label9 = (gcnew System::Windows::Forms::Label());
			this->tbMagneticZ = (gcnew System::Windows::Forms::TextBox());
			this->tbMagneticY = (gcnew System::Windows::Forms::TextBox());
			this->tbMagneticX = (gcnew System::Windows::Forms::TextBox());
			this->groupBox4 = (gcnew System::Windows::Forms::GroupBox());
			this->label10 = (gcnew System::Windows::Forms::Label());
			this->label11 = (gcnew System::Windows::Forms::Label());
			this->label12 = (gcnew System::Windows::Forms::Label());
			this->tbGyroZ = (gcnew System::Windows::Forms::TextBox());
			this->tbGyroY = (gcnew System::Windows::Forms::TextBox());
			this->tbGyroX = (gcnew System::Windows::Forms::TextBox());
			this->tbVersion = (gcnew System::Windows::Forms::TextBox());
			this->tbSensorID = (gcnew System::Windows::Forms::TextBox());
			this->tbType = (gcnew System::Windows::Forms::TextBox());
			this->tbTimestamp = (gcnew System::Windows::Forms::TextBox());
			this->label14 = (gcnew System::Windows::Forms::Label());
			this->label15 = (gcnew System::Windows::Forms::Label());
			this->label16 = (gcnew System::Windows::Forms::Label());
			this->label17 = (gcnew System::Windows::Forms::Label());
			this->tbInvalidMessage = (gcnew System::Windows::Forms::TextBox());
			this->groupBox5 = (gcnew System::Windows::Forms::GroupBox());
			this->label18 = (gcnew System::Windows::Forms::Label());
			this->tbQuatW = (gcnew System::Windows::Forms::TextBox());
			this->label19 = (gcnew System::Windows::Forms::Label());
			this->label20 = (gcnew System::Windows::Forms::Label());
			this->label21 = (gcnew System::Windows::Forms::Label());
			this->tbQuatZ = (gcnew System::Windows::Forms::TextBox());
			this->tbQuatY = (gcnew System::Windows::Forms::TextBox());
			this->tbQuatX = (gcnew System::Windows::Forms::TextBox());
			this->sensorFlowChart = (gcnew System::Windows::Forms::DataVisualization::Charting::Chart());
			this->button1 = (gcnew System::Windows::Forms::Button());
			this->label22 = (gcnew System::Windows::Forms::Label());
			this->lTotalMessageCount = (gcnew System::Windows::Forms::Label());
			this->timerSerialRead = (gcnew System::Windows::Forms::Timer(this->components));
			this->serialPortBNO055 = (gcnew System::IO::Ports::SerialPort(this->components));
			this->lBufferReset = (gcnew System::Windows::Forms::Label());
			this->label24 = (gcnew System::Windows::Forms::Label());
			this->lTotalReceivedBytes = (gcnew System::Windows::Forms::Label());
			this->label25 = (gcnew System::Windows::Forms::Label());
			this->lDifferenceBetweenReceivedBytes = (gcnew System::Windows::Forms::Label());
			this->label27 = (gcnew System::Windows::Forms::Label());
			this->lBufferSizeDecreased = (gcnew System::Windows::Forms::Label());
			this->label26 = (gcnew System::Windows::Forms::Label());
			this->lSerialBufferReadSize = (gcnew System::Windows::Forms::Label());
			this->label28 = (gcnew System::Windows::Forms::Label());
			this->groupBox6 = (gcnew System::Windows::Forms::GroupBox());
			this->lWorkInProgress = (gcnew System::Windows::Forms::Label());
			this->panel1 = (gcnew System::Windows::Forms::Panel());
			this->lCurrentAction = (gcnew System::Windows::Forms::Label());
			this->lNextAction = (gcnew System::Windows::Forms::Label());
			this->label30 = (gcnew System::Windows::Forms::Label());
			this->lRemainingTimeToNextAction = (gcnew System::Windows::Forms::Label());
			this->bLimitlessDataFlow = (gcnew System::Windows::Forms::Button());
			this->label23 = (gcnew System::Windows::Forms::Label());
			this->tTimeToNextAction = (gcnew System::Windows::Forms::Timer(this->components));
			this->bToggleTraining = (gcnew System::Windows::Forms::Button());
			this->bOpenTrainingDirectory = (gcnew System::Windows::Forms::Button());
			this->label29 = (gcnew System::Windows::Forms::Label());
			this->button2 = (gcnew System::Windows::Forms::Button());
			this->groupBox7 = (gcnew System::Windows::Forms::GroupBox());
			this->groupBox8 = (gcnew System::Windows::Forms::GroupBox());
			this->label13 = (gcnew System::Windows::Forms::Label());
			this->label32 = (gcnew System::Windows::Forms::Label());
			this->label33 = (gcnew System::Windows::Forms::Label());
			this->tbAccD = (gcnew System::Windows::Forms::TextBox());
			this->tbAccE = (gcnew System::Windows::Forms::TextBox());
			this->tbAccN = (gcnew System::Windows::Forms::TextBox());
			this->bSetAccOffset = (gcnew System::Windows::Forms::Button());
			this->groupBox9 = (gcnew System::Windows::Forms::GroupBox());
			this->groupBox10 = (gcnew System::Windows::Forms::GroupBox());
			this->clbDataComponents = (gcnew System::Windows::Forms::CheckedListBox());
			this->label31 = (gcnew System::Windows::Forms::Label());
			this->tbDataCount = (gcnew System::Windows::Forms::TextBox());
			this->tbChartMin = (gcnew System::Windows::Forms::TextBox());
			this->tbChartMax = (gcnew System::Windows::Forms::TextBox());
			this->label34 = (gcnew System::Windows::Forms::Label());
			this->label35 = (gcnew System::Windows::Forms::Label());
			this->label36 = (gcnew System::Windows::Forms::Label());
			this->tbSystemCal = (gcnew System::Windows::Forms::TextBox());
			this->groupBox11 = (gcnew System::Windows::Forms::GroupBox());
			this->label40 = (gcnew System::Windows::Forms::Label());
			this->tbMagCal = (gcnew System::Windows::Forms::TextBox());
			this->label39 = (gcnew System::Windows::Forms::Label());
			this->tbAccCal = (gcnew System::Windows::Forms::TextBox());
			this->label38 = (gcnew System::Windows::Forms::Label());
			this->gbGyroCal = (gcnew System::Windows::Forms::TextBox());
			this->label37 = (gcnew System::Windows::Forms::Label());
			this->lTimeDiffMs = (gcnew System::Windows::Forms::Label());
			this->label42 = (gcnew System::Windows::Forms::Label());
			this->tbRefreshVelPos = (gcnew System::Windows::Forms::Button());
			this->groupBox12 = (gcnew System::Windows::Forms::GroupBox());
			this->lQuaternionDistance = (gcnew System::Windows::Forms::Label());
			this->label45 = (gcnew System::Windows::Forms::Label());
			this->tbQDistanceZ = (gcnew System::Windows::Forms::TextBox());
			this->label44 = (gcnew System::Windows::Forms::Label());
			this->tbQDistanceY = (gcnew System::Windows::Forms::TextBox());
			this->label43 = (gcnew System::Windows::Forms::Label());
			this->tbQDistanceX = (gcnew System::Windows::Forms::TextBox());
			this->label41 = (gcnew System::Windows::Forms::Label());
			this->tbQDistanceW = (gcnew System::Windows::Forms::TextBox());
			this->groupBox13 = (gcnew System::Windows::Forms::GroupBox());
			this->pDirectionQuat8 = (gcnew System::Windows::Forms::Panel());
			this->label52 = (gcnew System::Windows::Forms::Label());
			this->pDirectionQuat5 = (gcnew System::Windows::Forms::Panel());
			this->label49 = (gcnew System::Windows::Forms::Label());
			this->pDirectionQuat7 = (gcnew System::Windows::Forms::Panel());
			this->label53 = (gcnew System::Windows::Forms::Label());
			this->pDirectionQuat2 = (gcnew System::Windows::Forms::Panel());
			this->label48 = (gcnew System::Windows::Forms::Label());
			this->pDirectionQuat6 = (gcnew System::Windows::Forms::Panel());
			this->label54 = (gcnew System::Windows::Forms::Label());
			this->pDirectionQuat4 = (gcnew System::Windows::Forms::Panel());
			this->label50 = (gcnew System::Windows::Forms::Label());
			this->pDirectionQuat1 = (gcnew System::Windows::Forms::Panel());
			this->label47 = (gcnew System::Windows::Forms::Label());
			this->pDirectionQuat3 = (gcnew System::Windows::Forms::Panel());
			this->label51 = (gcnew System::Windows::Forms::Label());
			this->pDirectionQuat0 = (gcnew System::Windows::Forms::Panel());
			this->label46 = (gcnew System::Windows::Forms::Label());
			this->button3 = (gcnew System::Windows::Forms::Button());
			this->pJoyButton1 = (gcnew System::Windows::Forms::Panel());
			this->label55 = (gcnew System::Windows::Forms::Label());
			this->label56 = (gcnew System::Windows::Forms::Label());
			this->pJoyButton2 = (gcnew System::Windows::Forms::Panel());
			this->label57 = (gcnew System::Windows::Forms::Label());
			this->pJoyButton3 = (gcnew System::Windows::Forms::Panel());
			this->label58 = (gcnew System::Windows::Forms::Label());
			this->pJoyButton4 = (gcnew System::Windows::Forms::Panel());
			this->label59 = (gcnew System::Windows::Forms::Label());
			this->pJoyButton5 = (gcnew System::Windows::Forms::Panel());
			this->label60 = (gcnew System::Windows::Forms::Label());
			this->pJoyButton6 = (gcnew System::Windows::Forms::Panel());
			this->pJoyButton8 = (gcnew System::Windows::Forms::Panel());
			this->label61 = (gcnew System::Windows::Forms::Label());
			this->pJoyButton7 = (gcnew System::Windows::Forms::Panel());
			this->label62 = (gcnew System::Windows::Forms::Label());
			this->bAlgorithmTest = (gcnew System::Windows::Forms::Button());
			this->bSaveSnapshot = (gcnew System::Windows::Forms::Button());
			this->cbAccOffset = (gcnew System::Windows::Forms::CheckBox());
			this->cbAddDataPointsFlag = (gcnew System::Windows::Forms::CheckBox());
			this->groupBox1->SuspendLayout();
			this->groupBox2->SuspendLayout();
			this->groupBox3->SuspendLayout();
			this->groupBox4->SuspendLayout();
			this->groupBox5->SuspendLayout();
			(cli::safe_cast<System::ComponentModel::ISupportInitialize^>(this->sensorFlowChart))->BeginInit();
			this->groupBox6->SuspendLayout();
			this->panel1->SuspendLayout();
			this->groupBox7->SuspendLayout();
			this->groupBox8->SuspendLayout();
			this->groupBox9->SuspendLayout();
			this->groupBox10->SuspendLayout();
			this->groupBox11->SuspendLayout();
			this->groupBox12->SuspendLayout();
			this->groupBox13->SuspendLayout();
			this->pDirectionQuat8->SuspendLayout();
			this->pDirectionQuat5->SuspendLayout();
			this->pDirectionQuat7->SuspendLayout();
			this->pDirectionQuat2->SuspendLayout();
			this->pDirectionQuat6->SuspendLayout();
			this->pDirectionQuat4->SuspendLayout();
			this->pDirectionQuat1->SuspendLayout();
			this->pDirectionQuat3->SuspendLayout();
			this->pDirectionQuat0->SuspendLayout();
			this->pJoyButton1->SuspendLayout();
			this->pJoyButton2->SuspendLayout();
			this->pJoyButton3->SuspendLayout();
			this->pJoyButton4->SuspendLayout();
			this->pJoyButton5->SuspendLayout();
			this->pJoyButton6->SuspendLayout();
			this->pJoyButton8->SuspendLayout();
			this->pJoyButton7->SuspendLayout();
			this->SuspendLayout();
			// 
			// bInitImu
			// 
			this->bInitImu->Location = System::Drawing::Point(204, 23);
			this->bInitImu->Name = L"bInitImu";
			this->bInitImu->Size = System::Drawing::Size(55, 49);
			this->bInitImu->TabIndex = 0;
			this->bInitImu->Text = L"Init Nav.";
			this->bInitImu->UseVisualStyleBackColor = true;
			this->bInitImu->Click += gcnew System::EventHandler(this, &MainForm::bInitImu_Click);
			// 
			// tbConsole
			// 
			this->tbConsole->Location = System::Drawing::Point(14, 23);
			this->tbConsole->Multiline = true;
			this->tbConsole->Name = L"tbConsole";
			this->tbConsole->Size = System::Drawing::Size(186, 34);
			this->tbConsole->TabIndex = 1;
			// 
			// groupBox1
			// 
			this->groupBox1->Anchor = static_cast<System::Windows::Forms::AnchorStyles>((System::Windows::Forms::AnchorStyles::Bottom | System::Windows::Forms::AnchorStyles::Right));
			this->groupBox1->Controls->Add(this->label3);
			this->groupBox1->Controls->Add(this->label2);
			this->groupBox1->Controls->Add(this->label1);
			this->groupBox1->Controls->Add(this->tbAccZ);
			this->groupBox1->Controls->Add(this->tbAccY);
			this->groupBox1->Controls->Add(this->tbAccX);
			this->groupBox1->Location = System::Drawing::Point(887, 548);
			this->groupBox1->Name = L"groupBox1";
			this->groupBox1->Size = System::Drawing::Size(229, 59);
			this->groupBox1->TabIndex = 2;
			this->groupBox1->TabStop = false;
			this->groupBox1->Text = L"Acceleration(m/s^2)";
			this->groupBox1->Enter += gcnew System::EventHandler(this, &MainForm::groupBox1_Enter_1);
			// 
			// label3
			// 
			this->label3->AutoSize = true;
			this->label3->Location = System::Drawing::Point(170, 14);
			this->label3->Name = L"label3";
			this->label3->Size = System::Drawing::Size(14, 13);
			this->label3->TabIndex = 5;
			this->label3->Text = L"Z";
			// 
			// label2
			// 
			this->label2->AutoSize = true;
			this->label2->Location = System::Drawing::Point(94, 14);
			this->label2->Name = L"label2";
			this->label2->Size = System::Drawing::Size(14, 13);
			this->label2->TabIndex = 4;
			this->label2->Text = L"Y";
			// 
			// label1
			// 
			this->label1->AutoSize = true;
			this->label1->Location = System::Drawing::Point(24, 14);
			this->label1->Name = L"label1";
			this->label1->Size = System::Drawing::Size(14, 13);
			this->label1->TabIndex = 3;
			this->label1->Text = L"X";
			// 
			// tbAccZ
			// 
			this->tbAccZ->Location = System::Drawing::Point(141, 31);
			this->tbAccZ->Name = L"tbAccZ";
			this->tbAccZ->Size = System::Drawing::Size(66, 20);
			this->tbAccZ->TabIndex = 2;
			// 
			// tbAccY
			// 
			this->tbAccY->Location = System::Drawing::Point(73, 31);
			this->tbAccY->Name = L"tbAccY";
			this->tbAccY->Size = System::Drawing::Size(62, 20);
			this->tbAccY->TabIndex = 1;
			// 
			// tbAccX
			// 
			this->tbAccX->Location = System::Drawing::Point(6, 31);
			this->tbAccX->Name = L"tbAccX";
			this->tbAccX->Size = System::Drawing::Size(61, 20);
			this->tbAccX->TabIndex = 0;
			// 
			// groupBox2
			// 
			this->groupBox2->Anchor = static_cast<System::Windows::Forms::AnchorStyles>((System::Windows::Forms::AnchorStyles::Bottom | System::Windows::Forms::AnchorStyles::Right));
			this->groupBox2->Controls->Add(this->label4);
			this->groupBox2->Controls->Add(this->label5);
			this->groupBox2->Controls->Add(this->label6);
			this->groupBox2->Controls->Add(this->orientZ);
			this->groupBox2->Controls->Add(this->orientY);
			this->groupBox2->Controls->Add(this->orientX);
			this->groupBox2->Location = System::Drawing::Point(887, 609);
			this->groupBox2->Name = L"groupBox2";
			this->groupBox2->Size = System::Drawing::Size(229, 60);
			this->groupBox2->TabIndex = 6;
			this->groupBox2->TabStop = false;
			this->groupBox2->Text = L"Euler(degrees)";
			// 
			// label4
			// 
			this->label4->AutoSize = true;
			this->label4->Location = System::Drawing::Point(171, 16);
			this->label4->Name = L"label4";
			this->label4->Size = System::Drawing::Size(14, 13);
			this->label4->TabIndex = 5;
			this->label4->Text = L"Z";
			// 
			// label5
			// 
			this->label5->AutoSize = true;
			this->label5->Location = System::Drawing::Point(94, 16);
			this->label5->Name = L"label5";
			this->label5->Size = System::Drawing::Size(14, 13);
			this->label5->TabIndex = 4;
			this->label5->Text = L"Y";
			// 
			// label6
			// 
			this->label6->AutoSize = true;
			this->label6->Location = System::Drawing::Point(24, 16);
			this->label6->Name = L"label6";
			this->label6->Size = System::Drawing::Size(14, 13);
			this->label6->TabIndex = 3;
			this->label6->Text = L"X";
			// 
			// orientZ
			// 
			this->orientZ->Location = System::Drawing::Point(142, 34);
			this->orientZ->Name = L"orientZ";
			this->orientZ->Size = System::Drawing::Size(66, 20);
			this->orientZ->TabIndex = 2;
			// 
			// orientY
			// 
			this->orientY->Location = System::Drawing::Point(73, 34);
			this->orientY->Name = L"orientY";
			this->orientY->Size = System::Drawing::Size(62, 20);
			this->orientY->TabIndex = 1;
			// 
			// orientX
			// 
			this->orientX->Location = System::Drawing::Point(6, 34);
			this->orientX->Name = L"orientX";
			this->orientX->Size = System::Drawing::Size(61, 20);
			this->orientX->TabIndex = 0;
			// 
			// groupBox3
			// 
			this->groupBox3->Anchor = static_cast<System::Windows::Forms::AnchorStyles>((System::Windows::Forms::AnchorStyles::Bottom | System::Windows::Forms::AnchorStyles::Right));
			this->groupBox3->Controls->Add(this->label7);
			this->groupBox3->Controls->Add(this->label8);
			this->groupBox3->Controls->Add(this->label9);
			this->groupBox3->Controls->Add(this->tbMagneticZ);
			this->groupBox3->Controls->Add(this->tbMagneticY);
			this->groupBox3->Controls->Add(this->tbMagneticX);
			this->groupBox3->Location = System::Drawing::Point(887, 670);
			this->groupBox3->Name = L"groupBox3";
			this->groupBox3->Size = System::Drawing::Size(229, 60);
			this->groupBox3->TabIndex = 7;
			this->groupBox3->TabStop = false;
			this->groupBox3->Text = L"Magnetic(uT)";
			// 
			// label7
			// 
			this->label7->AutoSize = true;
			this->label7->Location = System::Drawing::Point(171, 19);
			this->label7->Name = L"label7";
			this->label7->Size = System::Drawing::Size(14, 13);
			this->label7->TabIndex = 5;
			this->label7->Text = L"Z";
			// 
			// label8
			// 
			this->label8->AutoSize = true;
			this->label8->Location = System::Drawing::Point(93, 19);
			this->label8->Name = L"label8";
			this->label8->Size = System::Drawing::Size(14, 13);
			this->label8->TabIndex = 4;
			this->label8->Text = L"Y";
			// 
			// label9
			// 
			this->label9->AutoSize = true;
			this->label9->Location = System::Drawing::Point(24, 19);
			this->label9->Name = L"label9";
			this->label9->Size = System::Drawing::Size(14, 13);
			this->label9->TabIndex = 3;
			this->label9->Text = L"X";
			// 
			// tbMagneticZ
			// 
			this->tbMagneticZ->Location = System::Drawing::Point(142, 36);
			this->tbMagneticZ->Name = L"tbMagneticZ";
			this->tbMagneticZ->Size = System::Drawing::Size(66, 20);
			this->tbMagneticZ->TabIndex = 2;
			// 
			// tbMagneticY
			// 
			this->tbMagneticY->Location = System::Drawing::Point(72, 36);
			this->tbMagneticY->Name = L"tbMagneticY";
			this->tbMagneticY->Size = System::Drawing::Size(62, 20);
			this->tbMagneticY->TabIndex = 1;
			// 
			// tbMagneticX
			// 
			this->tbMagneticX->Location = System::Drawing::Point(6, 36);
			this->tbMagneticX->Name = L"tbMagneticX";
			this->tbMagneticX->Size = System::Drawing::Size(61, 20);
			this->tbMagneticX->TabIndex = 0;
			// 
			// groupBox4
			// 
			this->groupBox4->Anchor = static_cast<System::Windows::Forms::AnchorStyles>((System::Windows::Forms::AnchorStyles::Bottom | System::Windows::Forms::AnchorStyles::Right));
			this->groupBox4->Controls->Add(this->label10);
			this->groupBox4->Controls->Add(this->label11);
			this->groupBox4->Controls->Add(this->label12);
			this->groupBox4->Controls->Add(this->tbGyroZ);
			this->groupBox4->Controls->Add(this->tbGyroY);
			this->groupBox4->Controls->Add(this->tbGyroX);
			this->groupBox4->Location = System::Drawing::Point(1126, 686);
			this->groupBox4->Name = L"groupBox4";
			this->groupBox4->Size = System::Drawing::Size(229, 59);
			this->groupBox4->TabIndex = 6;
			this->groupBox4->TabStop = false;
			this->groupBox4->Text = L"Gyro(rad/s)";
			// 
			// label10
			// 
			this->label10->AutoSize = true;
			this->label10->Location = System::Drawing::Point(173, 14);
			this->label10->Name = L"label10";
			this->label10->Size = System::Drawing::Size(14, 13);
			this->label10->TabIndex = 5;
			this->label10->Text = L"Z";
			// 
			// label11
			// 
			this->label11->AutoSize = true;
			this->label11->Location = System::Drawing::Point(93, 14);
			this->label11->Name = L"label11";
			this->label11->Size = System::Drawing::Size(14, 13);
			this->label11->TabIndex = 4;
			this->label11->Text = L"Y";
			// 
			// label12
			// 
			this->label12->AutoSize = true;
			this->label12->Location = System::Drawing::Point(24, 14);
			this->label12->Name = L"label12";
			this->label12->Size = System::Drawing::Size(14, 13);
			this->label12->TabIndex = 3;
			this->label12->Text = L"X";
			// 
			// tbGyroZ
			// 
			this->tbGyroZ->Location = System::Drawing::Point(144, 31);
			this->tbGyroZ->Name = L"tbGyroZ";
			this->tbGyroZ->Size = System::Drawing::Size(66, 20);
			this->tbGyroZ->TabIndex = 2;
			// 
			// tbGyroY
			// 
			this->tbGyroY->Location = System::Drawing::Point(72, 31);
			this->tbGyroY->Name = L"tbGyroY";
			this->tbGyroY->Size = System::Drawing::Size(62, 20);
			this->tbGyroY->TabIndex = 1;
			// 
			// tbGyroX
			// 
			this->tbGyroX->Location = System::Drawing::Point(6, 31);
			this->tbGyroX->Name = L"tbGyroX";
			this->tbGyroX->Size = System::Drawing::Size(61, 20);
			this->tbGyroX->TabIndex = 0;
			// 
			// tbVersion
			// 
			this->tbVersion->Location = System::Drawing::Point(94, 75);
			this->tbVersion->Name = L"tbVersion";
			this->tbVersion->Size = System::Drawing::Size(49, 20);
			this->tbVersion->TabIndex = 8;
			// 
			// tbSensorID
			// 
			this->tbSensorID->Location = System::Drawing::Point(94, 101);
			this->tbSensorID->Name = L"tbSensorID";
			this->tbSensorID->Size = System::Drawing::Size(49, 20);
			this->tbSensorID->TabIndex = 9;
			// 
			// tbType
			// 
			this->tbType->Location = System::Drawing::Point(94, 127);
			this->tbType->Name = L"tbType";
			this->tbType->Size = System::Drawing::Size(49, 20);
			this->tbType->TabIndex = 10;
			// 
			// tbTimestamp
			// 
			this->tbTimestamp->Location = System::Drawing::Point(94, 153);
			this->tbTimestamp->Name = L"tbTimestamp";
			this->tbTimestamp->Size = System::Drawing::Size(100, 20);
			this->tbTimestamp->TabIndex = 11;
			// 
			// label14
			// 
			this->label14->AutoSize = true;
			this->label14->Location = System::Drawing::Point(34, 78);
			this->label14->Name = L"label14";
			this->label14->Size = System::Drawing::Size(42, 13);
			this->label14->TabIndex = 12;
			this->label14->Text = L"Version";
			this->label14->TextAlign = System::Drawing::ContentAlignment::TopRight;
			// 
			// label15
			// 
			this->label15->AutoSize = true;
			this->label15->Location = System::Drawing::Point(22, 104);
			this->label15->Name = L"label15";
			this->label15->Size = System::Drawing::Size(54, 13);
			this->label15->TabIndex = 13;
			this->label15->Text = L"Sensor ID";
			this->label15->TextAlign = System::Drawing::ContentAlignment::TopRight;
			// 
			// label16
			// 
			this->label16->AutoSize = true;
			this->label16->Location = System::Drawing::Point(45, 130);
			this->label16->Name = L"label16";
			this->label16->Size = System::Drawing::Size(31, 13);
			this->label16->TabIndex = 14;
			this->label16->Text = L"Type";
			this->label16->TextAlign = System::Drawing::ContentAlignment::TopRight;
			// 
			// label17
			// 
			this->label17->AutoSize = true;
			this->label17->Location = System::Drawing::Point(18, 156);
			this->label17->Name = L"label17";
			this->label17->Size = System::Drawing::Size(58, 13);
			this->label17->TabIndex = 15;
			this->label17->Text = L"Timestamp";
			this->label17->TextAlign = System::Drawing::ContentAlignment::TopRight;
			// 
			// tbInvalidMessage
			// 
			this->tbInvalidMessage->Location = System::Drawing::Point(94, 179);
			this->tbInvalidMessage->Name = L"tbInvalidMessage";
			this->tbInvalidMessage->Size = System::Drawing::Size(100, 20);
			this->tbInvalidMessage->TabIndex = 16;
			this->tbInvalidMessage->Text = L"0";
			// 
			// groupBox5
			// 
			this->groupBox5->Anchor = static_cast<System::Windows::Forms::AnchorStyles>((System::Windows::Forms::AnchorStyles::Bottom | System::Windows::Forms::AnchorStyles::Right));
			this->groupBox5->Controls->Add(this->label18);
			this->groupBox5->Controls->Add(this->tbQuatW);
			this->groupBox5->Controls->Add(this->label19);
			this->groupBox5->Controls->Add(this->label20);
			this->groupBox5->Controls->Add(this->label21);
			this->groupBox5->Controls->Add(this->tbQuatZ);
			this->groupBox5->Controls->Add(this->tbQuatY);
			this->groupBox5->Controls->Add(this->tbQuatX);
			this->groupBox5->Location = System::Drawing::Point(1122, 545);
			this->groupBox5->Name = L"groupBox5";
			this->groupBox5->Size = System::Drawing::Size(236, 73);
			this->groupBox5->TabIndex = 8;
			this->groupBox5->TabStop = false;
			this->groupBox5->Text = L"Quaternion";
			// 
			// label18
			// 
			this->label18->AutoSize = true;
			this->label18->Location = System::Drawing::Point(14, 20);
			this->label18->Name = L"label18";
			this->label18->Size = System::Drawing::Size(18, 13);
			this->label18->TabIndex = 7;
			this->label18->Text = L"W";
			// 
			// tbQuatW
			// 
			this->tbQuatW->Location = System::Drawing::Point(11, 42);
			this->tbQuatW->Name = L"tbQuatW";
			this->tbQuatW->Size = System::Drawing::Size(39, 20);
			this->tbQuatW->TabIndex = 6;
			// 
			// label19
			// 
			this->label19->AutoSize = true;
			this->label19->Location = System::Drawing::Point(177, 20);
			this->label19->Name = L"label19";
			this->label19->Size = System::Drawing::Size(14, 13);
			this->label19->TabIndex = 5;
			this->label19->Text = L"Z";
			// 
			// label20
			// 
			this->label20->AutoSize = true;
			this->label20->Location = System::Drawing::Point(123, 20);
			this->label20->Name = L"label20";
			this->label20->Size = System::Drawing::Size(14, 13);
			this->label20->TabIndex = 4;
			this->label20->Text = L"Y";
			// 
			// label21
			// 
			this->label21->AutoSize = true;
			this->label21->Location = System::Drawing::Point(73, 20);
			this->label21->Name = L"label21";
			this->label21->Size = System::Drawing::Size(14, 13);
			this->label21->TabIndex = 3;
			this->label21->Text = L"X";
			// 
			// tbQuatZ
			// 
			this->tbQuatZ->Location = System::Drawing::Point(165, 42);
			this->tbQuatZ->Name = L"tbQuatZ";
			this->tbQuatZ->Size = System::Drawing::Size(46, 20);
			this->tbQuatZ->TabIndex = 2;
			// 
			// tbQuatY
			// 
			this->tbQuatY->Location = System::Drawing::Point(112, 42);
			this->tbQuatY->Name = L"tbQuatY";
			this->tbQuatY->Size = System::Drawing::Size(41, 20);
			this->tbQuatY->TabIndex = 1;
			// 
			// tbQuatX
			// 
			this->tbQuatX->Location = System::Drawing::Point(61, 42);
			this->tbQuatX->Name = L"tbQuatX";
			this->tbQuatX->Size = System::Drawing::Size(39, 20);
			this->tbQuatX->TabIndex = 0;
			// 
			// sensorFlowChart
			// 
			this->sensorFlowChart->Anchor = static_cast<System::Windows::Forms::AnchorStyles>((((System::Windows::Forms::AnchorStyles::Top | System::Windows::Forms::AnchorStyles::Bottom)
				| System::Windows::Forms::AnchorStyles::Left)
				| System::Windows::Forms::AnchorStyles::Right));
			chartArea1->AxisY->Maximum = 30;
			chartArea1->AxisY->Minimum = -10;
			chartArea1->Name = L"ChartArea1";
			this->sensorFlowChart->ChartAreas->Add(chartArea1);
			legend1->Name = L"Legend1";
			this->sensorFlowChart->Legends->Add(legend1);
			this->sensorFlowChart->Location = System::Drawing::Point(265, 12);
			this->sensorFlowChart->Name = L"sensorFlowChart";
			this->sensorFlowChart->Size = System::Drawing::Size(1093, 513);
			this->sensorFlowChart->TabIndex = 17;
			this->sensorFlowChart->Text = L"chart1";
			// 
			// button1
			// 
			this->button1->Location = System::Drawing::Point(204, 151);
			this->button1->Name = L"button1";
			this->button1->Size = System::Drawing::Size(55, 48);
			this->button1->TabIndex = 18;
			this->button1->Text = L"Refresh";
			this->button1->UseVisualStyleBackColor = true;
			this->button1->Click += gcnew System::EventHandler(this, &MainForm::button1_Click);
			// 
			// label22
			// 
			this->label22->AutoSize = true;
			this->label22->Location = System::Drawing::Point(142, 213);
			this->label22->Name = L"label22";
			this->label22->Size = System::Drawing::Size(108, 13);
			this->label22->TabIndex = 21;
			this->label22->Text = L"Toplam Doğru Mesaj:";
			// 
			// lTotalMessageCount
			// 
			this->lTotalMessageCount->AutoSize = true;
			this->lTotalMessageCount->Location = System::Drawing::Point(249, 213);
			this->lTotalMessageCount->Name = L"lTotalMessageCount";
			this->lTotalMessageCount->Size = System::Drawing::Size(10, 13);
			this->lTotalMessageCount->TabIndex = 22;
			this->lTotalMessageCount->Text = L"-";
			// 
			// timerSerialRead
			// 
			this->timerSerialRead->Enabled = true;
			this->timerSerialRead->Interval = 10;
			this->timerSerialRead->Tick += gcnew System::EventHandler(this, &MainForm::timerSerialRead_Tick);
			// 
			// serialPortBNO055
			// 
			this->serialPortBNO055->BaudRate = 115200;
			this->serialPortBNO055->PortName = L"COM3";
			// 
			// lBufferReset
			// 
			this->lBufferReset->AutoSize = true;
			this->lBufferReset->Location = System::Drawing::Point(109, 242);
			this->lBufferReset->Name = L"lBufferReset";
			this->lBufferReset->Size = System::Drawing::Size(10, 13);
			this->lBufferReset->TabIndex = 24;
			this->lBufferReset->Text = L"-";
			// 
			// label24
			// 
			this->label24->AutoSize = true;
			this->label24->Location = System::Drawing::Point(11, 242);
			this->label24->Name = L"label24";
			this->label24->Size = System::Drawing::Size(69, 13);
			this->label24->TabIndex = 23;
			this->label24->Text = L"Buffer Reset:";
			// 
			// lTotalReceivedBytes
			// 
			this->lTotalReceivedBytes->AutoSize = true;
			this->lTotalReceivedBytes->Location = System::Drawing::Point(249, 227);
			this->lTotalReceivedBytes->Name = L"lTotalReceivedBytes";
			this->lTotalReceivedBytes->Size = System::Drawing::Size(10, 13);
			this->lTotalReceivedBytes->TabIndex = 26;
			this->lTotalReceivedBytes->Text = L"-";
			// 
			// label25
			// 
			this->label25->AutoSize = true;
			this->label25->Location = System::Drawing::Point(145, 226);
			this->label25->Name = L"label25";
			this->label25->Size = System::Drawing::Size(66, 13);
			this->label25->TabIndex = 25;
			this->label25->Text = L"Alınan Byte: ";
			// 
			// lDifferenceBetweenReceivedBytes
			// 
			this->lDifferenceBetweenReceivedBytes->AutoSize = true;
			this->lDifferenceBetweenReceivedBytes->Location = System::Drawing::Point(249, 242);
			this->lDifferenceBetweenReceivedBytes->Name = L"lDifferenceBetweenReceivedBytes";
			this->lDifferenceBetweenReceivedBytes->Size = System::Drawing::Size(10, 13);
			this->lDifferenceBetweenReceivedBytes->TabIndex = 28;
			this->lDifferenceBetweenReceivedBytes->Text = L"-";
			// 
			// label27
			// 
			this->label27->AutoSize = true;
			this->label27->Location = System::Drawing::Point(147, 242);
			this->label27->Name = L"label27";
			this->label27->Size = System::Drawing::Size(73, 13);
			this->label27->TabIndex = 27;
			this->label27->Text = L"Aradaki  Fark:";
			// 
			// lBufferSizeDecreased
			// 
			this->lBufferSizeDecreased->AutoSize = true;
			this->lBufferSizeDecreased->Location = System::Drawing::Point(110, 226);
			this->lBufferSizeDecreased->Name = L"lBufferSizeDecreased";
			this->lBufferSizeDecreased->Size = System::Drawing::Size(10, 13);
			this->lBufferSizeDecreased->TabIndex = 30;
			this->lBufferSizeDecreased->Text = L"-";
			// 
			// label26
			// 
			this->label26->AutoSize = true;
			this->label26->Location = System::Drawing::Point(9, 227);
			this->label26->Name = L"label26";
			this->label26->Size = System::Drawing::Size(68, 13);
			this->label26->TabIndex = 29;
			this->label26->Text = L"Buffer azaldı:";
			// 
			// lSerialBufferReadSize
			// 
			this->lSerialBufferReadSize->AutoSize = true;
			this->lSerialBufferReadSize->Location = System::Drawing::Point(110, 213);
			this->lSerialBufferReadSize->Name = L"lSerialBufferReadSize";
			this->lSerialBufferReadSize->Size = System::Drawing::Size(10, 13);
			this->lSerialBufferReadSize->TabIndex = 32;
			this->lSerialBufferReadSize->Text = L"-";
			// 
			// label28
			// 
			this->label28->AutoSize = true;
			this->label28->Location = System::Drawing::Point(9, 213);
			this->label28->Name = L"label28";
			this->label28->Size = System::Drawing::Size(97, 13);
			this->label28->TabIndex = 31;
			this->label28->Text = L"Serial Buffer Boyut:";
			// 
			// groupBox6
			// 
			this->groupBox6->Controls->Add(this->lWorkInProgress);
			this->groupBox6->Controls->Add(this->panel1);
			this->groupBox6->Controls->Add(this->lRemainingTimeToNextAction);
			this->groupBox6->Controls->Add(this->bLimitlessDataFlow);
			this->groupBox6->Controls->Add(this->label23);
			this->groupBox6->Location = System::Drawing::Point(13, 100);
			this->groupBox6->Name = L"groupBox6";
			this->groupBox6->Size = System::Drawing::Size(187, 104);
			this->groupBox6->TabIndex = 33;
			this->groupBox6->TabStop = false;
			this->groupBox6->Text = L"Eğitim ";
			// 
			// lWorkInProgress
			// 
			this->lWorkInProgress->AutoSize = true;
			this->lWorkInProgress->Location = System::Drawing::Point(54, 60);
			this->lWorkInProgress->Name = L"lWorkInProgress";
			this->lWorkInProgress->Size = System::Drawing::Size(10, 13);
			this->lWorkInProgress->TabIndex = 5;
			this->lWorkInProgress->Text = L"-";
			this->lWorkInProgress->TextAlign = System::Drawing::ContentAlignment::MiddleCenter;
			// 
			// panel1
			// 
			this->panel1->BackColor = System::Drawing::Color::Crimson;
			this->panel1->Controls->Add(this->lCurrentAction);
			this->panel1->Controls->Add(this->lNextAction);
			this->panel1->Controls->Add(this->label30);
			this->panel1->Location = System::Drawing::Point(113, 19);
			this->panel1->Name = L"panel1";
			this->panel1->Size = System::Drawing::Size(62, 35);
			this->panel1->TabIndex = 4;
			// 
			// lCurrentAction
			// 
			this->lCurrentAction->AutoSize = true;
			this->lCurrentAction->Font = (gcnew System::Drawing::Font(L"Microsoft Sans Serif", 12.25F, System::Drawing::FontStyle::Bold, System::Drawing::GraphicsUnit::Point,
				static_cast<System::Byte>(162)));
			this->lCurrentAction->ForeColor = System::Drawing::SystemColors::ButtonHighlight;
			this->lCurrentAction->Location = System::Drawing::Point(12, 8);
			this->lCurrentAction->Name = L"lCurrentAction";
			this->lCurrentAction->Size = System::Drawing::Size(19, 20);
			this->lCurrentAction->TabIndex = 0;
			this->lCurrentAction->Text = L"1";
			// 
			// lNextAction
			// 
			this->lNextAction->AutoSize = true;
			this->lNextAction->Font = (gcnew System::Drawing::Font(L"Microsoft Sans Serif", 12.25F, System::Drawing::FontStyle::Bold, System::Drawing::GraphicsUnit::Point,
				static_cast<System::Byte>(162)));
			this->lNextAction->ForeColor = System::Drawing::SystemColors::AppWorkspace;
			this->lNextAction->Location = System::Drawing::Point(37, 8);
			this->lNextAction->Name = L"lNextAction";
			this->lNextAction->Size = System::Drawing::Size(19, 20);
			this->lNextAction->TabIndex = 3;
			this->lNextAction->Text = L"1";
			// 
			// label30
			// 
			this->label30->AutoSize = true;
			this->label30->Font = (gcnew System::Drawing::Font(L"Microsoft Sans Serif", 12.25F, System::Drawing::FontStyle::Regular, System::Drawing::GraphicsUnit::Point,
				static_cast<System::Byte>(162)));
			this->label30->ForeColor = System::Drawing::SystemColors::ButtonHighlight;
			this->label30->Location = System::Drawing::Point(26, 7);
			this->label30->Name = L"label30";
			this->label30->Size = System::Drawing::Size(15, 20);
			this->label30->TabIndex = 2;
			this->label30->Text = L"-";
			// 
			// lRemainingTimeToNextAction
			// 
			this->lRemainingTimeToNextAction->AutoSize = true;
			this->lRemainingTimeToNextAction->Location = System::Drawing::Point(54, 33);
			this->lRemainingTimeToNextAction->Name = L"lRemainingTimeToNextAction";
			this->lRemainingTimeToNextAction->Size = System::Drawing::Size(10, 13);
			this->lRemainingTimeToNextAction->TabIndex = 1;
			this->lRemainingTimeToNextAction->Text = L"-";
			// 
			// bLimitlessDataFlow
			// 
			this->bLimitlessDataFlow->Location = System::Drawing::Point(106, 60);
			this->bLimitlessDataFlow->Name = L"bLimitlessDataFlow";
			this->bLimitlessDataFlow->Size = System::Drawing::Size(75, 23);
			this->bLimitlessDataFlow->TabIndex = 37;
			this->bLimitlessDataFlow->Text = L"Sürekli Veri Kaydı";
			this->bLimitlessDataFlow->UseVisualStyleBackColor = true;
			this->bLimitlessDataFlow->Click += gcnew System::EventHandler(this, &MainForm::bLimitlessDataFlow_Click);
			// 
			// label23
			// 
			this->label23->AutoSize = true;
			this->label23->Location = System::Drawing::Point(7, 33);
			this->label23->Name = L"label23";
			this->label23->Size = System::Drawing::Size(32, 13);
			this->label23->TabIndex = 0;
			this->label23->Text = L"Süre:";
			// 
			// tTimeToNextAction
			// 
			this->tTimeToNextAction->Tick += gcnew System::EventHandler(this, &MainForm::tTimeToNextAction_Tick_1);
			// 
			// bToggleTraining
			// 
			this->bToggleTraining->Location = System::Drawing::Point(6, 64);
			this->bToggleTraining->Name = L"bToggleTraining";
			this->bToggleTraining->Size = System::Drawing::Size(85, 30);
			this->bToggleTraining->TabIndex = 34;
			this->bToggleTraining->Text = L"Eğitimi Başlat";
			this->bToggleTraining->UseVisualStyleBackColor = true;
			this->bToggleTraining->Click += gcnew System::EventHandler(this, &MainForm::bToggleTraining_Click);
			// 
			// bOpenTrainingDirectory
			// 
			this->bOpenTrainingDirectory->Location = System::Drawing::Point(97, 66);
			this->bOpenTrainingDirectory->Name = L"bOpenTrainingDirectory";
			this->bOpenTrainingDirectory->Size = System::Drawing::Size(103, 29);
			this->bOpenTrainingDirectory->TabIndex = 35;
			this->bOpenTrainingDirectory->Text = L"Eğitim Dizinini Aç";
			this->bOpenTrainingDirectory->UseVisualStyleBackColor = true;
			this->bOpenTrainingDirectory->Click += gcnew System::EventHandler(this, &MainForm::bOpenTrainingDirectory_Click);
			// 
			// label29
			// 
			this->label29->AutoSize = true;
			this->label29->Location = System::Drawing::Point(1, 182);
			this->label29->Name = L"label29";
			this->label29->Size = System::Drawing::Size(79, 13);
			this->label29->TabIndex = 36;
			this->label29->Text = L"Tanımsız Mesaj";
			this->label29->TextAlign = System::Drawing::ContentAlignment::TopRight;
			// 
			// button2
			// 
			this->button2->Location = System::Drawing::Point(106, 11);
			this->button2->Name = L"button2";
			this->button2->Size = System::Drawing::Size(75, 23);
			this->button2->TabIndex = 38;
			this->button2->Text = L"Desen yükle";
			this->button2->UseVisualStyleBackColor = true;
			this->button2->Click += gcnew System::EventHandler(this, &MainForm::button2_Click);
			// 
			// groupBox7
			// 
			this->groupBox7->Controls->Add(this->button2);
			this->groupBox7->Location = System::Drawing::Point(13, 15);
			this->groupBox7->Name = L"groupBox7";
			this->groupBox7->Size = System::Drawing::Size(187, 43);
			this->groupBox7->TabIndex = 39;
			this->groupBox7->TabStop = false;
			this->groupBox7->Text = L"Yüklü desenler";
			// 
			// groupBox8
			// 
			this->groupBox8->Anchor = static_cast<System::Windows::Forms::AnchorStyles>((System::Windows::Forms::AnchorStyles::Bottom | System::Windows::Forms::AnchorStyles::Right));
			this->groupBox8->Controls->Add(this->label13);
			this->groupBox8->Controls->Add(this->label32);
			this->groupBox8->Controls->Add(this->label33);
			this->groupBox8->Controls->Add(this->tbAccD);
			this->groupBox8->Controls->Add(this->tbAccE);
			this->groupBox8->Controls->Add(this->tbAccN);
			this->groupBox8->Location = System::Drawing::Point(1126, 624);
			this->groupBox8->Name = L"groupBox8";
			this->groupBox8->Size = System::Drawing::Size(232, 56);
			this->groupBox8->TabIndex = 6;
			this->groupBox8->TabStop = false;
			this->groupBox8->Text = L"NEDAcceleration(m/s^2)";
			// 
			// label13
			// 
			this->label13->AutoSize = true;
			this->label13->Location = System::Drawing::Point(170, 14);
			this->label13->Name = L"label13";
			this->label13->Size = System::Drawing::Size(15, 13);
			this->label13->TabIndex = 5;
			this->label13->Text = L"D";
			// 
			// label32
			// 
			this->label32->AutoSize = true;
			this->label32->Location = System::Drawing::Point(94, 14);
			this->label32->Name = L"label32";
			this->label32->Size = System::Drawing::Size(14, 13);
			this->label32->TabIndex = 4;
			this->label32->Text = L"E";
			// 
			// label33
			// 
			this->label33->AutoSize = true;
			this->label33->Location = System::Drawing::Point(24, 14);
			this->label33->Name = L"label33";
			this->label33->Size = System::Drawing::Size(15, 13);
			this->label33->TabIndex = 3;
			this->label33->Text = L"N";
			// 
			// tbAccD
			// 
			this->tbAccD->Location = System::Drawing::Point(141, 30);
			this->tbAccD->Name = L"tbAccD";
			this->tbAccD->Size = System::Drawing::Size(66, 20);
			this->tbAccD->TabIndex = 2;
			// 
			// tbAccE
			// 
			this->tbAccE->Location = System::Drawing::Point(73, 30);
			this->tbAccE->Name = L"tbAccE";
			this->tbAccE->Size = System::Drawing::Size(62, 20);
			this->tbAccE->TabIndex = 1;
			// 
			// tbAccN
			// 
			this->tbAccN->Location = System::Drawing::Point(6, 30);
			this->tbAccN->Name = L"tbAccN";
			this->tbAccN->Size = System::Drawing::Size(61, 20);
			this->tbAccN->TabIndex = 0;
			// 
			// bSetAccOffset
			// 
			this->bSetAccOffset->Location = System::Drawing::Point(204, 78);
			this->bSetAccOffset->Name = L"bSetAccOffset";
			this->bSetAccOffset->Size = System::Drawing::Size(55, 65);
			this->bSetAccOffset->TabIndex = 43;
			this->bSetAccOffset->Text = L"İvme Ofset Ayarla";
			this->bSetAccOffset->UseVisualStyleBackColor = true;
			this->bSetAccOffset->Click += gcnew System::EventHandler(this, &MainForm::bSetAccOffset_Click);
			// 
			// groupBox9
			// 
			this->groupBox9->Anchor = static_cast<System::Windows::Forms::AnchorStyles>((System::Windows::Forms::AnchorStyles::Bottom | System::Windows::Forms::AnchorStyles::Left));
			this->groupBox9->Controls->Add(this->groupBox7);
			this->groupBox9->Controls->Add(this->bOpenTrainingDirectory);
			this->groupBox9->Controls->Add(this->groupBox6);
			this->groupBox9->Controls->Add(this->bToggleTraining);
			this->groupBox9->Location = System::Drawing::Point(265, 533);
			this->groupBox9->Name = L"groupBox9";
			this->groupBox9->Size = System::Drawing::Size(212, 212);
			this->groupBox9->TabIndex = 44;
			this->groupBox9->TabStop = false;
			this->groupBox9->Text = L"Eğitim";
			// 
			// groupBox10
			// 
			this->groupBox10->Controls->Add(this->clbDataComponents);
			this->groupBox10->Controls->Add(this->label31);
			this->groupBox10->Controls->Add(this->tbDataCount);
			this->groupBox10->Location = System::Drawing::Point(6, 274);
			this->groupBox10->Name = L"groupBox10";
			this->groupBox10->Size = System::Drawing::Size(244, 573);
			this->groupBox10->TabIndex = 45;
			this->groupBox10->TabStop = false;
			this->groupBox10->Text = L"Grafik Ayarları";
			// 
			// clbDataComponents
			// 
			this->clbDataComponents->CheckOnClick = true;
			this->clbDataComponents->FormattingEnabled = true;
			this->clbDataComponents->Location = System::Drawing::Point(15, 82);
			this->clbDataComponents->Name = L"clbDataComponents";
			this->clbDataComponents->Size = System::Drawing::Size(213, 484);
			this->clbDataComponents->TabIndex = 2;
			this->clbDataComponents->ItemCheck += gcnew System::Windows::Forms::ItemCheckEventHandler(this, &MainForm::clbDataComponents_ItemCheck);
			// 
			// label31
			// 
			this->label31->AutoSize = true;
			this->label31->Location = System::Drawing::Point(100, 34);
			this->label31->Name = L"label31";
			this->label31->Size = System::Drawing::Size(55, 13);
			this->label31->TabIndex = 1;
			this->label31->Text = L"Veri Sayısı";
			// 
			// tbDataCount
			// 
			this->tbDataCount->Location = System::Drawing::Point(170, 31);
			this->tbDataCount->Name = L"tbDataCount";
			this->tbDataCount->Size = System::Drawing::Size(58, 20);
			this->tbDataCount->TabIndex = 0;
			// 
			// tbChartMin
			// 
			this->tbChartMin->Anchor = static_cast<System::Windows::Forms::AnchorStyles>((System::Windows::Forms::AnchorStyles::Bottom | System::Windows::Forms::AnchorStyles::Right));
			this->tbChartMin->Location = System::Drawing::Point(814, 555);
			this->tbChartMin->Name = L"tbChartMin";
			this->tbChartMin->Size = System::Drawing::Size(56, 20);
			this->tbChartMin->TabIndex = 46;
			this->tbChartMin->TextChanged += gcnew System::EventHandler(this, &MainForm::tbChartMin_TextChanged);
			// 
			// tbChartMax
			// 
			this->tbChartMax->Anchor = static_cast<System::Windows::Forms::AnchorStyles>((System::Windows::Forms::AnchorStyles::Bottom | System::Windows::Forms::AnchorStyles::Right));
			this->tbChartMax->Location = System::Drawing::Point(814, 581);
			this->tbChartMax->Name = L"tbChartMax";
			this->tbChartMax->Size = System::Drawing::Size(56, 20);
			this->tbChartMax->TabIndex = 47;
			this->tbChartMax->TextChanged += gcnew System::EventHandler(this, &MainForm::tbChartMax_TextChanged);
			// 
			// label34
			// 
			this->label34->Anchor = static_cast<System::Windows::Forms::AnchorStyles>((System::Windows::Forms::AnchorStyles::Bottom | System::Windows::Forms::AnchorStyles::Right));
			this->label34->AutoSize = true;
			this->label34->Location = System::Drawing::Point(814, 536);
			this->label34->Name = L"label34";
			this->label34->Size = System::Drawing::Size(38, 13);
			this->label34->TabIndex = 48;
			this->label34->Text = L"Grafik:";
			// 
			// label35
			// 
			this->label35->Anchor = static_cast<System::Windows::Forms::AnchorStyles>((System::Windows::Forms::AnchorStyles::Bottom | System::Windows::Forms::AnchorStyles::Left));
			this->label35->AutoSize = true;
			this->label35->Location = System::Drawing::Point(770, 558);
			this->label35->Name = L"label35";
			this->label35->Size = System::Drawing::Size(27, 13);
			this->label35->TabIndex = 49;
			this->label35->Text = L"Min:";
			// 
			// label36
			// 
			this->label36->Anchor = static_cast<System::Windows::Forms::AnchorStyles>((System::Windows::Forms::AnchorStyles::Bottom | System::Windows::Forms::AnchorStyles::Left));
			this->label36->AutoSize = true;
			this->label36->Location = System::Drawing::Point(770, 582);
			this->label36->Name = L"label36";
			this->label36->Size = System::Drawing::Size(36, 13);
			this->label36->TabIndex = 50;
			this->label36->Text = L"Maks:";
			// 
			// tbSystemCal
			// 
			this->tbSystemCal->Location = System::Drawing::Point(54, 20);
			this->tbSystemCal->Name = L"tbSystemCal";
			this->tbSystemCal->Size = System::Drawing::Size(30, 20);
			this->tbSystemCal->TabIndex = 51;
			// 
			// groupBox11
			// 
			this->groupBox11->Anchor = static_cast<System::Windows::Forms::AnchorStyles>((System::Windows::Forms::AnchorStyles::Bottom | System::Windows::Forms::AnchorStyles::Left));
			this->groupBox11->Controls->Add(this->label40);
			this->groupBox11->Controls->Add(this->tbMagCal);
			this->groupBox11->Controls->Add(this->label39);
			this->groupBox11->Controls->Add(this->tbAccCal);
			this->groupBox11->Controls->Add(this->label38);
			this->groupBox11->Controls->Add(this->gbGyroCal);
			this->groupBox11->Controls->Add(this->label37);
			this->groupBox11->Controls->Add(this->tbSystemCal);
			this->groupBox11->Location = System::Drawing::Point(780, 613);
			this->groupBox11->Name = L"groupBox11";
			this->groupBox11->Size = System::Drawing::Size(101, 132);
			this->groupBox11->TabIndex = 52;
			this->groupBox11->TabStop = false;
			this->groupBox11->Text = L"CalibrationData";
			// 
			// label40
			// 
			this->label40->AutoSize = true;
			this->label40->Location = System::Drawing::Point(7, 101);
			this->label40->Name = L"label40";
			this->label40->Size = System::Drawing::Size(28, 13);
			this->label40->TabIndex = 58;
			this->label40->Text = L"Mag";
			// 
			// tbMagCal
			// 
			this->tbMagCal->Location = System::Drawing::Point(54, 98);
			this->tbMagCal->Name = L"tbMagCal";
			this->tbMagCal->Size = System::Drawing::Size(30, 20);
			this->tbMagCal->TabIndex = 57;
			// 
			// label39
			// 
			this->label39->AutoSize = true;
			this->label39->Location = System::Drawing::Point(7, 75);
			this->label39->Name = L"label39";
			this->label39->Size = System::Drawing::Size(26, 13);
			this->label39->TabIndex = 56;
			this->label39->Text = L"Acc";
			// 
			// tbAccCal
			// 
			this->tbAccCal->Location = System::Drawing::Point(54, 72);
			this->tbAccCal->Name = L"tbAccCal";
			this->tbAccCal->Size = System::Drawing::Size(30, 20);
			this->tbAccCal->TabIndex = 55;
			// 
			// label38
			// 
			this->label38->AutoSize = true;
			this->label38->Location = System::Drawing::Point(7, 49);
			this->label38->Name = L"label38";
			this->label38->Size = System::Drawing::Size(29, 13);
			this->label38->TabIndex = 54;
			this->label38->Text = L"Gyro";
			// 
			// gbGyroCal
			// 
			this->gbGyroCal->Location = System::Drawing::Point(54, 46);
			this->gbGyroCal->Name = L"gbGyroCal";
			this->gbGyroCal->Size = System::Drawing::Size(30, 20);
			this->gbGyroCal->TabIndex = 53;
			// 
			// label37
			// 
			this->label37->AutoSize = true;
			this->label37->Location = System::Drawing::Point(7, 23);
			this->label37->Name = L"label37";
			this->label37->Size = System::Drawing::Size(38, 13);
			this->label37->TabIndex = 52;
			this->label37->Text = L"Sistem";
			// 
			// lTimeDiffMs
			// 
			this->lTimeDiffMs->AutoSize = true;
			this->lTimeDiffMs->Location = System::Drawing::Point(249, 258);
			this->lTimeDiffMs->Name = L"lTimeDiffMs";
			this->lTimeDiffMs->Size = System::Drawing::Size(10, 13);
			this->lTimeDiffMs->TabIndex = 54;
			this->lTimeDiffMs->Text = L"-";
			// 
			// label42
			// 
			this->label42->AutoSize = true;
			this->label42->Location = System::Drawing::Point(147, 258);
			this->label42->Name = L"label42";
			this->label42->Size = System::Drawing::Size(52, 13);
			this->label42->TabIndex = 53;
			this->label42->Text = L"Time Diff:";
			// 
			// tbRefreshVelPos
			// 
			this->tbRefreshVelPos->Anchor = static_cast<System::Windows::Forms::AnchorStyles>((System::Windows::Forms::AnchorStyles::Bottom | System::Windows::Forms::AnchorStyles::Left));
			this->tbRefreshVelPos->Location = System::Drawing::Point(689, 545);
			this->tbRefreshVelPos->Name = L"tbRefreshVelPos";
			this->tbRefreshVelPos->Size = System::Drawing::Size(75, 23);
			this->tbRefreshVelPos->TabIndex = 55;
			this->tbRefreshVelPos->Text = L"Sıfırla";
			this->tbRefreshVelPos->UseVisualStyleBackColor = true;
			this->tbRefreshVelPos->Click += gcnew System::EventHandler(this, &MainForm::tbRefreshVelPos_Click);
			// 
			// groupBox12
			// 
			this->groupBox12->Controls->Add(this->lQuaternionDistance);
			this->groupBox12->Controls->Add(this->label45);
			this->groupBox12->Controls->Add(this->tbQDistanceZ);
			this->groupBox12->Controls->Add(this->label44);
			this->groupBox12->Controls->Add(this->tbQDistanceY);
			this->groupBox12->Controls->Add(this->label43);
			this->groupBox12->Controls->Add(this->tbQDistanceX);
			this->groupBox12->Controls->Add(this->label41);
			this->groupBox12->Controls->Add(this->tbQDistanceW);
			this->groupBox12->Location = System::Drawing::Point(1069, 751);
			this->groupBox12->Name = L"groupBox12";
			this->groupBox12->Size = System::Drawing::Size(286, 76);
			this->groupBox12->TabIndex = 56;
			this->groupBox12->TabStop = false;
			this->groupBox12->Text = L"QuaternionDistance";
			// 
			// lQuaternionDistance
			// 
			this->lQuaternionDistance->AutoSize = true;
			this->lQuaternionDistance->Location = System::Drawing::Point(199, 46);
			this->lQuaternionDistance->Name = L"lQuaternionDistance";
			this->lQuaternionDistance->Size = System::Drawing::Size(57, 13);
			this->lQuaternionDistance->TabIndex = 15;
			this->lQuaternionDistance->Text = L"QDistance";
			// 
			// label45
			// 
			this->label45->AutoSize = true;
			this->label45->Location = System::Drawing::Point(152, 27);
			this->label45->Name = L"label45";
			this->label45->Size = System::Drawing::Size(14, 13);
			this->label45->TabIndex = 14;
			this->label45->Text = L"Z";
			// 
			// tbQDistanceZ
			// 
			this->tbQDistanceZ->Location = System::Drawing::Point(144, 43);
			this->tbQDistanceZ->Name = L"tbQDistanceZ";
			this->tbQDistanceZ->Size = System::Drawing::Size(39, 20);
			this->tbQDistanceZ->TabIndex = 13;
			// 
			// label44
			// 
			this->label44->AutoSize = true;
			this->label44->Location = System::Drawing::Point(105, 27);
			this->label44->Name = L"label44";
			this->label44->Size = System::Drawing::Size(14, 13);
			this->label44->TabIndex = 12;
			this->label44->Text = L"Y";
			// 
			// tbQDistanceY
			// 
			this->tbQDistanceY->Location = System::Drawing::Point(99, 43);
			this->tbQDistanceY->Name = L"tbQDistanceY";
			this->tbQDistanceY->Size = System::Drawing::Size(39, 20);
			this->tbQDistanceY->TabIndex = 11;
			// 
			// label43
			// 
			this->label43->AutoSize = true;
			this->label43->Location = System::Drawing::Point(62, 27);
			this->label43->Name = L"label43";
			this->label43->Size = System::Drawing::Size(14, 13);
			this->label43->TabIndex = 10;
			this->label43->Text = L"X";
			// 
			// tbQDistanceX
			// 
			this->tbQDistanceX->Location = System::Drawing::Point(54, 43);
			this->tbQDistanceX->Name = L"tbQDistanceX";
			this->tbQDistanceX->Size = System::Drawing::Size(39, 20);
			this->tbQDistanceX->TabIndex = 9;
			this->tbQDistanceX->TextChanged += gcnew System::EventHandler(this, &MainForm::tbQDistanceX_TextChanged);
			// 
			// label41
			// 
			this->label41->AutoSize = true;
			this->label41->Location = System::Drawing::Point(19, 27);
			this->label41->Name = L"label41";
			this->label41->Size = System::Drawing::Size(18, 13);
			this->label41->TabIndex = 8;
			this->label41->Text = L"W";
			// 
			// tbQDistanceW
			// 
			this->tbQDistanceW->Location = System::Drawing::Point(9, 43);
			this->tbQDistanceW->Name = L"tbQDistanceW";
			this->tbQDistanceW->Size = System::Drawing::Size(39, 20);
			this->tbQDistanceW->TabIndex = 7;
			this->tbQDistanceW->TextChanged += gcnew System::EventHandler(this, &MainForm::tbQDistanceW_TextChanged);
			// 
			// groupBox13
			// 
			this->groupBox13->Anchor = static_cast<System::Windows::Forms::AnchorStyles>((System::Windows::Forms::AnchorStyles::Bottom | System::Windows::Forms::AnchorStyles::Left));
			this->groupBox13->Controls->Add(this->pDirectionQuat8);
			this->groupBox13->Controls->Add(this->pDirectionQuat5);
			this->groupBox13->Controls->Add(this->pDirectionQuat7);
			this->groupBox13->Controls->Add(this->pDirectionQuat2);
			this->groupBox13->Controls->Add(this->pDirectionQuat6);
			this->groupBox13->Controls->Add(this->pDirectionQuat4);
			this->groupBox13->Controls->Add(this->pDirectionQuat1);
			this->groupBox13->Controls->Add(this->pDirectionQuat3);
			this->groupBox13->Controls->Add(this->pDirectionQuat0);
			this->groupBox13->Location = System::Drawing::Point(483, 536);
			this->groupBox13->Name = L"groupBox13";
			this->groupBox13->Size = System::Drawing::Size(200, 201);
			this->groupBox13->TabIndex = 57;
			this->groupBox13->TabStop = false;
			this->groupBox13->Text = L"Directions";
			// 
			// pDirectionQuat8
			// 
			this->pDirectionQuat8->BackColor = System::Drawing::SystemColors::ActiveCaption;
			this->pDirectionQuat8->Controls->Add(this->label52);
			this->pDirectionQuat8->Location = System::Drawing::Point(128, 138);
			this->pDirectionQuat8->Name = L"pDirectionQuat8";
			this->pDirectionQuat8->Size = System::Drawing::Size(55, 55);
			this->pDirectionQuat8->TabIndex = 13;
			this->pDirectionQuat8->Tag = L"8";
			this->pDirectionQuat8->DoubleClick += gcnew System::EventHandler(this, &MainForm::panel2_DoubleClick);
			// 
			// label52
			// 
			this->label52->AutoSize = true;
			this->label52->Location = System::Drawing::Point(21, 20);
			this->label52->Name = L"label52";
			this->label52->Size = System::Drawing::Size(12, 13);
			this->label52->TabIndex = 0;
			this->label52->Text = L"\\";
			// 
			// pDirectionQuat5
			// 
			this->pDirectionQuat5->BackColor = System::Drawing::SystemColors::ActiveCaption;
			this->pDirectionQuat5->Controls->Add(this->label49);
			this->pDirectionQuat5->Location = System::Drawing::Point(128, 77);
			this->pDirectionQuat5->Name = L"pDirectionQuat5";
			this->pDirectionQuat5->Size = System::Drawing::Size(55, 55);
			this->pDirectionQuat5->TabIndex = 13;
			this->pDirectionQuat5->Tag = L"5";
			this->pDirectionQuat5->DoubleClick += gcnew System::EventHandler(this, &MainForm::panel2_DoubleClick);
			// 
			// label49
			// 
			this->label49->AutoSize = true;
			this->label49->Location = System::Drawing::Point(21, 20);
			this->label49->Name = L"label49";
			this->label49->Size = System::Drawing::Size(18, 13);
			this->label49->TabIndex = 0;
			this->label49->Text = L"→";
			// 
			// pDirectionQuat7
			// 
			this->pDirectionQuat7->BackColor = System::Drawing::SystemColors::ActiveCaption;
			this->pDirectionQuat7->Controls->Add(this->label53);
			this->pDirectionQuat7->Location = System::Drawing::Point(67, 138);
			this->pDirectionQuat7->Name = L"pDirectionQuat7";
			this->pDirectionQuat7->Size = System::Drawing::Size(55, 55);
			this->pDirectionQuat7->TabIndex = 12;
			this->pDirectionQuat7->Tag = L"7";
			this->pDirectionQuat7->DoubleClick += gcnew System::EventHandler(this, &MainForm::panel2_DoubleClick);
			// 
			// label53
			// 
			this->label53->AutoSize = true;
			this->label53->Location = System::Drawing::Point(21, 20);
			this->label53->Name = L"label53";
			this->label53->Size = System::Drawing::Size(13, 13);
			this->label53->TabIndex = 0;
			this->label53->Text = L"↓";
			// 
			// pDirectionQuat2
			// 
			this->pDirectionQuat2->BackColor = System::Drawing::SystemColors::ActiveCaption;
			this->pDirectionQuat2->Controls->Add(this->label48);
			this->pDirectionQuat2->Location = System::Drawing::Point(128, 16);
			this->pDirectionQuat2->Name = L"pDirectionQuat2";
			this->pDirectionQuat2->Size = System::Drawing::Size(55, 55);
			this->pDirectionQuat2->TabIndex = 10;
			this->pDirectionQuat2->Tag = L"2";
			this->pDirectionQuat2->DoubleClick += gcnew System::EventHandler(this, &MainForm::panel2_DoubleClick);
			// 
			// label48
			// 
			this->label48->AutoSize = true;
			this->label48->Location = System::Drawing::Point(21, 20);
			this->label48->Name = L"label48";
			this->label48->Size = System::Drawing::Size(12, 13);
			this->label48->TabIndex = 0;
			this->label48->Text = L"/";
			// 
			// pDirectionQuat6
			// 
			this->pDirectionQuat6->BackColor = System::Drawing::SystemColors::ActiveCaption;
			this->pDirectionQuat6->Controls->Add(this->label54);
			this->pDirectionQuat6->Location = System::Drawing::Point(6, 138);
			this->pDirectionQuat6->Name = L"pDirectionQuat6";
			this->pDirectionQuat6->Size = System::Drawing::Size(55, 55);
			this->pDirectionQuat6->TabIndex = 11;
			this->pDirectionQuat6->Tag = L"6";
			this->pDirectionQuat6->DoubleClick += gcnew System::EventHandler(this, &MainForm::panel2_DoubleClick);
			// 
			// label54
			// 
			this->label54->AutoSize = true;
			this->label54->Location = System::Drawing::Point(21, 20);
			this->label54->Name = L"label54";
			this->label54->Size = System::Drawing::Size(12, 13);
			this->label54->TabIndex = 0;
			this->label54->Text = L"/";
			// 
			// pDirectionQuat4
			// 
			this->pDirectionQuat4->BackColor = System::Drawing::SystemColors::ActiveCaption;
			this->pDirectionQuat4->Controls->Add(this->label50);
			this->pDirectionQuat4->Location = System::Drawing::Point(67, 77);
			this->pDirectionQuat4->Name = L"pDirectionQuat4";
			this->pDirectionQuat4->Size = System::Drawing::Size(55, 55);
			this->pDirectionQuat4->TabIndex = 12;
			this->pDirectionQuat4->Tag = L"4";
			this->pDirectionQuat4->DoubleClick += gcnew System::EventHandler(this, &MainForm::panel2_DoubleClick);
			// 
			// label50
			// 
			this->label50->AutoSize = true;
			this->label50->Location = System::Drawing::Point(21, 20);
			this->label50->Name = L"label50";
			this->label50->Size = System::Drawing::Size(14, 13);
			this->label50->TabIndex = 0;
			this->label50->Text = L"X";
			// 
			// pDirectionQuat1
			// 
			this->pDirectionQuat1->BackColor = System::Drawing::SystemColors::ActiveCaption;
			this->pDirectionQuat1->Controls->Add(this->label47);
			this->pDirectionQuat1->Location = System::Drawing::Point(67, 16);
			this->pDirectionQuat1->Name = L"pDirectionQuat1";
			this->pDirectionQuat1->Size = System::Drawing::Size(55, 55);
			this->pDirectionQuat1->TabIndex = 10;
			this->pDirectionQuat1->Tag = L"1";
			this->pDirectionQuat1->Paint += gcnew System::Windows::Forms::PaintEventHandler(this, &MainForm::pDirectionQuat1_Paint);
			this->pDirectionQuat1->DoubleClick += gcnew System::EventHandler(this, &MainForm::panel2_DoubleClick);
			// 
			// label47
			// 
			this->label47->AutoSize = true;
			this->label47->Location = System::Drawing::Point(21, 20);
			this->label47->Name = L"label47";
			this->label47->Size = System::Drawing::Size(13, 13);
			this->label47->TabIndex = 0;
			this->label47->Text = L"↑";
			// 
			// pDirectionQuat3
			// 
			this->pDirectionQuat3->BackColor = System::Drawing::SystemColors::ActiveCaption;
			this->pDirectionQuat3->Controls->Add(this->label51);
			this->pDirectionQuat3->Location = System::Drawing::Point(6, 77);
			this->pDirectionQuat3->Name = L"pDirectionQuat3";
			this->pDirectionQuat3->Size = System::Drawing::Size(55, 55);
			this->pDirectionQuat3->TabIndex = 11;
			this->pDirectionQuat3->Tag = L"3";
			this->pDirectionQuat3->DoubleClick += gcnew System::EventHandler(this, &MainForm::panel2_DoubleClick);
			// 
			// label51
			// 
			this->label51->AutoSize = true;
			this->label51->Location = System::Drawing::Point(21, 20);
			this->label51->Name = L"label51";
			this->label51->Size = System::Drawing::Size(18, 13);
			this->label51->TabIndex = 0;
			this->label51->Text = L"←";
			// 
			// pDirectionQuat0
			// 
			this->pDirectionQuat0->BackColor = System::Drawing::SystemColors::ActiveCaption;
			this->pDirectionQuat0->Controls->Add(this->label46);
			this->pDirectionQuat0->Location = System::Drawing::Point(6, 16);
			this->pDirectionQuat0->Name = L"pDirectionQuat0";
			this->pDirectionQuat0->Size = System::Drawing::Size(55, 55);
			this->pDirectionQuat0->TabIndex = 9;
			this->pDirectionQuat0->Tag = L"0";
			this->pDirectionQuat0->Paint += gcnew System::Windows::Forms::PaintEventHandler(this, &MainForm::panel2_Paint);
			this->pDirectionQuat0->DoubleClick += gcnew System::EventHandler(this, &MainForm::panel2_DoubleClick);
			// 
			// label46
			// 
			this->label46->AutoSize = true;
			this->label46->Location = System::Drawing::Point(21, 20);
			this->label46->Name = L"label46";
			this->label46->Size = System::Drawing::Size(12, 13);
			this->label46->TabIndex = 0;
			this->label46->Text = L"\\";
			// 
			// button3
			// 
			this->button3->Anchor = static_cast<System::Windows::Forms::AnchorStyles>((System::Windows::Forms::AnchorStyles::Bottom | System::Windows::Forms::AnchorStyles::Left));
			this->button3->Location = System::Drawing::Point(541, 743);
			this->button3->Name = L"button3";
			this->button3->Size = System::Drawing::Size(75, 48);
			this->button3->TabIndex = 58;
			this->button3->Text = L"Bakış Açısı İlkle";
			this->button3->UseVisualStyleBackColor = true;
			this->button3->Click += gcnew System::EventHandler(this, &MainForm::button3_Click);
			// 
			// pJoyButton1
			// 
			this->pJoyButton1->Anchor = static_cast<System::Windows::Forms::AnchorStyles>((System::Windows::Forms::AnchorStyles::Bottom | System::Windows::Forms::AnchorStyles::Left));
			this->pJoyButton1->BackColor = System::Drawing::SystemColors::ActiveCaption;
			this->pJoyButton1->Controls->Add(this->label55);
			this->pJoyButton1->Location = System::Drawing::Point(689, 574);
			this->pJoyButton1->Name = L"pJoyButton1";
			this->pJoyButton1->Size = System::Drawing::Size(36, 36);
			this->pJoyButton1->TabIndex = 10;
			this->pJoyButton1->Tag = L"0";
			this->pJoyButton1->Paint += gcnew System::Windows::Forms::PaintEventHandler(this, &MainForm::pJoyButton1_Paint);
			this->pJoyButton1->MouseDoubleClick += gcnew System::Windows::Forms::MouseEventHandler(this, &MainForm::pJoyButton1_MouseDoubleClick);
			// 
			// label55
			// 
			this->label55->Anchor = static_cast<System::Windows::Forms::AnchorStyles>((((System::Windows::Forms::AnchorStyles::Top | System::Windows::Forms::AnchorStyles::Bottom)
				| System::Windows::Forms::AnchorStyles::Left)
				| System::Windows::Forms::AnchorStyles::Right));
			this->label55->AutoSize = true;
			this->label55->Location = System::Drawing::Point(12, 12);
			this->label55->Name = L"label55";
			this->label55->Size = System::Drawing::Size(13, 13);
			this->label55->TabIndex = 0;
			this->label55->Text = L"0";
			// 
			// label56
			// 
			this->label56->Anchor = static_cast<System::Windows::Forms::AnchorStyles>((((System::Windows::Forms::AnchorStyles::Top | System::Windows::Forms::AnchorStyles::Bottom)
				| System::Windows::Forms::AnchorStyles::Left)
				| System::Windows::Forms::AnchorStyles::Right));
			this->label56->AutoSize = true;
			this->label56->Location = System::Drawing::Point(12, 12);
			this->label56->Name = L"label56";
			this->label56->Size = System::Drawing::Size(13, 13);
			this->label56->TabIndex = 0;
			this->label56->Text = L"1";
			// 
			// pJoyButton2
			// 
			this->pJoyButton2->Anchor = static_cast<System::Windows::Forms::AnchorStyles>((System::Windows::Forms::AnchorStyles::Bottom | System::Windows::Forms::AnchorStyles::Left));
			this->pJoyButton2->BackColor = System::Drawing::SystemColors::ActiveCaption;
			this->pJoyButton2->Controls->Add(this->label56);
			this->pJoyButton2->Location = System::Drawing::Point(728, 574);
			this->pJoyButton2->Name = L"pJoyButton2";
			this->pJoyButton2->Size = System::Drawing::Size(36, 36);
			this->pJoyButton2->TabIndex = 11;
			this->pJoyButton2->Tag = L"1";
			this->pJoyButton2->MouseDoubleClick += gcnew System::Windows::Forms::MouseEventHandler(this, &MainForm::pJoyButton1_MouseDoubleClick);
			// 
			// label57
			// 
			this->label57->Anchor = static_cast<System::Windows::Forms::AnchorStyles>((((System::Windows::Forms::AnchorStyles::Top | System::Windows::Forms::AnchorStyles::Bottom)
				| System::Windows::Forms::AnchorStyles::Left)
				| System::Windows::Forms::AnchorStyles::Right));
			this->label57->AutoSize = true;
			this->label57->Location = System::Drawing::Point(12, 12);
			this->label57->Name = L"label57";
			this->label57->Size = System::Drawing::Size(13, 13);
			this->label57->TabIndex = 0;
			this->label57->Text = L"2";
			// 
			// pJoyButton3
			// 
			this->pJoyButton3->Anchor = static_cast<System::Windows::Forms::AnchorStyles>((System::Windows::Forms::AnchorStyles::Bottom | System::Windows::Forms::AnchorStyles::Left));
			this->pJoyButton3->BackColor = System::Drawing::SystemColors::ActiveCaption;
			this->pJoyButton3->Controls->Add(this->label57);
			this->pJoyButton3->Location = System::Drawing::Point(689, 617);
			this->pJoyButton3->Name = L"pJoyButton3";
			this->pJoyButton3->Size = System::Drawing::Size(36, 36);
			this->pJoyButton3->TabIndex = 11;
			this->pJoyButton3->Tag = L"2";
			this->pJoyButton3->MouseDoubleClick += gcnew System::Windows::Forms::MouseEventHandler(this, &MainForm::pJoyButton1_MouseDoubleClick);
			// 
			// label58
			// 
			this->label58->Anchor = static_cast<System::Windows::Forms::AnchorStyles>((((System::Windows::Forms::AnchorStyles::Top | System::Windows::Forms::AnchorStyles::Bottom)
				| System::Windows::Forms::AnchorStyles::Left)
				| System::Windows::Forms::AnchorStyles::Right));
			this->label58->AutoSize = true;
			this->label58->Location = System::Drawing::Point(12, 12);
			this->label58->Name = L"label58";
			this->label58->Size = System::Drawing::Size(13, 13);
			this->label58->TabIndex = 0;
			this->label58->Text = L"3";
			// 
			// pJoyButton4
			// 
			this->pJoyButton4->Anchor = static_cast<System::Windows::Forms::AnchorStyles>((System::Windows::Forms::AnchorStyles::Bottom | System::Windows::Forms::AnchorStyles::Left));
			this->pJoyButton4->BackColor = System::Drawing::SystemColors::ActiveCaption;
			this->pJoyButton4->Controls->Add(this->label58);
			this->pJoyButton4->Location = System::Drawing::Point(728, 617);
			this->pJoyButton4->Name = L"pJoyButton4";
			this->pJoyButton4->Size = System::Drawing::Size(36, 36);
			this->pJoyButton4->TabIndex = 11;
			this->pJoyButton4->Tag = L"3";
			this->pJoyButton4->MouseDoubleClick += gcnew System::Windows::Forms::MouseEventHandler(this, &MainForm::pJoyButton1_MouseDoubleClick);
			// 
			// label59
			// 
			this->label59->Anchor = static_cast<System::Windows::Forms::AnchorStyles>((((System::Windows::Forms::AnchorStyles::Top | System::Windows::Forms::AnchorStyles::Bottom)
				| System::Windows::Forms::AnchorStyles::Left)
				| System::Windows::Forms::AnchorStyles::Right));
			this->label59->AutoSize = true;
			this->label59->Location = System::Drawing::Point(12, 12);
			this->label59->Name = L"label59";
			this->label59->Size = System::Drawing::Size(13, 13);
			this->label59->TabIndex = 0;
			this->label59->Text = L"4";
			// 
			// pJoyButton5
			// 
			this->pJoyButton5->Anchor = static_cast<System::Windows::Forms::AnchorStyles>((System::Windows::Forms::AnchorStyles::Bottom | System::Windows::Forms::AnchorStyles::Left));
			this->pJoyButton5->BackColor = System::Drawing::SystemColors::ActiveCaption;
			this->pJoyButton5->Controls->Add(this->label59);
			this->pJoyButton5->Location = System::Drawing::Point(689, 658);
			this->pJoyButton5->Name = L"pJoyButton5";
			this->pJoyButton5->Size = System::Drawing::Size(36, 36);
			this->pJoyButton5->TabIndex = 59;
			this->pJoyButton5->Tag = L"4";
			this->pJoyButton5->MouseDoubleClick += gcnew System::Windows::Forms::MouseEventHandler(this, &MainForm::pJoyButton1_MouseDoubleClick);
			// 
			// label60
			// 
			this->label60->Anchor = static_cast<System::Windows::Forms::AnchorStyles>((((System::Windows::Forms::AnchorStyles::Top | System::Windows::Forms::AnchorStyles::Bottom)
				| System::Windows::Forms::AnchorStyles::Left)
				| System::Windows::Forms::AnchorStyles::Right));
			this->label60->AutoSize = true;
			this->label60->Location = System::Drawing::Point(12, 12);
			this->label60->Name = L"label60";
			this->label60->Size = System::Drawing::Size(13, 13);
			this->label60->TabIndex = 0;
			this->label60->Text = L"5";
			// 
			// pJoyButton6
			// 
			this->pJoyButton6->Anchor = static_cast<System::Windows::Forms::AnchorStyles>((System::Windows::Forms::AnchorStyles::Bottom | System::Windows::Forms::AnchorStyles::Left));
			this->pJoyButton6->BackColor = System::Drawing::SystemColors::ActiveCaption;
			this->pJoyButton6->Controls->Add(this->label60);
			this->pJoyButton6->Location = System::Drawing::Point(728, 658);
			this->pJoyButton6->Name = L"pJoyButton6";
			this->pJoyButton6->Size = System::Drawing::Size(36, 36);
			this->pJoyButton6->TabIndex = 11;
			this->pJoyButton6->Tag = L"5";
			this->pJoyButton6->MouseDoubleClick += gcnew System::Windows::Forms::MouseEventHandler(this, &MainForm::pJoyButton1_MouseDoubleClick);
			// 
			// pJoyButton8
			// 
			this->pJoyButton8->Anchor = static_cast<System::Windows::Forms::AnchorStyles>((System::Windows::Forms::AnchorStyles::Bottom | System::Windows::Forms::AnchorStyles::Left));
			this->pJoyButton8->BackColor = System::Drawing::SystemColors::ActiveCaption;
			this->pJoyButton8->Controls->Add(this->label61);
			this->pJoyButton8->Location = System::Drawing::Point(728, 699);
			this->pJoyButton8->Name = L"pJoyButton8";
			this->pJoyButton8->Size = System::Drawing::Size(36, 36);
			this->pJoyButton8->TabIndex = 60;
			this->pJoyButton8->Tag = L"7";
			this->pJoyButton8->MouseDoubleClick += gcnew System::Windows::Forms::MouseEventHandler(this, &MainForm::pJoyButton1_MouseDoubleClick);
			// 
			// label61
			// 
			this->label61->Anchor = static_cast<System::Windows::Forms::AnchorStyles>((((System::Windows::Forms::AnchorStyles::Top | System::Windows::Forms::AnchorStyles::Bottom)
				| System::Windows::Forms::AnchorStyles::Left)
				| System::Windows::Forms::AnchorStyles::Right));
			this->label61->AutoSize = true;
			this->label61->Location = System::Drawing::Point(12, 12);
			this->label61->Name = L"label61";
			this->label61->Size = System::Drawing::Size(13, 13);
			this->label61->TabIndex = 0;
			this->label61->Text = L"7";
			// 
			// pJoyButton7
			// 
			this->pJoyButton7->Anchor = static_cast<System::Windows::Forms::AnchorStyles>((System::Windows::Forms::AnchorStyles::Bottom | System::Windows::Forms::AnchorStyles::Left));
			this->pJoyButton7->BackColor = System::Drawing::SystemColors::ActiveCaption;
			this->pJoyButton7->Controls->Add(this->label62);
			this->pJoyButton7->Location = System::Drawing::Point(689, 699);
			this->pJoyButton7->Name = L"pJoyButton7";
			this->pJoyButton7->Size = System::Drawing::Size(36, 36);
			this->pJoyButton7->TabIndex = 61;
			this->pJoyButton7->Tag = L"6";
			this->pJoyButton7->MouseDoubleClick += gcnew System::Windows::Forms::MouseEventHandler(this, &MainForm::pJoyButton1_MouseDoubleClick);
			// 
			// label62
			// 
			this->label62->Anchor = static_cast<System::Windows::Forms::AnchorStyles>((((System::Windows::Forms::AnchorStyles::Top | System::Windows::Forms::AnchorStyles::Bottom)
				| System::Windows::Forms::AnchorStyles::Left)
				| System::Windows::Forms::AnchorStyles::Right));
			this->label62->AutoSize = true;
			this->label62->Location = System::Drawing::Point(12, 12);
			this->label62->Name = L"label62";
			this->label62->Size = System::Drawing::Size(13, 13);
			this->label62->TabIndex = 0;
			this->label62->Text = L"6";
			// 
			// bAlgorithmTest
			// 
			this->bAlgorithmTest->Anchor = static_cast<System::Windows::Forms::AnchorStyles>((System::Windows::Forms::AnchorStyles::Bottom | System::Windows::Forms::AnchorStyles::Left));
			this->bAlgorithmTest->Location = System::Drawing::Point(288, 752);
			this->bAlgorithmTest->Name = L"bAlgorithmTest";
			this->bAlgorithmTest->Size = System::Drawing::Size(134, 30);
			this->bAlgorithmTest->TabIndex = 62;
			this->bAlgorithmTest->Text = L"Algoritma Test";
			this->bAlgorithmTest->UseVisualStyleBackColor = true;
			this->bAlgorithmTest->Click += gcnew System::EventHandler(this, &MainForm::bAlgorithmTest_Click);
			// 
			// bSaveSnapshot
			// 
			this->bSaveSnapshot->Anchor = static_cast<System::Windows::Forms::AnchorStyles>((System::Windows::Forms::AnchorStyles::Bottom | System::Windows::Forms::AnchorStyles::Left));
			this->bSaveSnapshot->Location = System::Drawing::Point(635, 743);
			this->bSaveSnapshot->Name = L"bSaveSnapshot";
			this->bSaveSnapshot->Size = System::Drawing::Size(75, 48);
			this->bSaveSnapshot->TabIndex = 63;
			this->bSaveSnapshot->Text = L"Snapshot Filesave";
			this->bSaveSnapshot->UseVisualStyleBackColor = true;
			this->bSaveSnapshot->Click += gcnew System::EventHandler(this, &MainForm::bSaveSnapshot_Click);
			// 
			// cbAccOffset
			// 
			this->cbAccOffset->Anchor = static_cast<System::Windows::Forms::AnchorStyles>((System::Windows::Forms::AnchorStyles::Bottom | System::Windows::Forms::AnchorStyles::Right));
			this->cbAccOffset->AutoSize = true;
			this->cbAccOffset->Location = System::Drawing::Point(893, 743);
			this->cbAccOffset->Name = L"cbAccOffset";
			this->cbAccOffset->Size = System::Drawing::Size(79, 17);
			this->cbAccOffset->TabIndex = 64;
			this->cbAccOffset->Text = L"Acc Offset ";
			this->cbAccOffset->UseVisualStyleBackColor = true;
			this->cbAccOffset->CheckedChanged += gcnew System::EventHandler(this, &MainForm::checkBox1_CheckedChanged);
			// 
			// cbAddDataPointsFlag
			// 
			this->cbAddDataPointsFlag->Anchor = static_cast<System::Windows::Forms::AnchorStyles>((System::Windows::Forms::AnchorStyles::Bottom | System::Windows::Forms::AnchorStyles::Right));
			this->cbAddDataPointsFlag->AutoSize = true;
			this->cbAddDataPointsFlag->Checked = true;
			this->cbAddDataPointsFlag->CheckState = System::Windows::Forms::CheckState::Checked;
			this->cbAddDataPointsFlag->Location = System::Drawing::Point(893, 767);
			this->cbAddDataPointsFlag->Name = L"cbAddDataPointsFlag";
			this->cbAddDataPointsFlag->Size = System::Drawing::Size(68, 17);
			this->cbAddDataPointsFlag->TabIndex = 65;
			this->cbAddDataPointsFlag->Text = L"Veri Ekle";
			this->cbAddDataPointsFlag->UseVisualStyleBackColor = true;
			this->cbAddDataPointsFlag->CheckedChanged += gcnew System::EventHandler(this, &MainForm::cbAddDataPointsFlag_CheckedChanged);
			// 
			// MainForm
			// 
			this->AutoScaleDimensions = System::Drawing::SizeF(6, 13);
			this->AutoScaleMode = System::Windows::Forms::AutoScaleMode::Font;
			this->ClientSize = System::Drawing::Size(1370, 847);
			this->Controls->Add(this->cbAddDataPointsFlag);
			this->Controls->Add(this->cbAccOffset);
			this->Controls->Add(this->bSaveSnapshot);
			this->Controls->Add(this->bAlgorithmTest);
			this->Controls->Add(this->pJoyButton8);
			this->Controls->Add(this->pJoyButton6);
			this->Controls->Add(this->pJoyButton7);
			this->Controls->Add(this->pJoyButton5);
			this->Controls->Add(this->pJoyButton4);
			this->Controls->Add(this->pJoyButton3);
			this->Controls->Add(this->pJoyButton2);
			this->Controls->Add(this->pJoyButton1);
			this->Controls->Add(this->button3);
			this->Controls->Add(this->groupBox13);
			this->Controls->Add(this->groupBox12);
			this->Controls->Add(this->tbRefreshVelPos);
			this->Controls->Add(this->lTimeDiffMs);
			this->Controls->Add(this->label42);
			this->Controls->Add(this->groupBox11);
			this->Controls->Add(this->label36);
			this->Controls->Add(this->label35);
			this->Controls->Add(this->label34);
			this->Controls->Add(this->tbChartMax);
			this->Controls->Add(this->tbChartMin);
			this->Controls->Add(this->groupBox10);
			this->Controls->Add(this->groupBox9);
			this->Controls->Add(this->bSetAccOffset);
			this->Controls->Add(this->groupBox8);
			this->Controls->Add(this->label29);
			this->Controls->Add(this->lSerialBufferReadSize);
			this->Controls->Add(this->label28);
			this->Controls->Add(this->lBufferSizeDecreased);
			this->Controls->Add(this->label26);
			this->Controls->Add(this->lDifferenceBetweenReceivedBytes);
			this->Controls->Add(this->label27);
			this->Controls->Add(this->lTotalReceivedBytes);
			this->Controls->Add(this->label25);
			this->Controls->Add(this->lBufferReset);
			this->Controls->Add(this->label24);
			this->Controls->Add(this->lTotalMessageCount);
			this->Controls->Add(this->label22);
			this->Controls->Add(this->button1);
			this->Controls->Add(this->sensorFlowChart);
			this->Controls->Add(this->groupBox5);
			this->Controls->Add(this->tbInvalidMessage);
			this->Controls->Add(this->label17);
			this->Controls->Add(this->label16);
			this->Controls->Add(this->label15);
			this->Controls->Add(this->label14);
			this->Controls->Add(this->tbTimestamp);
			this->Controls->Add(this->tbType);
			this->Controls->Add(this->tbSensorID);
			this->Controls->Add(this->tbVersion);
			this->Controls->Add(this->groupBox4);
			this->Controls->Add(this->groupBox3);
			this->Controls->Add(this->groupBox2);
			this->Controls->Add(this->groupBox1);
			this->Controls->Add(this->tbConsole);
			this->Controls->Add(this->bInitImu);
			this->Name = L"MainForm";
			this->Text = L"MainForm";
			this->Load += gcnew System::EventHandler(this, &MainForm::MainForm_Load);
			this->Shown += gcnew System::EventHandler(this, &MainForm::MainForm_Shown);
			this->groupBox1->ResumeLayout(false);
			this->groupBox1->PerformLayout();
			this->groupBox2->ResumeLayout(false);
			this->groupBox2->PerformLayout();
			this->groupBox3->ResumeLayout(false);
			this->groupBox3->PerformLayout();
			this->groupBox4->ResumeLayout(false);
			this->groupBox4->PerformLayout();
			this->groupBox5->ResumeLayout(false);
			this->groupBox5->PerformLayout();
			(cli::safe_cast<System::ComponentModel::ISupportInitialize^>(this->sensorFlowChart))->EndInit();
			this->groupBox6->ResumeLayout(false);
			this->groupBox6->PerformLayout();
			this->panel1->ResumeLayout(false);
			this->panel1->PerformLayout();
			this->groupBox7->ResumeLayout(false);
			this->groupBox8->ResumeLayout(false);
			this->groupBox8->PerformLayout();
			this->groupBox9->ResumeLayout(false);
			this->groupBox10->ResumeLayout(false);
			this->groupBox10->PerformLayout();
			this->groupBox11->ResumeLayout(false);
			this->groupBox11->PerformLayout();
			this->groupBox12->ResumeLayout(false);
			this->groupBox12->PerformLayout();
			this->groupBox13->ResumeLayout(false);
			this->pDirectionQuat8->ResumeLayout(false);
			this->pDirectionQuat8->PerformLayout();
			this->pDirectionQuat5->ResumeLayout(false);
			this->pDirectionQuat5->PerformLayout();
			this->pDirectionQuat7->ResumeLayout(false);
			this->pDirectionQuat7->PerformLayout();
			this->pDirectionQuat2->ResumeLayout(false);
			this->pDirectionQuat2->PerformLayout();
			this->pDirectionQuat6->ResumeLayout(false);
			this->pDirectionQuat6->PerformLayout();
			this->pDirectionQuat4->ResumeLayout(false);
			this->pDirectionQuat4->PerformLayout();
			this->pDirectionQuat1->ResumeLayout(false);
			this->pDirectionQuat1->PerformLayout();
			this->pDirectionQuat3->ResumeLayout(false);
			this->pDirectionQuat3->PerformLayout();
			this->pDirectionQuat0->ResumeLayout(false);
			this->pDirectionQuat0->PerformLayout();
			this->pJoyButton1->ResumeLayout(false);
			this->pJoyButton1->PerformLayout();
			this->pJoyButton2->ResumeLayout(false);
			this->pJoyButton2->PerformLayout();
			this->pJoyButton3->ResumeLayout(false);
			this->pJoyButton3->PerformLayout();
			this->pJoyButton4->ResumeLayout(false);
			this->pJoyButton4->PerformLayout();
			this->pJoyButton5->ResumeLayout(false);
			this->pJoyButton5->PerformLayout();
			this->pJoyButton6->ResumeLayout(false);
			this->pJoyButton6->PerformLayout();
			this->pJoyButton8->ResumeLayout(false);
			this->pJoyButton8->PerformLayout();
			this->pJoyButton7->ResumeLayout(false);
			this->pJoyButton7->PerformLayout();
			this->ResumeLayout(false);
			this->PerformLayout();

		}
#pragma endregion
	private: System::Void bInitImu_Click(System::Object^  sender, System::EventArgs^  e) {

		IMUNavigationReset = true;
		

	}
	private: System::Void MainForm_Load(System::Object^  sender, System::EventArgs^  e) {

		InitDataComponents();
		

		double min_val = pUISettingsSaver->GetChartAxisLimit("min");
		double max_val = pUISettingsSaver->GetChartAxisLimit("max");
		float w, x, y, z;

		for (int i = 0; i < 9; i++) {
			pUISettingsSaver->GetDirectionQuaternion(i, &w, &x, &y, &z);

			pDirectionQuaternions[i].a = w;
			pDirectionQuaternions[i].b = x;
			pDirectionQuaternions[i].c = y;
			pDirectionQuaternions[i].d = z;
		}

		if (min_val > max_val || (max_val - min_val) < 0.0001) {

			min_val = -1;
			max_val = 1;
		}
	
		tbChartMin->Text = min_val.ToString();
		tbChartMax->Text = max_val.ToString();

		sensorFlowChart->ChartAreas[0]->AxisY->Minimum = min_val;
		sensorFlowChart->ChartAreas[0]->AxisY->Maximum = max_val;



	}
	private:
		void UpdateJoystick()
		{
			int X, Y, Z, ZR, XR;
			int middle = 32767 / 2;
			int max = 0;
			int min = 32767;

			int id = 1;
			bool res;
			switch (FoundDirectionID) {
			case 0:
				X = max;
				Y = max;
				break;
			case 1:
				X = middle;
				Y = max;
				break;
			case 2:
				X = min;
				Y = max;
				break;
			case 3:
				X = max;
				Y = middle;
				break;
			case 4:
				X = middle;
				Y = middle;
				break;
			case 5:
				X = min;
				Y = middle;
				break;
			case 6:
				X = max;
				Y = min;
				break;
			case 7:
				X = middle;
				Y = min;
				break;
			case 8:
				X = min;
				Y = min;
				break;

			default:
				X = middle;
				Y = middle;
			}
			res = pJoystick->SetAxis(X, id, HID_USAGES::HID_USAGE_X);
			res = pJoystick->SetAxis(Y, id, HID_USAGES::HID_USAGE_Y);

			res = pJoystick->SetAxis(middle, id, HID_USAGES::HID_USAGE_RX);
			res = pJoystick->SetAxis(middle, id, HID_USAGES::HID_USAGE_RY);
			//res = pJoystick->SetAxis(0, id, HID_USAGES::HID_USAGE_Z);

			for (int i = 0; i < 8; i++) {
				res = pJoystick->SetBtn(pJoyButtons[i], id, i + 1);
			}
			
			
		}
	Quaternion ToQuaternion(double yaw, double pitch, double roll) // yaw (Z), pitch (Y), roll (X)
	{
		// Abbreviations for the various angular functions
		//yaw = -yaw;
		//pitch = -pitch;
		//roll = -roll;
	
		double cy = cos(yaw * 0.5 * 0.0174527f);
		double sy = sin(yaw * 0.5* 0.0174527f);
		double cp = cos(pitch * 0.5* 0.0174527f);
		double sp = sin(pitch * 0.5* 0.0174527f);
		double cr = cos(roll * 0.5* 0.0174527f);
		double sr = sin(roll * 0.5* 0.0174527f);
		
		Quaternion q;
		/*
		q.a = cy * cp * cr + sy * sp * sr;
		q.b = cy * cp * sr - sy * sp * cr;
		q.c = sy * cp * sr + cy * sp * cr;
		q.d = sy * cp * cr - cy * sp * sr;
		*/
		/*
		Quaternion q;
		q.a = cy * cp * cr - sy * sp * sr;
		q.b = cy * cp * sr + sy * sp * cr;
		q.c = sy * cp * sr + cy * sp * sr;
		q.d = cy * sp * cr - sy * cp * sr;
		*/

		q.a = cy * cp * cr + sy * sp * sr;
		q.b = cy * cp * sr + sy * sp * cr;
		q.c = cy * sp * cr - sy * cp * sr;
		q.d = sy * cp * cr - cy * sp * sr;

		return q;
	}

	void  CalculateMeanAndStd(float timestamp, ActionArrayBuffer ^action_array, array<float> ^sendor_data_array)
	{
		array<double> ^means = gcnew array<double>(1);
		array<double> ^center_of_masses = gcnew array<double>(1);
		array<double> ^rms = gcnew array<double>(1);


		array<array<float> ^> ^p_array = action_array->GetBuffer();

		array<array<float> ^> ^p_sub_array = gcnew array<array<float> ^>(20);

		if (p_array->Length > 40) {

			for (int i = 0; i < 19 && i < p_array->Length; i++) {
				p_sub_array[i] = gcnew array<float>(1);

				p_sub_array[i][0] = p_array[p_array->Length - 19 + i][SENSOR_DATA_TYPE_COUNT + SENSOR_DATA_TYPE_FILTERED_ACC_NED_Z];

			}

			p_sub_array[19] = gcnew array<float>(1);
			p_sub_array[19][0] = sendor_data_array[SENSOR_DATA_TYPE_COUNT + SENSOR_DATA_TYPE_FILTERED_ACC_NED_Z];



			//MeanAndCenterOffMass::Calculate(p_sub_array, 0, 0, means, center_of_masses);
			//StandartDeviation::Calculate(p_sub_array, 0, 0, means, std_devs);
			RMS::Calculate(p_sub_array, 0, 0, 0, p_sub_array->Length, rms);

			sendor_data_array[SENSOR_DATA_TYPE_COUNT + SENSOR_DATA_FILTERED_NED_ACC_DOWN_RMS] = rms[0];
		}


	}

	void UpdateRelativeVelocityAndPosition(float timestamp, ActionArrayBuffer ^action_array, array<float> ^sendor_data_array)
	{
		float velocity_3d[3] = { 0, 0, 0 };
		float position_3d[3] = { 0, 0, 0 };

		static float previous_position_3d[3];

		array<array<float> ^> ^p_array = action_array->GetBuffer();
/*
		for (int i = p_array->Length; i > 0; i--) {
			velocity_3d[0] += p_array[i - 1][SENSOR_DATA_TYPE_COUNT + SENSOR_DATA_TYPE_RELATIVE_ACCELERATION_NED_X] * 1e-2;
			velocity_3d[1] += p_array[i - 1][SENSOR_DATA_TYPE_COUNT + SENSOR_DATA_TYPE_RELATIVE_ACCELERATION_NED_Y] * 1e-2;
			velocity_3d[2] += (p_array[i -1][SENSOR_DATA_TYPE_COUNT + SENSOR_DATA_TYPE_RELATIVE_ACCELERATION_NED_Z] - BNOCONTROL_SENSOR_GRAVITY_VALUE) * 1e-2;

			position_3d[0] += velocity_3d[0]* 1e-2;
			position_3d[1] += velocity_3d[1]* 1e-2;
			position_3d[2] += velocity_3d[2]* 1e-2;
		}*/

		for (int i = 0 ; i < p_array->Length; i++) {
			velocity_3d[0] += p_array[i][SENSOR_DATA_TYPE_COUNT + SENSOR_DATA_TYPE_RELATIVE_ACCELERATION_NED_X] * 1e-2;
			velocity_3d[1] += p_array[i][SENSOR_DATA_TYPE_COUNT + SENSOR_DATA_TYPE_RELATIVE_ACCELERATION_NED_Y] * 1e-2;
			velocity_3d[2] += (p_array[i][SENSOR_DATA_TYPE_COUNT + SENSOR_DATA_TYPE_RELATIVE_ACCELERATION_NED_Z] - BNOCONTROL_SENSOR_GRAVITY_VALUE) * 1e-2;

			position_3d[0] += velocity_3d[0] * 1e-2;
			position_3d[1] += velocity_3d[1] * 1e-2;
			position_3d[2] += velocity_3d[2] * 1e-2;
		}

		sendor_data_array[SENSOR_DATA_TYPE_COUNT + SENSOR_DATA_TYPE_RELATIVE_VELOCITY_NED_X] = velocity_3d[0];
		sendor_data_array[SENSOR_DATA_TYPE_COUNT + SENSOR_DATA_TYPE_RELATIVE_VELOCITY_NED_Y] = velocity_3d[1];
		sendor_data_array[SENSOR_DATA_TYPE_COUNT + SENSOR_DATA_TYPE_RELATIVE_VELOCITY_NED_Z] = velocity_3d[2];

		sendor_data_array[SENSOR_DATA_TYPE_COUNT + SENSOR_DATA_TYPE_RELATIVE_POSITION_NED_X] = position_3d[0];
		sendor_data_array[SENSOR_DATA_TYPE_COUNT + SENSOR_DATA_TYPE_RELATIVE_POSITION_NED_Y] = position_3d[1];
		sendor_data_array[SENSOR_DATA_TYPE_COUNT + SENSOR_DATA_TYPE_RELATIVE_POSITION_NED_Z] = position_3d[2];

		sendor_data_array[SENSOR_DATA_TYPE_COUNT + SENSOR_DATA_TYPE_RELATIVE_POSITION_NED_Z_DIFF] = sendor_data_array[SENSOR_DATA_TYPE_COUNT + SENSOR_DATA_TYPE_RELATIVE_POSITION_NED_Z] - previous_position_3d[2];
		
		previous_position_3d[2] = sendor_data_array[SENSOR_DATA_TYPE_COUNT + SENSOR_DATA_TYPE_RELATIVE_POSITION_NED_Z];
	}
	void DirectionCheckByQuaternions(ViewSerialEvent event, array<float> ^sendor_data_array)
	{
		int i = 0;
		FoundDirectionID = -1;
		double closest_distance = 1.0f;
		double distance = 1.0f;
		float roll_angle_rad = (event.pEvent->euler[2] * DEGREE_TO_RADIAN_CONSTANT);
		float pitch_angle_rad = (event.pEvent->euler[1] * DEGREE_TO_RADIAN_CONSTANT);
		float yaw_angle_rad = (event.pEvent->euler[0] * DEGREE_TO_RADIAN_CONSTANT);
		Quaternion q = quaternion_initialize(event.pEvent->quat[0], event.pEvent->quat[1], event.pEvent->quat[2], event.pEvent->quat[3]);
		
		q = quaternion_normalize(q);

		Quaternion q_roll_canceller = quaternion_from_euler(-0, -0, roll_angle_rad);

		//q_roll_canceller.a = -q_roll_canceller.a;
		q = quaternion_product(q, q_roll_canceller);

		
		q = quaternion_product(q, *pStandpointQuaternion);


		euler_angles roll_canceller = ToEulerAngles(q_roll_canceller);
		euler_angles original = ToEulerAngles(q);

		sendor_data_array[SENSOR_DATA_TYPE_COUNT + SENSOR_DATA_TYPE_ROLL_COMPANSED_QUATERNION_W] = q.a;
		sendor_data_array[SENSOR_DATA_TYPE_COUNT + SENSOR_DATA_TYPE_ROLL_COMPANSED_QUATERNION_X] = q.b;
		sendor_data_array[SENSOR_DATA_TYPE_COUNT + SENSOR_DATA_TYPE_ROLL_COMPANSED_QUATERNION_Y] = q.c;
		sendor_data_array[SENSOR_DATA_TYPE_COUNT + SENSOR_DATA_TYPE_ROLL_COMPANSED_QUATERNION_Z] = q.d;
#if 0
		sendor_data_array[SENSOR_DATA_TYPE_COUNT + 33] = roll_canceller.yaw * (180 / 3.1415);
		sendor_data_array[SENSOR_DATA_TYPE_COUNT + 34] = roll_canceller.pitch* (180 / 3.1415);
		sendor_data_array[SENSOR_DATA_TYPE_COUNT + 35] = roll_canceller.roll* (180 / 3.1415);
#endif
		for (i = 0; i < 9; i++) {
			distance = quaternion_distance(q, pDirectionQuaternions[i]);
			if (closest_distance > distance) {

				closest_distance = distance;
				FoundDirectionID = i;
			}

		}

		if (FoundDirectionID != -1) {
			for (i = 0; i < 9; i++) {

				if (i != FoundDirectionID) {
					pDirectionQuatControls[i]->BackColor = System::Drawing::Color::Gray;
				}
				else {
					pDirectionQuatControls[i]->BackColor = System::Drawing::Color::Green;
				}

			}

		}
	}
	void CalculateQuaternionDistance(ViewSerialEvent event, array<float> ^sendor_data_array)
	{
		Quaternion q = quaternion_initialize(event.pEvent->quat[0], event.pEvent->quat[1], event.pEvent->quat[2], event.pEvent->quat[3]);
		double distance = quaternion_distance(*pRefQuaternion, q);

		sendor_data_array[SENSOR_DATA_TYPE_COUNT + SENSOR_DATA_TYPE_QUATERNION_DISTANCE_TO_REFERANCE] = distance;

	}
	void UpdateVelocityAndPosition(float timestamp, ViewSerialEvent event, array<float> ^sendor_data_array)
	{
		float ned_acc[3];
		float acc[3];

		static float previos_ned_acc[3] = { 0, 0, 0 };
		float ned_acc_diff[3] = { 0, 0, 0 };
		counter++;

		acc[0] = event.pEvent->acc[0] - acc_offset[0];
		acc[1] = event.pEvent->acc[1] - acc_offset[1];
		acc[2] = event.pEvent->acc[2] - acc_offset[2];

		BodyToNedAcc(event.pEvent->quat, acc, ned_acc);

		tbAccN->Text = Convert::ToString(ned_acc[0]);
		tbAccE->Text = Convert::ToString(ned_acc[1]);
		tbAccD->Text = Convert::ToString(ned_acc[2]);


		sendor_data_array[SENSOR_DATA_TYPE_COUNT + SENSOR_DATA_TYPE_RELATIVE_ACCELERATION_NED_X] = ned_acc[0];
		sendor_data_array[SENSOR_DATA_TYPE_COUNT + SENSOR_DATA_TYPE_RELATIVE_ACCELERATION_NED_Y] = ned_acc[1];
		sendor_data_array[SENSOR_DATA_TYPE_COUNT + SENSOR_DATA_TYPE_RELATIVE_ACCELERATION_NED_Z] = ned_acc[2];


		sendor_data_array[SENSOR_DATA_TYPE_COUNT + SENSOR_DATA_TYPE_FILTERED_ACC_NED_X] = pKalmanEstimators[SENSOR_DATA_TYPE_COUNT + SENSOR_DATA_TYPE_FILTERED_ACC_NED_X]->updateEstimate(ned_acc[0]);
		sendor_data_array[SENSOR_DATA_TYPE_COUNT + SENSOR_DATA_TYPE_FILTERED_ACC_NED_Y] = pKalmanEstimators[SENSOR_DATA_TYPE_COUNT + SENSOR_DATA_TYPE_FILTERED_ACC_NED_Y]->updateEstimate(ned_acc[1]);
		sendor_data_array[SENSOR_DATA_TYPE_COUNT + SENSOR_DATA_TYPE_FILTERED_ACC_NED_Z] = pKalmanEstimators[SENSOR_DATA_TYPE_COUNT + SENSOR_DATA_TYPE_FILTERED_ACC_NED_Z]->updateEstimate(ned_acc[2]);





		ned_acc_diff[0] = (ned_acc[0] - previos_ned_acc[0]) / 100;
		ned_acc_diff[1] = (ned_acc[1] - previos_ned_acc[1]) / 100;
		ned_acc_diff[2] = (ned_acc[2] - previos_ned_acc[2]) / 100;

		for (int i = 0; i < 100; i++) {

			
			ned_abs_vel_3d[0] += (previos_ned_acc[0] + ned_acc_diff[0]) * 1e-4;
			ned_abs_vel_3d[1] += (previos_ned_acc[1] + ned_acc_diff[1]) * 1e-4;
			ned_abs_vel_3d[2] += (previos_ned_acc[2] + ned_acc_diff[2] - BNOCONTROL_SENSOR_GRAVITY_VALUE) * 1e-4;
			
			ned_abs_pos_3d[0] += ned_abs_vel_3d[0] * 1e-3;
			ned_abs_pos_3d[1] += ned_abs_vel_3d[1] * 1e-3;
			ned_abs_pos_3d[2] += ned_abs_vel_3d[2] * 1e-3;


		}



		previos_ned_acc[0] = ned_acc[0];
		previos_ned_acc[1] = ned_acc[1];
		previos_ned_acc[2] = ned_acc[2];

		/*
		ned_abs_vel_3d[0] += ned_acc[0] * 1e-2;
		ned_abs_vel_3d[1] += ned_acc[1] * 1e-2;
		ned_abs_vel_3d[2] += (ned_acc[2] - BNOCONTROL_SENSOR_GRAVITY_VALUE) * 1e-2;


		ned_abs_pos_3d[0] += ned_abs_vel_3d[0] ;
		ned_abs_pos_3d[1] += ned_abs_vel_3d[1] ;
		ned_abs_pos_3d[2] += ned_abs_vel_3d[2] * 1e-1;


		*/
		sendor_data_array[SENSOR_DATA_TYPE_COUNT + SENSOR_DATA_TYPE_ABSOLUTE_VELOCITY_NED_X] = ned_abs_vel_3d[0];
		sendor_data_array[SENSOR_DATA_TYPE_COUNT + SENSOR_DATA_TYPE_ABSOLUTE_VELOCITY_NED_Y] = ned_abs_vel_3d[1];
		sendor_data_array[SENSOR_DATA_TYPE_COUNT + SENSOR_DATA_TYPE_ABSOLUTE_VELOCITY_NED_Z] = ned_abs_vel_3d[2];

		sendor_data_array[SENSOR_DATA_TYPE_COUNT + SENSOR_DATA_TYPE_ABSOLUTE_POSITION_NED_X] = ned_abs_pos_3d[0];
		sendor_data_array[SENSOR_DATA_TYPE_COUNT + SENSOR_DATA_TYPE_ABSOLUTE_POSITION_NED_Y] = ned_abs_pos_3d[1];
		sendor_data_array[SENSOR_DATA_TYPE_COUNT + SENSOR_DATA_TYPE_ABSOLUTE_POSITION_NED_Z] = ned_abs_pos_3d[2];



	}
	void AddAdditionalData(float timestamp, ViewSerialEvent event, array<float> ^sendor_data_array)
	{
		static float last_time_stamp = 0;
		float time_diff_ms = (event.pEvent->timestamp - last_time_stamp) ;
		lTimeDiffMs->Text = time_diff_ms.ToString();;
		last_time_stamp = event.pEvent->timestamp;

		sendor_data_array[SENSOR_DATA_TYPE_COUNT + SENSOR_DATA_TYPE_TIME_DIFF] = time_diff_ms;

		/* kalmanise acc */

	//	event.pEvent->acc[0] = pKalmanEstimators[3]->updateEstimate(event.pEvent->acc[0]);
	//	event.pEvent->acc[1] = pKalmanEstimators[4]->updateEstimate(event.pEvent->acc[1]);
	//	event.pEvent->acc[2] = pKalmanEstimators[5]->updateEstimate(event.pEvent->acc[2]);

		sendor_data_array[3] = event.pEvent->acc[0];
		sendor_data_array[4] = event.pEvent->acc[1];
		sendor_data_array[5] = event.pEvent->acc[2];

		/* Calibration Components*/
		sendor_data_array[SENSOR_DATA_TYPE_COUNT + SENSOR_DATA_TYPE_CALIBRATION_STATUS_SYSTEM ] = event.pEvent->calibration.sys;
		sendor_data_array[SENSOR_DATA_TYPE_COUNT + SENSOR_DATA_TYPE_CALIBRATION_STATUS_GYRO]	= event.pEvent->calibration.gyro;
		sendor_data_array[SENSOR_DATA_TYPE_COUNT + SENSOR_DATA_TYPE_CALIBRATION_STATUS_ACC]		= event.pEvent->calibration.acc;
		sendor_data_array[SENSOR_DATA_TYPE_COUNT + SENSOR_DATA_TYPE_CALIBRATION_STATUS_MAG]		= event.pEvent->calibration.mag;
#if 0
		/* OKUBENI
		BURADA RPY'ni el ile ve Madgwick ile hesaplamalari var. 
		Su an icin ihtiyac olmadigindan dolayi kod kapatildi
		
		*/
		/* Theta, Phi and Psi */
		static float thetaTilt = 0;
		static float phi = 0;
		static float CompConstant = 0.98;
		static float lastTimestamp = timestamp + 1000000;
		

		float time_const = (timestamp - lastTimestamp) / 1000000;
		thetaTilt	= CompConstant * (thetaTilt -  (time_const * event.pEvent->gyro[1]))	+ (1 - CompConstant)* (Math::Atan2(event.pEvent->acc[0] , event.pEvent->acc[2]));
		phi			= CompConstant * (phi - (time_const * event.pEvent->gyro[0]))			+ (1 - CompConstant)* (-Math::Atan2(event.pEvent->acc[1], event.pEvent->acc[2]));

		lastTimestamp = timestamp;

		phi = event.pEvent->euler[2] * DEGREE_TO_RADIAN_CONSTANT;
		thetaTilt = event.pEvent->euler[1] * DEGREE_TO_RADIAN_CONSTANT;

		float magY = event.pEvent->magneto[1] * Math::Cos(phi) + event.pEvent->magneto[2] * Math::Sin(phi) ;
		float magX = event.pEvent->magneto[0] * Math::Cos(thetaTilt) + event.pEvent->magneto[1] * Math::Sin(thetaTilt) * Math::Sin(phi) - event.pEvent->magneto[2] * Math::Cos(phi) * Math::Sin(thetaTilt);
		float psi = Math::Atan2(magY, magX );

		sendor_data_array[SENSOR_DATA_TYPE_COUNT + SENSOR_DATA_TYPE_CALCULATED_TILT]	= thetaTilt * (1/ DEGREE_TO_RADIAN_CONSTANT);
		sendor_data_array[SENSOR_DATA_TYPE_COUNT + SENSOR_DATA_TYPE_CALCULATED_PHI]		= phi		  * (1/ DEGREE_TO_RADIAN_CONSTANT);
		sendor_data_array[SENSOR_DATA_TYPE_COUNT + SENSOR_DATA_TYPE_CALCULATED_PSI]		= Math::Abs(psi	* (1 / DEGREE_TO_RADIAN_CONSTANT) - 180);


		/* Madgwick Quaternion*/
		
		pMadgwick->update(	event.pEvent->gyro[0],		event.pEvent->gyro[1],		event.pEvent->gyro[2],
							event.pEvent->acc[0]/9.8,		event.pEvent->acc[1] / 9.8,		event.pEvent->acc[2] / 9.8,
							event.pEvent->magneto[0],	event.pEvent->magneto[1],	event.pEvent->magneto[2]);
		/*
		pMadgwick->update(event.pEvent->gyro[0], event.pEvent->gyro[1], event.pEvent->gyro[2],
			event.pEvent->acc[0] , event.pEvent->acc[1] , event.pEvent->acc[2] ,
			0, 0, 0);
			*/

		psi = pMadgwick->getYaw();
		phi = pMadgwick->getRoll();
		thetaTilt = pMadgwick->getPitch();

		sendor_data_array[SENSOR_DATA_TYPE_COUNT + SENSOR_DATA_TYPE_CALCULATED_MADGWICK_YAW]	= psi;
		sendor_data_array[SENSOR_DATA_TYPE_COUNT + SENSOR_DATA_TYPE_CALCULATED_MADGWICK_PITCH]	= -thetaTilt;
		sendor_data_array[SENSOR_DATA_TYPE_COUNT + SENSOR_DATA_TYPE_CALCULATED_MADGWICK_ROLL]	= -phi;

		float madg_quat[4];
		pMadgwick->getQuat(madg_quat);
#endif

	}

	void CalculateSensorRelativeAcc(float timestamp, ViewSerialEvent event, array<float> ^sendor_data_array)
	{
		vector_ijk body_acc, down_acc;
		body_acc.a = event.pEvent->acc[0];
		body_acc.b = event.pEvent->acc[1];
		body_acc.c = event.pEvent->acc[2];

		Quaternion rotation_quat;
		rotation_quat.a = event.pEvent->quat[0];
		rotation_quat.b = 0;
		rotation_quat.c = 0;
		rotation_quat.d = event.pEvent->quat[3];

		rotation_quat = quaternion_normalize(rotation_quat);



		down_acc = quaternion_rotate_vector(body_acc, rotation_quat);

		float down_filtered = pKalmanEstimators[3]->updateEstimate(down_acc.c);

		sendor_data_array[SENSOR_DATA_TYPE_COUNT + SENSOR_DATA_TYPE_ROLL_AND_PITCH_COMPANSED_DOWN_ACC] = down_filtered;
	}
	void PreprocessSensorData(float timestamp, ViewSerialEvent event)
	{
		/* Gyro degree to radian*/
		event.pEvent->gyro[0] *= DEGREE_TO_RADIAN_CONSTANT;
		event.pEvent->gyro[1] *= DEGREE_TO_RADIAN_CONSTANT;
		event.pEvent->gyro[2] *= DEGREE_TO_RADIAN_CONSTANT;


	}
	void DisplayCalibrationStatus(TypeCalibration calib)
	{
		tbSystemCal->Text	= Convert::ToString(calib.sys);
		gbGyroCal->Text		= Convert::ToString(calib.gyro);
		tbAccCal->Text		= Convert::ToString(calib.acc);
		tbMagCal->Text		= Convert::ToString(calib.mag);
	}
	void AddPoints(float timestamp, array<float> ^sendor_data_array)
	{
		if (AddDataPointsFlag) {

			DataComponent ^p_component;
			timestamp = (uint32_t)timestamp / 1000;

			for (int i = 0; i < clbDataComponents->Items->Count; i++)
			{
				if (sensorFlowChart->Series[i]->Points->Count > MaxSampleSize) {

					sensorFlowChart->Series[i]->Points->RemoveAt(0);
				}

				p_component = (DataComponent ^)clbDataComponents->Items[i];
				sensorFlowChart->Series[i]->Points->AddXY(timestamp, sendor_data_array[p_component->Offset]);


			}
			sensorFlowChart->ResetAutoValues();
		}

		
	}

	void InitDataComponents()
	{
		int index = 0;
		for each(DataComponent ^component in clbDataComponents->Items)
		{
			DataVisualization::Charting::Series ^series = sensorFlowChart->Series->Add(component->Name);
			series->ChartType = DataVisualization::Charting::SeriesChartType::FastLine;
			series->BorderWidth = 2;
			series->YValuesPerPoint = 1;
			series->ToolTip = "#VALX #VAL{0.0}";

			if (clbDataComponents->GetItemChecked(index) == false) {
				series->Enabled = false;
			}
			
			index++;
		}
	}
	private: System::Void groupBox1_Enter(System::Object^  sender, System::EventArgs^  e) {
	}

		void AppendSensorDataToFile(uint32_t timestamp, ViewSerialEvent p_event, array<float> ^p_events_array)
		{
			if (nullptr != pTextWriter) {
				pTextWriter->Write(p_event.GetEvent()->timestamp + "\t");

				pTextWriter->Write(p_event.GetEvent()->euler[0] + "\t");
				pTextWriter->Write(p_event.GetEvent()->euler[1] + "\t");
				pTextWriter->Write(p_event.GetEvent()->euler[2] + "\t");

				pTextWriter->Write(p_event.GetEvent()->acc[0] + "\t");
				pTextWriter->Write(p_event.GetEvent()->acc[1] + "\t");
				pTextWriter->Write(p_event.GetEvent()->acc[2] + "\t");

				pTextWriter->Write(p_event.GetEvent()->gyro[0] + "\t");
				pTextWriter->Write(p_event.GetEvent()->gyro[1] + "\t");
				pTextWriter->Write(p_event.GetEvent()->gyro[2] + "\t");

				pTextWriter->Write(p_event.GetEvent()->magneto[0] + "\t");
				pTextWriter->Write(p_event.GetEvent()->magneto[1] + "\t");
				pTextWriter->Write(p_event.GetEvent()->magneto[2] + "\t");



				pTextWriter->Write(p_event.GetEvent()->quat[0] + "\t");
				pTextWriter->Write(p_event.GetEvent()->quat[1] + "\t");
				pTextWriter->Write(p_event.GetEvent()->quat[2] + "\t");
				pTextWriter->Write(p_event.GetEvent()->quat[3] + "\t");


				/* 20 Kayit*/

				
				for (int i = SENSOR_DATA_TYPE_COUNT; i < (SENSOR_DATA_TYPE_COUNT + CUSTOM_SENSOR_DATA_TYPE_COUNT); i++) 
				{
					pTextWriter->Write(p_events_array[i] + "\t");
				}

				pTextWriter->WriteLine();

				pTextWriter->Flush();
			}
		}


		System::Void SerialThreadDelegate(ViewSerialEvent p_event)
		{
			if ((36 != p_event.GetEvent()->version ||
				55 != p_event.GetEvent()->sensor_id ||
				3 != p_event.GetEvent()->type ||
				p_event.GetEvent()->timestamp < 0)) {
				tbInvalidMessage->Text = Convert::ToString(Convert::ToInt32(tbInvalidMessage->Text) + 1);
			}
			else {
				message_count++;
				lTotalMessageCount->Text = Convert::ToString(message_count);
				tbVersion->Text = Convert::ToString(p_event.GetEvent()->version);
				tbSensorID->Text = Convert::ToString(p_event.GetEvent()->sensor_id);
				tbType->Text = Convert::ToString(p_event.GetEvent()->type);
				tbTimestamp->Text = Convert::ToString(Convert::ToSingle(p_event.GetEvent()->timestamp));
				ViewSerialEvent last_event;

					

			
					

				array<float> ^sendor_data_array = gcnew array<float>(SENSOR_DATA_TYPE_COUNT + CUSTOM_SENSOR_DATA_TYPE_COUNT);
				pin_ptr<float> pinPtr = &sendor_data_array[0];
				memcpy(pinPtr, &p_event.pEvent->euler[0], sizeof(float) * SENSOR_DATA_TYPE_COUNT);



				ReflectSensorValuesAsText(p_event);

					
					
					

				PreprocessSensorData(p_event.pEvent->timestamp, p_event);

				AddAdditionalData(p_event.pEvent->timestamp, p_event, sendor_data_array);

				CalculateSensorRelativeAcc(p_event.pEvent->timestamp, p_event, sendor_data_array);

				

					

					

				UpdateVelocityAndPosition(p_event.pEvent->timestamp, p_event, sendor_data_array);

				UpdateRelativeVelocityAndPosition(p_event.pEvent->timestamp, pActionArrayBuffer, sendor_data_array);

				CalculateQuaternionDistance(p_event, sendor_data_array);
					
				DirectionCheckByQuaternions(p_event, sendor_data_array);


				AddDiffData(sendor_data_array);

				AddZeroCrossings(sendor_data_array);

				CalculateMeanAndStd(p_event.pEvent->timestamp, pActionArrayBuffer, sendor_data_array);


				UpdateJoystick();

				pActionArrayBuffer->Push(sendor_data_array);

				if (pActionArrayBuffer->GetCount() >ACTION_BUFFER_ARRAY_LENGTH) {
					pActionArrayBuffer->Pop();
				}

				

				if (true == SamplingInProgress) {
					AppendSensorDataToFile(p_event.pEvent->timestamp, p_event, sendor_data_array);
				}

				PatternMatch(p_event, sendor_data_array);

				//PatternMatch(p_event);

				AccOffsetCalculation(p_event);

				

				AddPoints(p_event.pEvent->timestamp, sendor_data_array);

				DisplayCalibrationStatus(p_event.pEvent->calibration);


				Array::Copy(sendor_data_array, 0, PreviousSensorDataArray, 0, sendor_data_array->Length);
				

				array<unsigned char> ^p_array = gcnew array<unsigned char>(sendor_data_array->Length * 4);
				Buffer::BlockCopy(sendor_data_array, 0, p_array, 0, p_array->Length);
					
				pUDPClient->Send(p_array, p_array->Length, "localhost", 12455);


				if (IMUNavigationReset) {
					counter = 0;
					ned_abs_vel_3d[0] = 0;
					ned_abs_vel_3d[1] = 0;
					ned_abs_vel_3d[2] = 0;

					ned_abs_pos_3d[0] = 0;
					ned_abs_pos_3d[1] = 0;
					ned_abs_pos_3d[2] = 0;

					IMUNavigationReset = false;
				}

			}

		}

		void AddZeroCrossings(array<float> ^sendor_data_array)
		{
			static float previous_acc_down = 0;

			float acc_down = sendor_data_array[SENSOR_DATA_TYPE_COUNT + SENSOR_DATA_TYPE_ROLL_AND_PITCH_COMPANSED_DOWN_ACC];
			float temp_acc_down = acc_down;
			float acc_zown_crossing = acc_down - previous_acc_down;

			if (acc_zown_crossing > 0.2) {

				acc_down = 1;
			}else if (acc_zown_crossing < -0.2) {

				acc_down = -1;
			}
			else {

				acc_down = 0.001;
			}

			previous_acc_down = temp_acc_down;

			sendor_data_array[SENSOR_DATA_TYPE_COUNT + SENSOR_DATA_TYPE_TEMP_ACC_DOWN_DIFF_CROSSING] = acc_down;

		}


		void AddDiffData(array<float> ^sendor_data_array)
		{
			float euler2_diff = GetDiffOfData(1, sendor_data_array);
			float euler3_diff = GetDiffOfData(2, sendor_data_array);
			float acc_down_diff = GetDiffOfData(SENSOR_DATA_TYPE_COUNT + SENSOR_DATA_TYPE_ROLL_AND_PITCH_COMPANSED_DOWN_ACC, sendor_data_array);


			sendor_data_array[SENSOR_DATA_TYPE_COUNT + SENSOR_DATA_TYPE_TEMP_EULER_2_DIFF] = pKalmanEstimators[1]->updateEstimate(euler2_diff);
			sendor_data_array[SENSOR_DATA_TYPE_COUNT + SENSOR_DATA_TYPE_TEMP_EULER_3_DIFF] = pKalmanEstimators[2]->updateEstimate(euler3_diff);
			sendor_data_array[SENSOR_DATA_TYPE_COUNT + SENSOR_DATA_TYPE_TEMP_ACC_DOWN_DIFF] = acc_down_diff;
			
		}


		float GetDiffOfData(unsigned int sdi, array<float> ^current_sensor_data)
		{
			return current_sensor_data[sdi] - PreviousSensorDataArray[sdi];

		}
			void BodyToNedAcc(float quat[4], float acc[3], float ned_acc[3])
			{


				vector_ijk vector_acc, vector_ned;
				vector_acc.a = acc[0];
				vector_acc.b = acc[1];
				vector_acc.c = acc[2];

				vector_ned = quaternion_rotate_vector(vector_acc, *(Quaternion *)quat);
				ned_acc[0] = vector_ned.a;
				ned_acc[1] = vector_ned.b;
				ned_acc[2] = vector_ned.c;
#if 0
				ned_acc[0] = (1 - 2 * (q[2]*q[2] + q[3]*q[3])) * acc[0] + (2 * (q[1]*q[2] + q[0]*q[3]))*acc[1] + (2 * (q[1]*q[3] - q[0]*q[2]))*acc[2];  // rotate linearaccel by quaternion
				ned_acc[1] = (2 * (q[1]*q[2] - q[0]*q[3]))*acc[0] + (1 - 2 * (q[1]*q[1] + q[3]*q[3]))*acc[1] + (2 * (q[2]*q[3] + q[0]*q[1]))*acc[2];
				ned_acc[2] = (2 * (q[1]*q[3] + q[0]*q[2]))*acc[0] + (2 * (q[2]*q[3] - q[0]*q[1]))*acc[1] + (1 - 2 * (q[1]*q[1] + q[2]*q[2]))*acc[2];
#endif
/*
				acc[0] = (1 - 2 * (quat.y()*quat.y() + quat.z()*quat.z()))*linearaccel[0] + (2 * (quat.x()*quat.y() + quat.w()*quat.z()))*linearaccel[1] + (2 * (quat.x()*quat.z() - quat.w()*quat.y()))*linearaccel[2];  // rotate linearaccel by quaternion
				acc[1] = (2 * (quat.x()*quat.y() - quat.w()*quat.z()))*linearaccel[0] + (1 - 2 * (quat.x()*quat.x() + quat.z()*quat.z()))*linearaccel[1] + (2 * (quat.y()*quat.z() + quat.w()*quat.x()))*linearaccel[2];
				acc[2] = (2 * (quat.x()*quat.z() + quat.w()*quat.y()))*linearaccel[0] + (2 * (quat.y()*quat.z() - quat.w()*quat.x()))*linearaccel[1] + (1 - 2 * (quat.x()*quat.x() + quat.y()*quat.y()))*linearaccel[2];
*/

			}
			System::Void ReflectSensorValuesAsText(ViewSerialEvent p_event)
			{
				tbAccX->Text = Convert::ToString(p_event.GetEvent()->acc[0]);
				tbAccY->Text = Convert::ToString(p_event.GetEvent()->acc[1]);
				tbAccZ->Text = Convert::ToString(p_event.GetEvent()->acc[2]);

				orientX->Text = Convert::ToString(p_event.GetEvent()->euler[0]);
				orientY->Text = Convert::ToString(p_event.GetEvent()->euler[1]);
				orientZ->Text = Convert::ToString(p_event.GetEvent()->euler[2]);

				tbMagneticX->Text = Convert::ToString(p_event.GetEvent()->magneto[0]);
				tbMagneticY->Text = Convert::ToString(p_event.GetEvent()->magneto[1]);
				tbMagneticZ->Text = Convert::ToString(p_event.GetEvent()->magneto[2]);

				tbGyroX->Text = Convert::ToString(p_event.GetEvent()->gyro[0]);
				tbGyroY->Text = Convert::ToString(p_event.GetEvent()->gyro[1]);
				tbGyroZ->Text = Convert::ToString(p_event.GetEvent()->gyro[2]);


				tbQuatW->Text = Convert::ToString(p_event.GetEvent()->quat[0]);
				tbQuatX->Text = Convert::ToString(p_event.GetEvent()->quat[1]);
				tbQuatY->Text = Convert::ToString(p_event.GetEvent()->quat[2]);
				tbQuatZ->Text = Convert::ToString(p_event.GetEvent()->quat[3]);
			}
			void AccOffsetCalculation(ViewSerialEvent p_event)
			{
				
				
				if (AccOffsetCalcInProgress) {

					total_acc_x += p_event.pEvent->acc[0];
					total_acc_y += p_event.pEvent->acc[1];
					total_acc_z += p_event.pEvent->acc[2];
					offset_counter++;
				}
			}
			private:	void PatternMatch(ViewSerialEvent p_event, array<float> ^p_events_array)
			{
				array<array<float> ^> ^p_last_events = pActionArrayBuffer->GetQueue()->ToArray();

				boolean action_detected = false;
				double distance = 0;

				for each(KeyValuePair<unsigned int, ActionDefinition ^> ^kvp in pActions) {

					distance = PatternMatcher::Match(kvp->Value, p_last_events);

					p_events_array[SENSOR_DATA_TYPE_COUNT + SENSOR_DATA_TYPE_DISTANCE_ACTION_0 + kvp->Key] = distance;
					if (distance < kvp->Value->GetThreshold())
					{
						kvp->Value->LastConsequtiveMatches++;

						if (kvp->Value->LastConsequtiveMatches < 5) {

							pJoyButtonList[kvp->Key]->BackColor = System::Drawing::Color::Green;
							pJoyButtons[kvp->Key] = true;
						}
						else {
							pJoyButtonList[kvp->Key]->BackColor = System::Drawing::Color::Gray;
							pJoyButtons[kvp->Key] = false;
							kvp->Value->LastConsequtiveMatches = 0;
						}
						

						
						
					}
					else {
						pJoyButtonList[kvp->Key]->BackColor = System::Drawing::Color::Gray;
						pJoyButtons[kvp->Key] = false;

						kvp->Value->LastConsequtiveMatches = 0;
					}

					
					SaveDirIndex++;
				}
				FileSave = false;
			}

private: System::Void MainForm_Shown(System::Object^  sender, System::EventArgs^  e) {

	}
	private: System::Void groupBox1_Enter_1(System::Object^  sender, System::EventArgs^  e) {
	}
	private: System::Void button1_Click(System::Object^  sender, System::EventArgs^  e) {

		for each(DataVisualization::Charting::Series ^series in sensorFlowChart->Series)
		{
			series->Points->Clear();
		}

		p_velocity_plotter->ClearPoints();
		p_position_plotter->ClearPoints();
		p_mag_plotter->ClearPoints();

		return;

	}

private: System::Void timerSerialRead_Tick(System::Object^  sender, System::EventArgs^  e) {

	int read_count;
	int end_marker_temp = 0;
	static int a = 1;
	bool input_marker_goes_ahead = false;
	S_BNO055_SENSOR_SAMPLE sensor_sample;
	
	int available_buffer = ((4096 - SerialBufferDataInputMarker) >= 512) ? 512 : (4096 - SerialBufferDataInputMarker - 1); // sondaki -1 'i unutma! Yoksa günlerini debugla harcarsin

	if (available_buffer != 512) {
		
		BufferSizeDecreased++;
		
		lBufferSizeDecreased->Text = BufferSizeDecreased.ToString();
		a++;
		if (0 == available_buffer) {

			SerialBufferDataInputMarker = 0;
			SerialBufferDataReadMarker = 0;
			available_buffer = 512;
			
		}
	}
	lSerialBufferReadSize->Text = available_buffer.ToString();
	if (!serialPortBNO055->IsOpen) {
		return;
	}
	read_count = serialPortBNO055->Read(SerialReadBuffer, SerialBufferDataInputMarker, available_buffer);


	TotalReadBytesCount += read_count;

	SerialBufferDataInputMarker = (SerialBufferDataInputMarker + read_count)%4096;

	


	if (false == input_marker_goes_ahead) {

		for (int i = SerialBufferDataReadMarker; i <= SerialBufferDataInputMarker; i++)
		{
			/* Baslangici bul */
			if (SerialReadBuffer[i] == 0x10 && SerialReadBuffer[i + 1] == 0x10 &&
				SerialReadBuffer[i + 2] == 0x10 && SerialReadBuffer[i + 3] == 0x10)
			{
				pin_ptr<unsigned char>	pin_ptr = &SerialReadBuffer[0];

				end_marker_temp = i + sizeof(S_BNO055_SENSOR_SAMPLE);
				

				if (end_marker_temp >= 4096) {

					memcpy(&pin_ptr[0], &pin_ptr[i], (SerialBufferDataInputMarker - i));
					SerialBufferDataInputMarker = (SerialBufferDataInputMarker - i);
					SerialBufferDataReadMarker = 0;
					BufferReset++;
					lBufferReset->Text = BufferReset.ToString();
					break;
				}
				else if (end_marker_temp > SerialBufferDataInputMarker) {
					a++;
					break;
				}
				else {
					a++;
				}
				end_marker_temp -= 8;

				/* Bitisi bul*/
				if (SerialReadBuffer[end_marker_temp] == 0x01 && SerialReadBuffer[end_marker_temp + 1] == 0x01
					&& SerialReadBuffer[end_marker_temp + 2] == 0x01 && SerialReadBuffer[end_marker_temp + 3] == 0x01)
				{
					//Data valid (checksum kontrolü de eklenebilir). O halde okuyalım

					S_BNO055_SENSOR_SAMPLE *p_sensor_data = (S_BNO055_SENSOR_SAMPLE *)&pin_ptr[i];
					ViewSerialEvent event = ViewSerialEvent(p_sensor_data);

					SerialThreadDelegate(event);

					SerialBufferDataReadMarker = i + sizeof(S_BNO055_SENSOR_SAMPLE);
				}

				if (end_marker_temp >= SerialBufferDataInputMarker) {
					a++;
				}
			}

		}
	}



	lTotalReceivedBytes->Text = TotalReadBytesCount.ToString();
	int d = sizeof(S_BNO055_SENSOR_SAMPLE);
	int c = TotalReadBytesCount - (d * message_count);
	lDifferenceBetweenReceivedBytes->Text = ((float)c/sizeof(S_BNO055_SENSOR_SAMPLE)).ToString();
	
	}
public:
		 virtual void OnActionRecordStarted()
		 {
			 SamplingInProgress = true;
			 this->panel1->BackColor = System::Drawing::Color::Green;
			 lWorkInProgress->Text = "Eğitim örneği kaydediliyor...";


			 p_acc_plotter->ClearPoints();
		 }

		 virtual void OnActionRecordEnded()
		 {
			 SamplingInProgress = false;
			 this->panel1->BackColor = System::Drawing::Color::Crimson;
			 lWorkInProgress->Text = "Eğitim örneği kaydedildi";
		 }

		 virtual void EndActionRecord()
		 {
			 throw gcnew System::NotImplementedException();
		 }
public:


private: System::Void bLimitlessDataFlow_Click(System::Object^  sender, System::EventArgs^  e) {
	
	tTimeToNextAction->Enabled = false;
	
	CreateActionSampleFile(E_ACTION_ZERO);
	pTextWriter->Flush();
	OnActionRecordStarted();
}
private: System::Void tTimeToNextAction_Tick_1(System::Object^  sender, System::EventArgs^  e) 
	{
		if (RemainingTimeMsToNextAction <= 0) {

			if (nullptr != pTextWriter) {
				pTextWriter->Close();
			}

			lCurrentAction->Text = lNextAction->Text;
			RemainingTimeMsToNextAction = pRandom->Next(2, 3) * 1000;
			E_ACTION_TYPES action = GetRandomAction();
			lNextAction->Text = ((unsigned int)action).ToString();

			ElapsedTimeMsSinceBeginning = 1200;

			CreateActionSampleFile((E_ACTION_TYPES)Convert::ToInt32(lCurrentAction->Text));

			OnActionRecordStarted();
		}
		else {
			RemainingTimeMsToNextAction -= tTimeToNextAction->Interval;
			lRemainingTimeToNextAction->Text = RemainingTimeMsToNextAction.ToString() + "ms";

			ElapsedTimeMsSinceBeginning -= tTimeToNextAction->Interval;
			if (ElapsedTimeMsSinceBeginning <= 0) {

				ElapsedTimeMsSinceBeginning = 0x7FFFFFFF;
				OnActionRecordEnded();
			}
		}
	}
private: System::Void bToggleTraining_Click(System::Object^  sender, System::EventArgs^  e) {
		bool is_timer_enabled = tTimeToNextAction->Enabled;
		if (true == is_timer_enabled) {
			tTimeToNextAction->Enabled = false;
			groupBox6->Text = "Eğitim (Durdu)";
		}else {
			tTimeToNextAction->Enabled = true;
			groupBox6->Text = "Eğitim (Aktif)";

		
		}
	}

	void CreateActionSampleFile(E_ACTION_TYPES action_type)
	{
		String ^dir = gcnew String(Environment::CurrentDirectory + "\\SensorData\\" + ((int)action_type).ToString());
		if (!IO::Directory::Exists(dir)) {

			IO::Directory::CreateDirectory(dir);
		}
		if (IO::Directory::Exists(dir)) {
			for (int a = 1; a < 0xFFFFFFFF; a++) {
				String ^path = gcnew String(dir + "\\motion_data_" + a + ".txt");
				if (!IO::File::Exists(path)) {

					pTextWriter = gcnew System::IO::StreamWriter(path, true);
					pTextWriter->Flush();

					break;
				}
			}
		}
	}
private: System::Void bOpenTrainingDirectory_Click(System::Object^  sender, System::EventArgs^  e) {
	System::Diagnostics::Process::Start(Environment::CurrentDirectory + "\\SensorData\\");
}

private: System::Void button2_Click(System::Object^  sender, System::EventArgs^  e) {
	PatternLoader ^loader = gcnew PatternLoader();
	loader->SetSensorTypes(clbDataComponents->Items);
	loader->SetMainFormOps(this);
	loader->Show();
}
private: System::Void bSetAccOffset_Click(System::Object^  sender, System::EventArgs^  e) {

	
	acc_offset[0] = total_acc_x/ offset_counter;
	acc_offset[1] = total_acc_y/ offset_counter;
	acc_offset[2] = total_acc_z/ offset_counter;

	total_acc_x = 0;
	total_acc_y = 0;
	total_acc_z = 0;
	offset_counter = 0;
}
private:

	void OpenTK3DInit()
	{
		/*
		OpenTK3D ^opentk3d = gcnew OpenTK3D(400, 400, "Selam Dünya");
		opentk3d->Run();
		 */
		
	}
private:
	Diagnostics::Process ^ GnuPlotInit()
	{
#if 1
		FILE *gp;
		if ((gp = _popen("C:\\Program Files (x86)\\gnuplot\\bin\\wgnuplotgg.exe -persist", "w")) == NULL) {
			return nullptr;
		}
		
		fprintf(gp, "plot '-' with lines title 'sin(x)");
		fprintf(gp, "3 3");
		fprintf(gp, "4 4");
		fprintf(gp, "5 5");
		fprintf(gp, "6 4");
		fflush(gp);
		fprintf(gp, "e");
		fflush(gp);
		

#endif
#if 0
		String ^path = "C:\\Program Files (x86)\\gnuplot\\bin\\wgnuplot_pipes.exe";
		Diagnostics::Process ^GnuplotProcess = gcnew Diagnostics::Process();
		GnuplotProcess->StartInfo->FileName = path;
		GnuplotProcess->StartInfo->UseShellExecute = false;
		GnuplotProcess->StartInfo->RedirectStandardInput = true;
		GnuplotProcess->StartInfo->RedirectStandardOutput = true;
		GnuplotProcess->Start();

		GnuPlotSW = GnuplotProcess->StandardInput;
		GnuPlotSR = GnuplotProcess->StandardOutput;

	//	GnuPlotSW->WriteLine("set xrange[-5:15]");


		GnuPlotSW->WriteLine("set term qt nopersist size 700,500");
		GnuPlotSW->WriteLine("plot '-' with lines title 'sin(x)");
		GnuPlotSW->WriteLine("3 3");
		GnuPlotSW->WriteLine("4 4");
		GnuPlotSW->WriteLine("5 5");
		GnuPlotSW->WriteLine("6 4");
		
		GnuPlotSW->WriteLine("7 4");
		GnuPlotSW->WriteLine("-");
		GnuPlotSW->WriteLine("replot");
		GnuPlotSW->WriteLine("8 4");
		GnuPlotSW->WriteLine("9 4");
		GnuPlotSW->WriteLine("e");
#endif
		/*
		GnuPlotSW->WriteLine("plot '-' with lines title 'sin(x)");
		GnuPlotSW->WriteLine("8 4");
		GnuPlotSW->WriteLine("e");*/

	//	GnuPlotSW->WriteLine("f(x) = x**2");
	//	GnuPlotSW->WriteLine("plot f(x), '-' with points");


		return nullptr;
	}
private: System::Void clbDataComponents_ItemCheck(System::Object^  sender, System::Windows::Forms::ItemCheckEventArgs^  e) {
	DataComponent ^p_component = (DataComponent ^)clbDataComponents->Items[e->Index];

	pUISettingsSaver->SaveDataSourceSelectedState(p_component->Name, (System::Windows::Forms::CheckState::Checked == e->NewValue));
	if (nullptr != sensorFlowChart->Series && sensorFlowChart->Series->Count > 0) {
		sensorFlowChart->Series[e->Index]->Enabled = (System::Windows::Forms::CheckState::Checked == e->NewValue);
	}

}
private: System::Void tbChartMin_TextChanged(System::Object^  sender, System::EventArgs^  e) {
	double min_val;
	if (Double::TryParse(((TextBox ^)sender)->Text, min_val)){
		if (min_val < sensorFlowChart->ChartAreas[0]->AxisY->Maximum) {
			sensorFlowChart->ChartAreas[0]->AxisY->Minimum = min_val;

			pUISettingsSaver->SaveChartAxis("min", min_val);
		}
	}
}
private: System::Void tbChartMax_TextChanged(System::Object^  sender, System::EventArgs^  e) {
	double max_val;
	if (Double::TryParse(((TextBox ^)sender)->Text, max_val)) {

		if (max_val > sensorFlowChart->ChartAreas[0]->AxisY->Minimum) {
			sensorFlowChart->ChartAreas[0]->AxisY->Maximum = max_val;
			pUISettingsSaver->SaveChartAxis("max", max_val);
		}
	}
}
private: System::Void tbRefreshVelPos_Click(System::Object^  sender, System::EventArgs^  e) 
{

}
private: System::Void tbQDistanceW_TextChanged(System::Object^  sender, System::EventArgs^  e) 
{
	boolean w_valid, x_valid, y_valid, z_valid;
	double w_val, x_val, y_val, z_val;
	Quaternion q;
	w_valid = Double::TryParse(tbQDistanceW->Text, w_val);
	x_valid = Double::TryParse(tbQDistanceX->Text, x_val);
	y_valid = Double::TryParse(tbQDistanceY->Text, y_val);
	z_valid = Double::TryParse(tbQDistanceZ->Text, z_val);

	if (w_valid && x_valid && y_valid && z_valid) {
		q = quaternion_initialize(w_val, x_val, y_val, z_val);
		q = quaternion_normalize(q);

		pRefQuaternion->a = q.a;
		pRefQuaternion->b = q.b;
		pRefQuaternion->c = q.c;
		pRefQuaternion->d = q.d;
	}

	



}
private: System::Void bOrientationSW_Click(System::Object^  sender, System::EventArgs^  e) {
}
private: System::Void tbQDistanceX_TextChanged(System::Object^  sender, System::EventArgs^  e) {
}
private: System::Void panel2_DoubleClick(System::Object^  sender, System::EventArgs^  e) 
{
	String ^ s_id = (String ^)((Control ^)sender)->Tag;
	int id = Convert::ToUInt32(s_id);

	pDirectionQuaternions[id].a = Convert::ToSingle(tbQuatW->Text);
	pDirectionQuaternions[id].b = Convert::ToSingle(tbQuatX->Text);
	pDirectionQuaternions[id].c = Convert::ToSingle(tbQuatY->Text);
	pDirectionQuaternions[id].d = Convert::ToSingle(tbQuatZ->Text);
	
	pUISettingsSaver->SaveDirectionQuaternion(id, pDirectionQuaternions[id].a, pDirectionQuaternions[id].b, pDirectionQuaternions[id].c, pDirectionQuaternions[id].d);



}
private: System::Void panel2_Paint(System::Object^  sender, System::Windows::Forms::PaintEventArgs^  e) {
}
private: System::Void button3_Click(System::Object^  sender, System::EventArgs^  e) 
{
	String ^ s_id = (String ^)((Control ^)sender)->Tag;
	int id = Convert::ToUInt32(s_id);
	Quaternion quaternion_standingpoint_yaw;
	Quaternion quaternion_reference_yaw = pDirectionQuaternions[4];

	quaternion_reference_yaw.a = quaternion_reference_yaw.a; // scalar part  correction for multiplication
	quaternion_reference_yaw.b = 0;
	quaternion_reference_yaw.c = 0;
	quaternion_reference_yaw = quaternion_normalize(quaternion_reference_yaw);

	
	pStandpointQuaternion->a = Convert::ToSingle(tbQuatW->Text);
	pStandpointQuaternion->b = 0;
	pStandpointQuaternion->c = 0;
	pStandpointQuaternion->d = Convert::ToSingle(tbQuatZ->Text);

	quaternion_standingpoint_yaw = quaternion_conjugate(quaternion_normalize(*pStandpointQuaternion));
	

	*pStandpointQuaternion = quaternion_product(quaternion_reference_yaw, quaternion_standingpoint_yaw);
	
}
private: System::Void bAlgorithmTest_Click(System::Object^  sender, System::EventArgs^  e) 
{
	BNOControl::TestMain ^p_algorithm_test_form = gcnew BNOControl::TestMain();

	p_algorithm_test_form->SetSensorTypes(clbDataComponents->Items);

	p_algorithm_test_form->Show();
}
private: System::Void pDirectionQuat1_Paint(System::Object^  sender, System::Windows::Forms::PaintEventArgs^  e) {
}
public: void FileSaveThresholdMethod(unsigned int target_action, bool is_save_enabled, double threshold)
{
	for each(KeyValuePair<unsigned int, ActionDefinition ^> ^kvp in pActions) {

		if (kvp->Key == (target_action)) {


			FileSaveThreshold = threshold;
		}

	}

}
public: void ExportActionPattern(unsigned int TargetAction)
{
	for each(KeyValuePair<unsigned int, ActionDefinition ^> ^kvp in pActions) {

		if (kvp->Key == (TargetAction)) {
			
			array<array<float> ^> ^p_org_pattern = kvp->Value->GetOriginalPattern();
			array<int> ^p_sdi = kvp->Value->GetSensorDataIndexs();
			String ^dir = gcnew String(Environment::CurrentDirectory + "\\SensorData\\Signals\\Action" + TargetAction.ToString());
			if (!IO::Directory::Exists(dir)) {

				IO::Directory::CreateDirectory(dir);
			}

			String ^path = gcnew String(dir + "\\saved_original_action.txt");

			System::IO::TextWriter ^pTextWriter;
			if (!IO::File::Exists(path)) {

				pTextWriter = gcnew System::IO::StreamWriter(path, true);
				pTextWriter->Flush();

			}
			unsigned int saved_action_length = p_org_pattern->Length;
			for (int j = 0; j < saved_action_length; j++) {
				pTextWriter->Write(p_org_pattern[j][0] + "\t");
			}
			pTextWriter->Flush();
			pTextWriter->Close();

		}
	}

}

public: void SetActionThreshold(unsigned int TargetAction, double threshold)
{
	for each(KeyValuePair<unsigned int, ActionDefinition ^> ^kvp in pActions) {

		if (kvp->Key == (TargetAction)) {
			kvp->Value->SetThreshold(threshold);
		}
	}

}

System::Void pJoyButton1_MouseDoubleClick(System::Object^  sender, System::Windows::Forms::MouseEventArgs^  e)
{
	unsigned int action = Convert::ToUInt32(((Control ^)sender)->Tag);
	BNOControl::ActionSettings ^p_action_settings = gcnew BNOControl::ActionSettings(action);
	p_action_settings->UpdateEvent += gcnew BNOControl::ActionSettings::UpdateEventDelegate(this, &BNOControl::MainForm::SetActionThreshold);
	p_action_settings->ExportActionPattern += gcnew BNOControl::ActionSettings::ExportActionPatternDelegate(this, &BNOControl::MainForm::ExportActionPattern);
	p_action_settings->FileSaveThresholdMethod += gcnew BNOControl::ActionSettings::FileSaveThresholdDelegate(this, &BNOControl::MainForm::FileSaveThresholdMethod);

	p_action_settings->Show();
}

private: System::Void pJoyButton1_Paint(System::Object^  sender, System::Windows::Forms::PaintEventArgs^  e) {
}
private: System::Void bSaveSnapshot_Click(System::Object^  sender, System::EventArgs^  e) 
{
	FileSave = true;
}
private: System::Void checkBox1_CheckedChanged(System::Object^  sender, System::EventArgs^  e) {

	if (AccOffsetCalcInProgress = cbAccOffset->Checked);
}
private: System::Void cbAddDataPointsFlag_CheckedChanged(System::Object^  sender, System::EventArgs^  e) {
	AddDataPointsFlag = cbAddDataPointsFlag->Checked;
}
};

}
