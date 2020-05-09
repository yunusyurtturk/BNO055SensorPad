#pragma once
#include "ViewTypes.h"
#include "COpChain.h"
#include "MSE3D.h"
#include "DTWDistance.h"
#include "MeanAndCenterOffMass.h"
#include "ISignalOp.h"
#include "Normalizer.h"

public ref class PatternDataValidity
{

};

public ref class ActionDefinition
{
private:
	String ^PatternName;
	array<array<float>^> ^pOriginalPattern;
	array<array<float>^> ^pNormalizedPattern;
	double Threshold;
	SignalProcessing::Operation ^pDistanceOp;

	array<BNOControl::Plotter ^> ^pPlotters;
	int OpChainCounter;
	array<int> ^SensorDataIndexs;
	array<double> ^CenterOffMasses;
	array<double> ^Means;
	unsigned int SensorDataCount;
	unsigned int PatternLength;

	unsigned int lastConsequtiveMatches;
public:
	ActionDefinition()
	{
		OpChainCounter = 0;
		Threshold = 1.5;
		PatternName = gcnew String("Unnamed Pattern");
		
		pPlotters = gcnew array<BNOControl::Plotter ^>(SENSOR_DATA_TYPE_COUNT + CUSTOM_SENSOR_DATA_TYPE_COUNT);
		
		PatternLength = 0;
		SensorDataCount = 0;
		LastConsequtiveMatches = 0;
	}

		property unsigned int LastConsequtiveMatches {
		unsigned int get() {
			return lastConsequtiveMatches;
		}
		void set(unsigned int val) {
			lastConsequtiveMatches = val;
		}
	}
	void Init()
	{
		if (SensorDataCount > 0) {
			CenterOffMasses = gcnew array<double>(SensorDataCount);
			Means = gcnew array<double>(SensorDataCount);
		}

		InitMeansAndCenterOfMasses();
	}
	void SetSensorDataIndexs(array<int> ^p_indexs)
	{
		SensorDataIndexs = p_indexs;
		SensorDataCount = p_indexs->Length;
	}
	array<int> ^ GetSensorDataIndexs()
	{
		return SensorDataIndexs;
	}
	void SetThreshold(double val)
	{	
		Threshold = val;
	}
	SignalProcessing::Operation ^ GetDistanceMethod()
	{
		return pDistanceOp;
	}
	void SetDistanceMethod(SignalProcessing::Operation ^p_distance_method)
	{
		pDistanceOp = p_distance_method;
	}
	void SetPattern(array<array<float> ^> ^p_pattern)
	{
		this->pOriginalPattern = p_pattern;
		PatternLength = p_pattern->Length;

		array<int> ^p_sdi = gcnew array<int>(SensorDataIndexs->Length);

		for (int i = 0; i < SensorDataIndexs->Length; i++) {

			p_sdi[i] = i;
		}

		pNormalizedPattern = SignalProcessing::Normalizer::Normalize(p_pattern, p_sdi);
	}
	
	double GetThreshold()
	{
		return Threshold;
	}
	void SetName(String ^p_val)
	{
		PatternName = p_val;
	}
	String ^GetName()
	{
		return PatternName;
	}
	
	void AddOpChain(SignalProcessing::OpChain ^p_chain)
	{
		
		OpChainCounter++;
	}



	
	array<array<float> ^> ^GetPattern()
	{
		return pNormalizedPattern;
	}
	
	void ShowCorrelation()
	{
		
	}
	double GetMean(int sdi) {
		return Means[sdi];
	}
	double GetCenterOffMass(int sdi) {
		return CenterOffMasses[sdi];
	}
private:
	
	void InitMeansAndCenterOfMasses()
	{
		double mean;
		array<array<float> ^>^p_pattern = GetPattern();
		unsigned int data_index = 0;
		unsigned int pattern_length = GetPattern()->Length;

		for (int i = 0; i < SensorDataCount; i++) {

			

			MeanAndCenterOffMass::Calculate(p_pattern, i, i, Means, CenterOffMasses);
#if 0
			for (int sample = 0; (sample < pattern_length && sample < pattern_length/2); sample++) {

				mean = mean + p_pattern[sample][i];
				mean = mean + p_pattern[sample - sample - 1][i];

				cumulative_weighted_mass = cumulative_weighted_mass + (sample - pattern_length / 2) * p_pattern[sample][i];
				cumulative_weighted_mass = cumulative_weighted_mass + (pattern_length/2 - sample) * p_pattern[pattern_length - sample - 1][i];

				cumulative_mass = cumulative_mass + p_pattern[sample][i];
				cumulative_mass = cumulative_mass + p_pattern[sample - sample - 1][i];
			}

			for (int sample = 0; sample < pattern_length; sample++) {

				mean = mean + p_pattern[sample][i];
				cumulative_weighted_mass = cumulative_weighted_mass + sample * p_pattern[sample][i];
				cumulative_mass = cumulative_mass + p_pattern[sample][i];
			}

			CenterOffMasses[i] = (cumulative_weighted_mass / cumulative_mass);

			mean = mean / pattern_length;
			Means[i] = mean;
#endif
		}
	}
	void AddSample(array<float> ^p_event, array<double>  ^correlations)
	{
#if 0
		for (int i = 0; i < (SENSOR_DATA_TYPE_COUNT + CUSTOM_SENSOR_DATA_TYPE_COUNT); i++) {

			if (pPatternDataValidity[i] == true) {

				pPlotters[i]->SetCorrelationOfCurrentFlow(correlations[i]);

				pPlotters[i]->GetSeriesByName("ActionWindow")->Points->AddY(p_event.pEvent->euler[i]);

				if (pPlotters[i]->GetSeriesByName("ActionWindow")->Points->Count > pPatternLengths[i]) {
					pPlotters[i]->GetSeriesByName("ActionWindow")->Points->RemoveAt(0);
				}

				pPlotters[i]->SetTrigger(
					correlations[i] < 35
				);
			

				
			}
		}
#endif
	}
	void AddPattern(array<ViewSerialEvent> ^p_pattern, int sensor_type_data_index)
	{
#if 0
		int length = p_pattern->Length;
		int action_max_length = pOriginalPattern->Length;

		pPatternLengths[sensor_type_data_index] = length;

	

		for (int i = 0; i < length && i < action_max_length; i++) {

			pOriginalPattern[i].pEvent->euler[sensor_type_data_index] = p_pattern[i].pEvent->euler[sensor_type_data_index];
			
		}



		/*************/

		BNOControl::Plotter ^p_plotter = gcnew BNOControl::Plotter();


		p_plotter->SetFormTitle(SensorDataNames::Names[sensor_type_data_index] + " <- Sensor Data:" + sensor_type_data_index);
		p_plotter->ResetChart();
		p_plotter->AddNewSeries("ActionWindow");
		p_plotter->AddNewSeries("Pattern");
		p_plotter->Show();

		pPlotters[sensor_type_data_index] = p_plotter;




		p_plotter->SetCorrelationOfPattern(pSumOfSquares[sensor_type_data_index]);
		double number;
		for (int i = 0; i < p_pattern->Length; i++) {

			p_plotter->GetSeriesByName("Pattern")->Points->AddY(p_pattern[i].pEvent->euler[sensor_type_data_index]);
		}




#endif

	}


};