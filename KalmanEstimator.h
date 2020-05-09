#pragma once
using namespace System;
namespace SignalProcessing
{
	ref class KalmanEstimator {
	private:
		float _err_measure;
		float _err_estimate;
		float _q;
		float _current_estimate;
		float _last_estimate;
		float _kalman_gain;
	public:
		KalmanEstimator() {

			_err_measure = 2;
			_err_estimate = 0.5;
			_q = 0.1;
		}
		void SetParams(float e_meas, float e_est, float q) {
			_err_measure = e_meas;
			_err_estimate = e_est;
			_q = q;
		}

		float updateEstimate(float mea)
		{
			_kalman_gain = _err_estimate / (_err_estimate + _err_measure);
			_current_estimate = _last_estimate + _kalman_gain * (mea - _last_estimate);
			_err_estimate = (1.0 - _kalman_gain)*_err_estimate + Math::Abs(_last_estimate - _current_estimate)*_q;
			_last_estimate = _current_estimate;

			return _current_estimate;
		}

	};

}