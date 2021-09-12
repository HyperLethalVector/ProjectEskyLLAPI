#include "common_header.h"
#include "Filter.h"
#include "SharedMath.h"
//The DollaryDooFilter, it's like a One Euro Filter, except down under!

class LowPassFilter {
public:
	void setAlpha(double _alpha) { 
		if (_alpha <= 0.0f || _alpha > 1.0f) {
			return;
		}
		a = _alpha;
	}
	LowPassFilter(double _alpha, double _initval = 0.0) {
		y = s = _initval;
		setAlpha(_alpha);
		initialized = false;
	}
	float Filter(double _value) {
		double result;
		if (initialized) {
			result = a * _value + (1.0f - a) * s;
		}
		else {
			result = _value;
			initialized = true;
		}
		y = _value;
		s = result;
		return result;
	}
	double filterWithAlpha(double _value, double _alpha) {
		setAlpha(_alpha);
		return Filter(_value);
	}
	bool hasLastRawValue() {
		return initialized;
	}
	double lastRawValue() {
		return y;
	}
	LowPassFilter() {
		LowPassFilter(0.0);
	}
private:
	double y, a, s;
	bool initialized;
};
class OneDollaryDooFilter {
public:
	double curValue;
	double prevValue;	
	OneDollaryDooFilter() {
		setFrequency(0.0);
		setMinCutoff(1.0);
		x = LowPassFilter(alpha(mincutoff));
		dx = LowPassFilter(alpha(dcutoff));
		lasttime = -1.0;
		curValue = 0.0;
		prevValue = curValue;
	}
	OneDollaryDooFilter(double _freq, double _mincutoff = 1.0, double _beta = 0.0, double _dcutoff = 1.0) {
		setFrequency(_freq);
		setMinCutoff(_mincutoff);
		x = LowPassFilter(alpha(mincutoff));
		dx = LowPassFilter(alpha(dcutoff));
		lasttime = -1.0;
		curValue = 0.0;
		prevValue = curValue; 
	}
	void UpdateParams(double _freq, double _mincutoff, double _beta, double _dcutoff) {
		setFrequency(_freq);
		setMinCutoff(_mincutoff);
		setBeta(_beta);
		setDerivativeCutoff(_dcutoff);
		x.setAlpha(alpha(mincutoff));
		dx.setAlpha(alpha(dcutoff));
	}
	double filter(double value, double timestamp = -1.0) {
		prevValue = curValue;
		if (lasttime != -1.0 && timestamp != 1.0)
			freq = 1.0 / (timestamp - lasttime);
		double dvalue = x.hasLastRawValue() ? (value - x.lastRawValue()) * freq : 0.0f;
		double edvalue = dx.filterWithAlpha(value, alpha(dcutoff));
		double cutoff = mincutoff + beta * abs(edvalue);
		curValue = x.filterWithAlpha(value, alpha(cutoff));
		return curValue;
	}
private:
	void setFrequency(double _f) {
		freq = _f;
	}
	void setMinCutoff(double _mc) {
		mincutoff = _mc;
	}
	void setBeta(double _b) {
		beta = _b;
	}
	double alpha(double _cutoff) {
		double te = 1.0f / freq;
		float tau = 1.0f / (2.0f * 3.14159265 * _cutoff);
		return 1.0f / (1.0f + tau / te);
	}
	void setDerivativeCutoff(float _dc) {
		dcutoff = _dc;
	}
	double freq = 300;
	double mincutoff;
	double beta;
	double dcutoff = 0.01;
	LowPassFilter x;
	LowPassFilter dx;
	double lasttime;
};
class OneDollaryDooFilterPose : PoseFilter {
	public:
		double tfreq;
		double tmincutoff;
		double tbeta;
		double tdcutoff;
		double rfreq;
		double rmincutoff;
		double rbeta;
		double rdcutoff;

		OneDollaryDooFilterPose() {
			OneDollaryDooFilterPose(200);
		}
		OneDollaryDooFilterPose(double _freq, double _mincutoff = 1.0, double _beta = 0.0, double _dcutoff = 0.0) {
			tfreq = _freq;
			tmincutoff = _mincutoff;
			tbeta = _beta;
			tdcutoff = _dcutoff;
			rfreq = _freq;
			rmincutoff = _mincutoff;
			rbeta = _beta;
			rdcutoff = _dcutoff;
			xPosFilter = OneDollaryDooFilter(_freq, _mincutoff, _beta, _dcutoff);
			yPosFilter = OneDollaryDooFilter(_freq, _mincutoff, _beta, _dcutoff);
			zPosFilter = OneDollaryDooFilter(_freq, _mincutoff, _beta, _dcutoff);
			xRotFilter = OneDollaryDooFilter(_freq, _mincutoff, _beta, _dcutoff);
			yRotFilter = OneDollaryDooFilter(_freq, _mincutoff, _beta, _dcutoff);
			zRotFilter = OneDollaryDooFilter(_freq, _mincutoff, _beta, _dcutoff);
		}
		void SetFilterEnabled(bool enabled) {
			useFilter = enabled;
		}
		void UpdateTranslationParams(double _freq, double _mincutoff = 1.0, double _beta = 0.0, double _dcutoff = 1.0) {
			tfreq = _freq;
			tmincutoff = _mincutoff;
			tbeta = _beta;
			tdcutoff = _dcutoff;
			xPosFilter.UpdateParams(_freq, _mincutoff, _beta, _dcutoff);
			yPosFilter.UpdateParams(_freq, _mincutoff, _beta, _dcutoff);
			zPosFilter.UpdateParams(_freq, _mincutoff, _beta, _dcutoff);
		}
		void UpdateRotationParams(double _freq, double _mincutoff = 1.0, double _beta = 0.0, double _dcutoff = 1.0) {
			rfreq = _freq;
			rmincutoff = _mincutoff;
			rbeta = _beta;
			rdcutoff = _dcutoff;
			xRotFilter.UpdateParams(_freq, _mincutoff, _beta, _dcutoff);
			yRotFilter.UpdateParams(_freq, _mincutoff, _beta, _dcutoff);
			zRotFilter.UpdateParams(_freq, _mincutoff, _beta, _dcutoff);
		}

		void Filter(double xt, double yt, double zt, double xr, double yr, double zr, double wr, float timestamp = -1.0) {
			
			unfilteredQuaternion.x = xr;
			unfilteredQuaternion.y = yr;
			unfilteredQuaternion.z = zr;
			unfilteredQuaternion.w = wr;
			QuatToEuler(unfilteredQuaternion, unfilteredEuler);
			if (useFilter) {
				
				filteredTranslation.x = xPosFilter.filter(xt, timestamp);
				filteredTranslation.y = yPosFilter.filter(yt, timestamp);
				filteredTranslation.z = zPosFilter.filter(zt, timestamp);

				filteredEuler.x = xRotFilter.filter(unfilteredEuler.x, timestamp);
				filteredEuler.y = yRotFilter.filter(unfilteredEuler.y, timestamp);
				filteredEuler.z = zRotFilter.filter(unfilteredEuler.z, timestamp);
			}  
			else { 
				filteredTranslation.x = xt;
				filteredTranslation.y = yt;
				filteredTranslation.z = zt;

				filteredEuler.x = unfilteredEuler.x;
				filteredEuler.y = unfilteredEuler.y;
				filteredEuler.z = unfilteredEuler.z;
			}
		}
		void Filter(double xt, double yt, double zt, float timestamp = -1.0) {

			if (useFilter) {
				filteredTranslation.x = xPosFilter.filter(xt, timestamp);
				filteredTranslation.y = yPosFilter.filter(yt, timestamp);
				filteredTranslation.z = zPosFilter.filter(zt, timestamp);
			}
			else {
				filteredTranslation.x = xt;
				filteredTranslation.y = yt;
				filteredTranslation.z = zt;
			}
		}
		void ObtainFilteredPose(float &xt, float &yt, float &zt, float &xr, float &yr, float &zr, float &wr) {
			EulerToQuat(filteredEuler, filteredQuaternion);
			xt = filteredTranslation.x;
			yt = filteredTranslation.y;
			zt = filteredTranslation.z;
			xr = filteredQuaternion.x;
			yr = filteredQuaternion.y;
			zr = filteredQuaternion.z;
			wr = filteredQuaternion.w; 
		}
		void ObtainFilteredPose(double& xt, double& yt, double& zt, double& xr, double& yr, double& zr, double& wr) {
			EulerToQuat(filteredEuler, filteredQuaternion);
			xt = filteredTranslation.x;
			yt = filteredTranslation.y;
			zt = filteredTranslation.z;
			xr = filteredQuaternion.x;
			yr = filteredQuaternion.y;
			zr = filteredQuaternion.z;
			wr = filteredQuaternion.w;
		}
		void ObtainFilteredPose(double& xt, double& yt, double& zt) {
			xt = filteredTranslation.x;
			yt = filteredTranslation.y;
			zt = filteredTranslation.z;
		}
		void Reset() {
			xPosFilter = OneDollaryDooFilter(tfreq, tmincutoff, tbeta, tdcutoff);
			yPosFilter = OneDollaryDooFilter(tfreq, tmincutoff, tbeta, tdcutoff);
			zPosFilter = OneDollaryDooFilter(tfreq, tmincutoff, tbeta, tdcutoff);
			xRotFilter = OneDollaryDooFilter(rfreq, rmincutoff, rbeta, rdcutoff);
			yRotFilter = OneDollaryDooFilter(rfreq, rmincutoff, rbeta, rdcutoff);
			zRotFilter = OneDollaryDooFilter(rfreq, rmincutoff, rbeta, rdcutoff);
		}
	private:
		OneDollaryDooFilter xPosFilter;
		OneDollaryDooFilter yPosFilter;
		OneDollaryDooFilter zPosFilter;
		OneDollaryDooFilter xRotFilter;
		OneDollaryDooFilter yRotFilter;
		OneDollaryDooFilter zRotFilter;

		bool useFilter = false;

		Vector3 filteredTranslation;

		Quaternion unfilteredQuaternion;
		Quaternion filteredQuaternion;

		Vector3 unfilteredEuler;
		Vector3 filteredEuler;
};