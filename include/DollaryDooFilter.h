#include "common_header.h"
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
	double freq;
	double mincutoff;
	double beta;
	double dcutoff;
	LowPassFilter x;
	LowPassFilter dx;
	double lasttime;
};
class OneDollaryDooFilterPose {
	public:
		OneDollaryDooFilterPose() {
			 
		}
		OneDollaryDooFilterPose(double _freq, double _mincutoff = 1.0, double _beta = 0.0, double _dcutoff = 0.0) {
			xPosFilter = OneDollaryDooFilter(_freq, _mincutoff, _beta, _dcutoff);
			yPosFilter = OneDollaryDooFilter(_freq, _mincutoff, _beta, _dcutoff);
			zPosFilter = OneDollaryDooFilter(_freq, _mincutoff, _beta, _dcutoff);
			xRotFilter = OneDollaryDooFilter(_freq, _mincutoff, _beta, _dcutoff);
			yRotFilter = OneDollaryDooFilter(_freq, _mincutoff, _beta, _dcutoff);
			zRotFilter = OneDollaryDooFilter(_freq, _mincutoff, _beta, _dcutoff);
			wRotFilter = OneDollaryDooFilter(_freq, _mincutoff, _beta, _dcutoff);
		}
		void SetFilterEnabled(bool enabled) {
			useFilter = enabled;
		}
		void UpdateTranslationParams(double _freq, double _mincutoff = 1.0, double _beta = 0.0, double _dcutoff = 1.0) {
			xPosFilter.UpdateParams(_freq, _mincutoff, _beta, _dcutoff);
			yPosFilter.UpdateParams(_freq, _mincutoff, _beta, _dcutoff);
			zPosFilter.UpdateParams(_freq, _mincutoff, _beta, _dcutoff);
		}
		void UpdateRotationParams(double _freq, double _mincutoff = 1.0, double _beta = 0.0, double _dcutoff = 1.0) {

			xRotFilter.UpdateParams(_freq, _mincutoff, _beta, _dcutoff);
			yRotFilter.UpdateParams(_freq, _mincutoff, _beta, _dcutoff);
			zRotFilter.UpdateParams(_freq, _mincutoff, _beta, _dcutoff);
			wRotFilter.UpdateParams(_freq, _mincutoff, _beta, _dcutoff);
		}

		void Filter(double xt, double yt, double zt, double xr, double yr, double zr, double wr, float timestamp = -1.0) {
			if (useFilter) {
				transX = xPosFilter.filter(xt, timestamp);
				transY = yPosFilter.filter(yt, timestamp);
				transZ = zPosFilter.filter(zt, timestamp);
				rotX = xRotFilter.filter(xr, timestamp);
				rotY = yRotFilter.filter(yr, timestamp);
				rotZ = zRotFilter.filter(zr, timestamp);
				rotW = wRotFilter.filter(wr, timestamp);
			}  
			else { 
				transX = xt;
				transY = yt;
				transZ = zt;
				rotX = xr;
				rotY = yr;
				rotZ = zr;
				rotW = wr;
			}
		}
		void ObtainFilteredPose(float &xt, float &yt, float &zt, float &xr, float &yr, float &zr, float &wr) {
			xt = transX;
			yt = transY;
			zt = transZ;
			xr = rotX;
			yr = rotY;
			zr = rotZ;
			wr = rotW; 
		}
	private:
		OneDollaryDooFilter xPosFilter;
		OneDollaryDooFilter yPosFilter;
		OneDollaryDooFilter zPosFilter;
		OneDollaryDooFilter xRotFilter;
		OneDollaryDooFilter yRotFilter;
		OneDollaryDooFilter zRotFilter;
		OneDollaryDooFilter wRotFilter;
		bool useFilter = false;
		double transX;
		double transY;
		double transZ;
		double rotX;
		double rotY;
		double rotZ;
		double rotW;
};