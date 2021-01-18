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
		double te = 1.0 / freq;
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
class OneDollaryDooFilterVector3 {
	public:
		OneDollaryDooFilterVector3() {
			 
		}
		OneDollaryDooFilterVector3(double _freq, double _mincutoff = 1.0, double _beta = 0.0, double _dcutoff = 0.0) {
			xFilter = OneDollaryDooFilter(_freq, _mincutoff, _beta, _dcutoff);
			yFilter = OneDollaryDooFilter(_freq, _mincutoff, _beta, _dcutoff);
			zFilter = OneDollaryDooFilter(_freq, _mincutoff, _beta, _dcutoff);
		}
		void UpdateParams(double _freq, double _mincutoff = 1.0, double _beta = 0.0, double _dcutoff = 1.0) {
			xFilter.UpdateParams(_freq, _mincutoff, _beta, _dcutoff);
			yFilter.UpdateParams(_freq, _mincutoff, _beta, _dcutoff);
			zFilter.UpdateParams(_freq, _mincutoff, _beta, _dcutoff);
		}
		void Filter(double x, double y, double z,double &x_out, double &y_out, double &z_out, float timestamp = -1.0) {
			x_out = xFilter.filter(x, timestamp);
			y_out = yFilter.filter(y, timestamp);
			z_out = zFilter.filter(z, timestamp);
		}
	private:
		OneDollaryDooFilter xFilter;
		OneDollaryDooFilter yFilter;
		OneDollaryDooFilter zFilter;
		double transX;
		double transY;
		double transZ;
};
class OneDollaryDooFilterQuaternion {
public:
	OneDollaryDooFilterQuaternion(double _freq, double _mincutoff = 1.0, double _beta = 0.0, double _dcutoff = 0.0) {
		xFilter = OneDollaryDooFilter(_freq, _mincutoff, _beta, _dcutoff);
		yFilter = OneDollaryDooFilter(_freq, _mincutoff, _beta, _dcutoff);
		zFilter = OneDollaryDooFilter(_freq, _mincutoff, _beta, _dcutoff);
		wFilter = OneDollaryDooFilter(_freq, _mincutoff, _beta, _dcutoff);
	}
	void UpdateParams(double _freq, double _mincutoff = 1.0, double _beta = 0.0, double _dcutoff = 1.0) {
		xFilter.UpdateParams(_freq, _mincutoff, _beta, _dcutoff);
		yFilter.UpdateParams(_freq, _mincutoff, _beta, _dcutoff);
		zFilter.UpdateParams(_freq, _mincutoff, _beta, _dcutoff);
		wFilter.UpdateParams(_freq, _mincutoff, _beta, _dcutoff);
	}
	void Filter(double x, double y, double z, double w, double& x_out, double& y_out, double& z_out, double& w_out, float timestamp = -1.0) {
		x_out = xFilter.filter(x, timestamp);
		y_out = yFilter.filter(y, timestamp);
		z_out = zFilter.filter(z, timestamp);
		w_out = zFilter.filter(w, timestamp);
	}
private:
	OneDollaryDooFilter xFilter;
	OneDollaryDooFilter yFilter;
	OneDollaryDooFilter zFilter;
	OneDollaryDooFilter wFilter;
	double rotX;
	double rotY;
	double rotZ;
	double rotW;
};
