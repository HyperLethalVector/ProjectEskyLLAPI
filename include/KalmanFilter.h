#include "common_header.h"
#include "Filter.h"
class KalmanFilter {
public:
	float DEFAULT_Q = 0.000001f;
	float DEFAULT_R = 0.01f;

	float DEFAULT_P = 1;
	KalmanFilter(){
		SetKalmanFilter(DEFAULT_Q, DEFAULT_R);
	}
	void SetKalmanFilter(float aQ, float aR) {
		q = aQ;
		r = aR;
		k = 0;
		x = 0;
	}

	void UpdateParams(float newQ, float newR) {
		// update values if supplied.
		if (newQ != NULL && q != newQ) {
			q = (float)newQ;
		}
		if (newR != NULL && r != newR) {
			r = (float)newR;
		}
	}
	float Update(float measurement) {
		// update measurement.
		{
			k = (p + q) / (p + q + r);
			p = r * (p + q) / (r + p + q);
		}

		// filter result back into calculation.
		float result = x + (measurement - x) * k;
		x = result;
		return result;
	}
	void Reset() {
		p = 1;
		x = 0;
		k = 0;
	}
private:
	float q = DEFAULT_Q;
	float r = DEFAULT_R;
	float p = DEFAULT_P;
	float x;
	float k;
};
class KalmanFilterPose {
public:
	KalmanFilterPose() {
		SetDefault(1.0, 0.0,1.0,0.0);
	} 
	void SetDefault(float Qt, float Rt,float Qr, float Rr) {
		xPosFilter.SetKalmanFilter(Qt, Rt);
		yPosFilter.SetKalmanFilter(Qt, Rt);
		zPosFilter.SetKalmanFilter(Qt, Rt);
		xRotFilter.SetKalmanFilter(Qr, Rr);
		yRotFilter.SetKalmanFilter(Qr, Rr);
		zRotFilter.SetKalmanFilter(Qr, Rr);
		wRotFilter.SetKalmanFilter(Qr, Rr);
	}
	void SetFilterEnabled(bool enabled) {
		useFilter = enabled;
	}
	void Reset() {
		xPosFilter.Reset();
		yPosFilter.Reset();
		zPosFilter.Reset();
		xRotFilter.Reset();
		yRotFilter.Reset();
		zRotFilter.Reset();
		wRotFilter.Reset();
	}
	void UpdateTranslationParams(double Q = 1.0, double R = 0.0) {
		xPosFilter.UpdateParams(Q,R);
		yPosFilter.UpdateParams(Q,R);
		zPosFilter.UpdateParams(Q, R);
	}
	void UpdateRotationParams(double Q = 1.0, double R = 0.0) {
		xRotFilter.UpdateParams(Q, R);
		yRotFilter.UpdateParams(Q, R);
		zRotFilter.UpdateParams(Q, R);
		wRotFilter.UpdateParams(Q, R);
	}
	//note this still does filtering on the x y z w components of rotation, should use rotation parameterisation as thaytay suggested
	void Filter(double xt, double yt, double zt, double xr, double yr, double zr, double wr, float timestamp = -1.0) {
		if (useFilter) {
			transX = xPosFilter.Update(xt);
			transY = yPosFilter.Update(yt);
			transZ = zPosFilter.Update(zt);
			rotX = xRotFilter.Update(xr);
			rotY = yRotFilter.Update(yr);
			rotZ = zRotFilter.Update(zr);
			rotW = wRotFilter.Update(wr);
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
	void ObtainFilteredPose(float& xt, float& yt, float& zt, float& xr, float& yr, float& zr, float& wr) {
		xt = transX;
		yt = transY;
		zt = transZ;
		xr = rotX;
		yr = rotY;
		zr = rotZ;
		wr = rotW;
	}
private:
	KalmanFilter xPosFilter;
	KalmanFilter yPosFilter;
	KalmanFilter zPosFilter;
	KalmanFilter xRotFilter;
	KalmanFilter yRotFilter;
	KalmanFilter zRotFilter;
	KalmanFilter wRotFilter; 
	bool useFilter = false;
	double transX = 0;
	double transY = 0;
	double transZ = 0;
	double rotX = 0;
	double rotY = 0;
	double rotZ = 0;
	double rotW = 1;
};