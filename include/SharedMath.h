#include <algorithm>
#include <cmath>
#include <cstring>
#include <vector>
#include <sstream>
#include <queue>
#include <map>
#include <set>
#pragma once
class Vector3 {
public:
	float x = 0;
	float y = 0;
	float z = 0;
};
class Quaternion {
public:
	float x = 0;
	float y = 0;
	float z = 0;
	float w = 1;
};
float Power2(float x) {
	return x * x;
}
float Length(Vector3& vec)
{
	return sqrtf(Power2(vec.x) + Power2(vec.y) + Power2(vec.z));
}

float Length(const Quaternion& quat)
{
	return sqrtf(quat.x * quat.x + quat.y * quat.y + quat.z * quat.z + quat.w * quat.w);
}

void Normalize(Vector3& vec)
{
	if (vec.x == 0 && vec.y == 0 && vec.z == 0)
		return;

	float len = Length(vec);
	vec.x /= len;
	vec.y /= len;
	vec.z /= len;
}
void Normalize(Quaternion& quat) {
	float len = Length(quat);
	quat.x /= len;
	quat.y /= len;
	quat.z /= len;
	quat.w /= len;
}
void EulerToQuat(Vector3& euler, Quaternion& rot)
{
	float theta = Length(euler);
	float c_t = cosf(theta / 2.0f);
	float sinc_t;

	if (theta > 1e-10) {
		sinc_t = sinf(theta / 2.0f) / theta;
	}
	else {
		sinc_t = 0.5 + theta * theta / 48.0f;
	}

	rot.x = euler.x * sinc_t;
	rot.y = euler.y * sinc_t;
	rot.z = euler.z * sinc_t;
	rot.w = c_t;
	Normalize(rot);
}
void QuatToEuler(Quaternion& quat, Vector3& rot)
{
	float theta;

	if (quat.w < 0)
		theta = -2 * acos(-quat.w);
	else
		theta = 2 * acos(quat.w);

	Vector3 v;
	v.x = quat.x;
	v.y = quat.y;
	v.z = quat.z;
	float v_len = Length(v);

	if (v_len > 1e-10) {
		float scalar = theta / v_len;
		rot.x = v.x * scalar;
		rot.y = v.y * scalar;
		rot.z = v.z * scalar;
	}
	else {
		float scalar = 2.0;
		rot.x = v.x * scalar;
		rot.y = v.y * scalar;
		rot.z = v.z * scalar;
	}
	if (isnan(rot.x))rot.x = 0;
	if (isnan(rot.y))rot.y = 0;
	if (isnan(rot.z))rot.z = 0;
}
