#include "stdafx.h"

#include "AHRS.h"
#include "IMU/Utils/MathUtils.h"

namespace slam {

AHRS::AHRS()
	: mbInitialized(false)
{
	Eigen::Matrix3f dR;
	dR << 0, -1, 0,
		1, 0, 0,
		0, 0, 1;
	mDeltaQuaternion = Eigen::Quaternionf(dR);
}


AHRS::~AHRS()
{
}

bool AHRS::GetQuaternion(float& qw, float& qx, float& qy, float& qz)
{
	if (!mbInitialized)
		return false;

	//if (mType == AHRSType::Madgwick) {
	//	// tranform to imu coordinate system
	//	Eigen::Quaternionf q(q0, q1, q2, q3);
	//	q = mDeltaQuaternion * q;
	//	qw = q.w();
	//	qx = q.x();
	//	qy = q.y();
	//	qz = q.z();
	//} 
	//else {
		qw = q0;
		qx = q1;
		qy = q2;
		qz = q3;
	//}

	return true;
}


// use imu's coordinate system: x right, y forward, z up
bool AHRS::InitOrientation(float gx, float gy, float gz, 
	float ax, float ay, float az, float mx, float my, float mz)
{
	if (mx == 0 && my == 0 && mz == 0)
		return false;

	float normA = sqrt(ax*ax + ay*ay + az*az);
	float g = 9.81f;
	if (normA < 0.8f * g) {
		// gravity less than 10% of normal value
		return false;
	}

	float Hx = my*az - mz*ay;
	float Hy = mz*ax - mx*az;
	float Hz = mx*ay - my*ax;
	float normH = sqrt(Hx*Hx + Hy*Hy + Hz*Hz);
	if (normH < 0.1f) {
		// device is close to free fall (or in space?), or close to
		// magnetic north pole. Typical values are  > 100.
		return false;
	}

	float invH = 1.0f / normH;
	Hx *= invH;
	Hy *= invH;
	Hz *= invH;

	float invA = 1.0f / normA;
	float Ax = ax * invA;
	float Ay = ay * invA;
	float Az = az * invA;
	
	float Mx = Ay*Hz - Az*Hy;
	float My = Az*Hx - Ax*Hz;
	float Mz = Ax*Hy - Ay*Hx;

	Eigen::Matrix3f R;
	R << Hx, Hy, Hz,
		 Mx, My, Mz,
		 Ax, Ay, Az;

#if 1
	float theta = -PI / 2;
	Eigen::Matrix3f dR;
	dR << cos(theta), -sin(theta), 0,
		sin(theta), cos(theta), 0,
		0, 0, 1;

	R = dR * R;
#endif

	Eigen::Quaternionf q(R);
	q0 = q.w();
	q1 = q.x();
	q2 = q.y();
	q3 = q.z();

	return true;
}

}