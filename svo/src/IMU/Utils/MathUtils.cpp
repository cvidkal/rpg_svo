#include "stdafx.h"
#include "MathUtils.h"


//-----------------------------------------------------------------------------
// Fast inverse square-root
// See: http://en.wikipedia.org/wiki/Fast_inverse_square_root

float invSqrt(float x) {
	float halfx = 0.5f * x;
	float y = x;
	long i = *(long*)&y;
	i = 0x5f3759df - (i >> 1);
	y = *(float*)&i;
	y = y * (1.5f - (halfx * y * y));
	return y;
}


//------------------------------------------------------------------------------
Eigen::Vector3d MatrixToAttitudeAngle(const Eigen::Matrix3d& attMat)
{
	Eigen::Vector3d attVec;

	// pitch
	attVec[0] = asin(attMat(2, 1));
	// roll
	attVec[1] = atan2(-attMat(2, 0), attMat(2, 2));
	// azimuth
	attVec[2] = atan2(-attMat(0, 1), attMat(1, 1));

	return attVec;
}

//------------------------------------------------------------------------------
Eigen::Vector3d QuaternionToAttitudeAngle(const Eigen::Quaterniond& q)
{
	Eigen::Matrix3d attMat = q.toRotationMatrix();
	Eigen::Vector3d attVec;

	// pitch
	attVec[0] = asin(attMat(2, 1));
	// roll
	attVec[1] = atan2(-attMat(2, 0), attMat(2, 2));
	// azimuth
	attVec[2] = atan2(-attMat(0, 1), attMat(1, 1));

	return attVec;
}

//------------------------------------------------------------------------------
Eigen::Matrix3d AttitudeAngleToMatrix(const Eigen::Vector3d& attVec)
{
	Eigen::Matrix3d attMat;

	// pitch
	float cp = cos(attVec[0]);
	float sp = sin(attVec[0]);
	// roll
	float cr = cos(attVec[1]);
	float sr = sin(attVec[1]);
	// azimuth
	float ca = cos(attVec[2]);
	float sa = sin(attVec[2]);

	attMat << cr*ca - sp*sr*sa, -cp*sa, sr*ca + sp*cr*sa,
		cr*sa - sp*sr*ca, cp*ca, sr*sa - sp*cr*ca,
		-cp*sr, sp, cp*cr;

	return attMat;
}
