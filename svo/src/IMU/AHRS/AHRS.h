/* Attitude Heading Reference System */

#pragma once

#include <Eigen/Dense>

namespace slam {


enum class AHRSType
{
	DeviceAHRS,
	GyroIntegration,
	Madgwick,
	Mahony
};


class AHRS
{
public:
	AHRS();
	virtual ~AHRS();

	AHRSType mType;

public:
	// 9 axis
	virtual void Update(float gx, float gy, float gz, float ax, float ay, float az, 
		float mx, float my, float mz, float dt) = 0;
	// 6 axis
	virtual void UpdateIMU(float gx, float gy, float gz, float ax, float ay, float az, float dt) = 0;

public:
	bool InitOrientation(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz);
	bool GetQuaternion(float& qw, float& qx, float& qy, float& qz);

protected:
	bool mbInitialized;
	float q0, q1, q2, q3;	// quaternion of sensor frame relative to auxiliary frame

	Eigen::Quaternionf mDeltaQuaternion;

};


}