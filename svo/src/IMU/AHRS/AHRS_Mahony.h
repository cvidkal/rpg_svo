//=====================================================================================================
// MahonyAHRS.h
//=====================================================================================================
//
// Madgwick's implementation of Mayhony's AHRS algorithm.
// See: http://www.x-io.co.uk/node/8#open_source_ahrs_and_imu_algorithms
//
// Date			Author			Notes
// 29/09/2011	SOH Madgwick    Initial release
// 02/10/2011	SOH Madgwick	Optimised for reduced CPU load
//
//=====================================================================================================
#pragma once

#include "AHRS.h"


#define twoKpDef	(2.0f * 0.5f)	// 2 * proportional gain
#define twoKiDef	(2.0f * 0.0f)	// 2 * integral gain


class MahonyAHRS : public AHRS
{
public:
	MahonyAHRS();
	~MahonyAHRS();

	void Update(float gx, float gy, float gz, float ax, float ay, float az,
		float mx, float my, float mz, float dt) override;

	void UpdateIMU(float gx, float gy, float gz, float ax, float ay, float az, float dt) override;

private:
	volatile float twoKp;			// 2 * proportional gain (Kp)
	volatile float twoKi;			// 2 * integral gain (Ki)
};

