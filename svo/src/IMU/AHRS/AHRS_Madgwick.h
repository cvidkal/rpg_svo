//=====================================================================================================
// MadgwickAHRS.h
//=====================================================================================================
//
// Implementation of Madgwick's IMU and AHRS algorithms.
// See: http://www.x-io.co.uk/node/8#open_source_ahrs_and_imu_algorithms
//
// Date			Author          Notes
// 29/09/2011	SOH Madgwick    Initial release
// 02/10/2011	SOH Madgwick	Optimised for reduced CPU load


#pragma once

#include "AHRS.h"

#define BetaDef		0.3f		// 2 * proportional gain

namespace slam {

class MadgwickAHRS : public AHRS
{
public:
	MadgwickAHRS();
	~MadgwickAHRS();

	void Update(float gx, float gy, float gz, float ax, float ay, float az, 
		float mx, float my, float mz, float dt) override;

	void UpdateIMU(float gx, float gy, float gz, float ax, float ay, float az, float dt) override;

private:
	volatile float beta;			// algorithm gain
};

}
