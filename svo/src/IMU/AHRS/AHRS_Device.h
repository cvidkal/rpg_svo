

#pragma once

#include "AHRS.h"

namespace slam {

#define BetaDef		0.3f		// 2 * proportional gain

class DeviceAHRS : public AHRS
{
public:
	DeviceAHRS();
	~DeviceAHRS();

	void Update(float gx, float gy, float gz, float ax, float ay, float az, 
		float mx, float my, float mz, float dt) override;

	void UpdateIMU(float gx, float gy, float gz, float ax, float ay, float az, float dt) override;
};

}