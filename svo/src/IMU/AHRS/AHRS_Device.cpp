
#include "stdafx.h"
#include "AHRS_Device.h"

namespace slam {

//---------------------------------------------------------------------------------------------------
DeviceAHRS::DeviceAHRS()
{
	mType = AHRSType::DeviceAHRS;

	q0 = 1.0f; 
	q1 = 0.0f; 
	q2 = 0.0f; 
	q3 = 0.0f;

}


//---------------------------------------------------------------------------------------------------
DeviceAHRS::~DeviceAHRS()
{
}

//---------------------------------------------------------------------------------------------------

void DeviceAHRS::Update(float gx, float gy, float gz, float ax, float ay, float az,
	float mx, float my, float mz, float dt) 
{
}

//---------------------------------------------------------------------------------------------------
void DeviceAHRS::UpdateIMU(float gx, float gy, float gz, float ax, float ay, float az, float dt)
{
}

}