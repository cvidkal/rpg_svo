#pragma once

#include <fstream>
#include <string>
#include "IMU/InertialSensor/InertialSensor.h"

namespace slam {

class ImuSaver : public InertialSensor::Listener
{
public:
	ImuSaver(const std::string path, bool bSaveMag = true);
	~ImuSaver();

private:
	void onInertialSensorData(
		float gx, float gy, float gz,
		float ax, float ay, float az,
		float mx, float my, float mz,
		float qw, float qx, float qy, float qz, 
		double timestamp) override;

private:
	FILE* mpFile;

	bool mbSaveMag; 

};

}
