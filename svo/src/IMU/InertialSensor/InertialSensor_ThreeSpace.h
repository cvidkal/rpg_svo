#pragma once

#include "util/Config.h"

#if USE_IMU_YEI

#include "yei_threespace_api.h"
#include "InertialSensor.h"
#include <vector>
#include <mutex>

namespace slam {

// Todo: Singleton
class InertialSensor_ThreeSpace : public InertialSensor
{
public:
	InertialSensor_ThreeSpace(const std::string& configFile);
	~InertialSensor_ThreeSpace();

	void Start() override;
	void Stop() override;

private:
	int mSampleInterval;
	TSS_Device_Id  mDeviceID;
	bool mIsStreaming; 
	long long mDeltaTimestamp;

private:

	bool Config();
	void AlignTimestamp();

	static void CALLBACK TSS_CallBackFunc(TSS_Device_Id device, char* output_data,
		unsigned int output_data_len, unsigned int* _timestamp);

private:
	static std::vector<InertialSensor_ThreeSpace*> mSensorArray;
	static std::mutex mSensorMutex;
};

}

#endif
