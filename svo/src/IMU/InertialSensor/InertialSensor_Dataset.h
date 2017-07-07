#pragma once

#include "InertialSensor.h"
#include <vector>

#include <thread>
#include <mutex>
#include <condition_variable>

using namespace std;

namespace slam {

struct ImuInput
{
	double timestamp;
	// In this order: gyro, acc, mag, quaternion
	float data[13];	
};

class InertialSensor_Dataset : public InertialSensor
{
public:
	InertialSensor_Dataset(const std::string& path, const std::string& configFile);
	~InertialSensor_Dataset();

	void Start() override;
	void Stop() override;

	static void NotifySendImuData(double timestamp); 

private:
	int mIndex;
	vector<ImuInput> mvImuData;

private:
	bool LoadImuData(const char* path);

private:
	// multi thread
	static std::mutex mMutex;
	static std::condition_variable mCondvar;
	bool mbThreadDone;
	bool mbThreadReset;
	std::thread* mpThread;

	static double mLastImageTimestamp; 

	void RunThread();
};

}
