#include "stdafx.h"

#include "InertialSensor_Dataset.h"
#include <fstream>
#include <sstream>
#include <iostream>
#include "IMU/Utils/TimeUtils.h"


namespace slam {

std::mutex InertialSensor_Dataset::mMutex;
std::condition_variable InertialSensor_Dataset::mCondvar; 
double InertialSensor_Dataset::mLastImageTimestamp;

//-----------------------------------------------------------------------------
InertialSensor_Dataset::InertialSensor_Dataset(const std::string& path, const std::string& configFile)
	: InertialSensor(configFile), mIndex(1)
{
	LoadImuData(path.c_str());

	// 
	mpThread = nullptr; 
}


InertialSensor_Dataset::~InertialSensor_Dataset()
{
	// stop
	Stop(); 
}

void InertialSensor_Dataset::Start()
{
	if (mpThread == nullptr) {
		// start a thread
		mbThreadDone = false;
		mpThread = new std::thread([&] {this->RunThread(); });
	}
}

void InertialSensor_Dataset::Stop()
{
	// stop thread
	mbThreadDone = true;
	mCondvar.notify_all();
	if (mpThread != nullptr) {
		mpThread->join();
		delete mpThread;
		mpThread = nullptr; 
	}
	
}

bool InertialSensor_Dataset::LoadImuData(const char* path)
{
	cout << "Loading IMU Data from " << path << " ...";

	ifstream imuFile;
	imuFile.open(path);
	if (!imuFile.is_open()) {
		cout << "Failed!" << endl;
		return false;
	}

	bool bFirstLine = true;
	char ch;
	while (!imuFile.eof()) {
		string s;
		double timestamp;
		getline(imuFile, s);
		if (bFirstLine) {
			bFirstLine = false;
			continue;
		}
		if (!s.empty())
		{
			ImuInput imuInput;
			stringstream ss;
			ss << s;
			ss >> timestamp;
			imuInput.timestamp = timestamp;
			for (int i = 0; i < 9; i++) {
				ss >> ch >> imuInput.data[i];
			}
			// quaternion
			ss >> ch >> imuInput.data[12];		// qw
			ss >> ch >> imuInput.data[9];		// qx
			ss >> ch >> imuInput.data[10];		// qy
			ss >> ch >> imuInput.data[11];		// qz

			mvImuData.push_back(imuInput);
		}
	}
	imuFile.close();
	cout << "Done! " << endl;

	return true;
}

//-----------------------------------------------------------------------------
void InertialSensor_Dataset::NotifySendImuData(double timestamp)
{
	std::lock_guard<std::mutex> ulock(mMutex);
	mLastImageTimestamp = timestamp; 

	mCondvar.notify_all();
}

//-----------------------------------------------------------------------------
void InertialSensor_Dataset::RunThread()
{
	double timestamp; 

	while (!mbThreadDone) {
		{
			std::unique_lock<std::mutex> ulock(mMutex);
			// wait until new vision data arrives
			mCondvar.wait(ulock);

			if (mbThreadDone)
				break;

			timestamp = mLastImageTimestamp; 
		}

		// send imu data
		while (mIndex < mvImuData.size() && mvImuData[mIndex].timestamp < timestamp + 0.03) {
			auto& imuInput = mvImuData[mIndex];
			float* gyro = imuInput.data;
			float* acc = imuInput.data + 3;
			float* mag = imuInput.data + 6;
			float* quat = imuInput.data + 9;
			OnImuReceived(gyro, acc, mag, quat, imuInput.timestamp);
			mIndex++; 
		}
	}
}

}
