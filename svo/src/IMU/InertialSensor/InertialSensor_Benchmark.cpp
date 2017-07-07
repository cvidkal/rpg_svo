#include "stdafx.h"

#include "InertialSensor_Benchmark.h"
#include <fstream>
#include <sstream>
#include <iostream>
#include "IMU/Utils/TimeUtils.h"
#include <opencv2/opencv.hpp>

namespace slam {

std::mutex InertialSensor_Benchmark::mMutex;
std::condition_variable InertialSensor_Benchmark::mCondvar; 
double InertialSensor_Benchmark::mLastImageTimestamp;

//-----------------------------------------------------------------------------
InertialSensor_Benchmark::InertialSensor_Benchmark(const std::string& path, const std::string& configFile)
	: InertialSensor(configFile), mIndex(1)
{
	LoadImuData(path.c_str());

	// 
	mpThread = nullptr; 
}


InertialSensor_Benchmark::~InertialSensor_Benchmark()
{
	// stop
	Stop(); 
}

void InertialSensor_Benchmark::Start()
{
	if (mpThread == nullptr) {
		// start a thread
		mbThreadDone = false;
		mpThread = new std::thread([&] {this->RunThread(); });
	}
}

void InertialSensor_Benchmark::Stop()
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

std::vector<std::string> InertialSensor_Benchmark::split(std::string str, char delimiter) {
    std::vector<std::string> internal;
    std::stringstream ss(str); // Turn the string into a stream.
    std::string tok;

    while (getline(ss, tok, delimiter)) {
        internal.push_back(tok);
    }
    return internal;
}

bool InertialSensor_Benchmark::LoadImuData(const char* path)
{

    cv::FileStorage fs("Dataset_Config.xml", cv::FileStorage::READ);
    fs["DatasetDir"] >> m_DatasetDir;
    fs["TestIMUSleep"] >> m_nTestIMUSleep;

    std::string imuFileName = m_DatasetDir + "/imu.csv";
    m_ImuDatasetFile.open(imuFileName);

    if (!m_ImuDatasetFile.good()){
        cout << "no imu.csv file, exit" << endl;
        exit(1);
    }

    std::string s;
    m_ImuDatasetFile >> s;

    while (!m_ImuDatasetFile.eof()){
        std::string s;
        m_ImuDatasetFile >> s;
        std::vector<std::string> nums = split(s, ',');
        if (s.empty()){
            continue;
        }
        long long timestamp = std::stoll(nums[0]);
        
        ImuInput imuInput;
        imuInput.timestamp = timestamp * 1e-9;

        for (int i = 0; i < 9; i++) {
            imuInput.data[i] = std::stof(nums[i+1]);
        }
        // quaternion
        imuInput.data[12] = 1;		// qw
        imuInput.data[9] = 0;		// qx
        imuInput.data[10] = 0;		// qy
        imuInput.data[11] = 0;		// qz

        mvImuData.push_back(imuInput);

    }
    m_ImuDatasetFile.close();

    m_nNumOfData = m_vTimestamps.size();
    m_nCurrentIndex = 0;

    return true;
}

//-----------------------------------------------------------------------------
void InertialSensor_Benchmark::NotifySendImuData(double timestamp)
{
	std::lock_guard<std::mutex> ulock(mMutex);
	mLastImageTimestamp = timestamp; 

	mCondvar.notify_all();
}

//-----------------------------------------------------------------------------
void InertialSensor_Benchmark::RunThread()
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
