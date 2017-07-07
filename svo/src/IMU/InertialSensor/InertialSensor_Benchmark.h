#pragma once

#include "InertialSensor.h"
#include "InertialSensor_Dataset.h"
#include <vector>

#include <thread>
#include <mutex>
#include <condition_variable>
#include <fstream>

using namespace std;

namespace slam {

class InertialSensor_Benchmark : public InertialSensor
{
public:
	InertialSensor_Benchmark(const std::string& path, const std::string& configFile);
	~InertialSensor_Benchmark();

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

private:
    /// Flag, if data playback from file is finished
    bool                    m_bDone;

    std::string             m_IMUCalibrationFileName;
    std::string             m_IMUCameraCalibrationFileName;


    std::ifstream           m_ImuDatasetFile;
    std::vector<long long>  m_vTimestamps;
    int                     m_nNumOfData;
    int                     m_nCurrentIndex;

    std::string             m_DatasetDir;
    int                     m_nTestIMUSleep;

    std::vector<std::string> split(std::string str, char delimiter);

};

}
