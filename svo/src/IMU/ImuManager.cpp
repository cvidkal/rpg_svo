#include "stdafx.h"

#include "ImuManager.h"
#include "IMU/InertialSensor/InertialSensor_Dataset.h"
#ifdef PLATFORM_ANDROID
#include "IMU/InertialSensor/InertialSensor_Android.h"
#else
#include "IMU/InertialSensor/InertialSensor_Benchmark.h"
#if USE_IMU_YEI
#include "IMU/InertialSensor/InertialSensor_ThreeSpace.h"
#endif
#endif

namespace slam {

//-----------------------------------------------------------------------------
ImuManager::ImuManager()
{

	// imu. todo: check imu 
	if (SLAM_CONTEXT.video_source == 0) {
		mpImu = new InertialSensor_Dataset(SLAM_CONTEXT.dataset_path + "imu.csv", SLAM_CONTEXT.getImuConfigPath());
	}
    else if (SLAM_CONTEXT.video_source == 3) {
#ifndef PLATFORM_ANDROID
        mpImu = new InertialSensor_Benchmark(SLAM_CONTEXT.dataset_path + "imu.csv", SLAM_CONTEXT.getImuConfigPath());
#endif
    }
	else if(SLAM_CONTEXT.video_source == 6)
	{
		mpImu = new InertialSensor_Dataset(SLAM_CONTEXT.dataset_path + "/imu0/data.csv", SLAM_CONTEXT.getImuConfigPath());
	}
	else {
#ifdef PLATFORM_ANDROID
		mpImu = new InertialSensor_Android(SLAM_CONTEXT.getImuConfigPath());
#else
#if USE_IMU_YEI
		mpImu = new InertialSensor_ThreeSpace(SLAM_CONTEXT.getImuConfigPath());
#endif
#endif
	}

	mpImu->SetCalibrationFile(SLAM_CONTEXT.getImuCalibrationPath());

	// imu buffer
	mpImuBuffer = new ImuBuffer(mpImu);

	
}

//-----------------------------------------------------------------------------
ImuManager& ImuManager::GetInstance()
{
	static ImuManager instance;
	return instance; 
}

//-----------------------------------------------------------------------------
ImuManager::~ImuManager()
{
	delete mpImuBuffer;
	delete mpImu;
}


//-----------------------------------------------------------------------------
void ImuManager::start()
{
	// start imu
	mpImu->Start();
}

//-----------------------------------------------------------------------------
void ImuManager::stop()
{
	// stop imu
	mpImu->Stop();
}

}
