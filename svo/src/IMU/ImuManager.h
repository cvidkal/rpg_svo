#pragma once

#include "IMU/ImuBuffer.h"

namespace slam {

#define ImuMgr		ImuManager::GetInstance()

/// ImuManager is singleton
class ImuManager
{
private:
	ImuManager();

public:
	static ImuManager& GetInstance();
	~ImuManager();

	void start(); 
	void stop(); 

private:
	// imu
	InertialSensor* mpImu;

	// imu buffer
	ImuBuffer* mpImuBuffer;


public:
	// get imu buffer
	inline ImuBuffer* getImuBuffer() const {
		return mpImuBuffer;
	}

	inline Eigen::Matrix3d getImuToCameraRotation() const {
		//return mpImu->mRci;
		return Eigen::Matrix3d::Identity(); 
	}
	
	inline Eigen::Vector3d getImuToCameraTranslation() const {
		return mpImu->mPci;
	}
};

}
