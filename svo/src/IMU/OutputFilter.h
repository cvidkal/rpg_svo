#pragma once

#include <Eigen/Dense>
#include "IMU/Utils/TypeDefs.h"
#include "ImuBuffer.h"
#include "ImuEstimator.h"
#include "OutputBuffer.h"

#include <thread>
#include <mutex>
#include <condition_variable>

namespace slam {

class OutputFilter
	: public ImuBuffer::Listener
{
public:
	OutputFilter();
	~OutputFilter();

	void reset(); 

private:
	bool mbInitilized;

	ImuBuffer* mpImuBuffer;
	OutputBuffer* mpOutputBuffer;

	// relationship between imu and camera frame
	//Eigen::Matrix3d mRci;
	Eigen::Vector3d mPci;

	// gravity vector in world frame
	Eigen::Vector3d mGravity;
	// vision scale
	double mScale;

	// imu bias
	Eigen::Vector3d mGyroBias;
	Eigen::Vector3d mAccBias;

	bool mbHaveEstimateGravity; 


private:
	void Fusion(const ImuData& imuData, FusionPose& fusionPose, long long timestamp);

	double mLastFusionTime;

//--- kalman filter
private:
	Eigen::VectorXd x;
	Eigen::MatrixXd P;
	
	void KalmanFilter(Eigen::VectorXd& x, Eigen::MatrixXd& P, const Eigen::MatrixXd& F,
		const Eigen::MatrixXd& Q, const Eigen::VectorXd& z, const Eigen::MatrixXd& R); 

private:
	void init(const FusionPose& fusionPose, double timestamp);


private:
	/// Implementation of imu buffer listener
	void onImuDataAdded(const ImuData& imuData);


private:
	// multi thread
	bool mbDone;
	std::thread mOutputThread;
	std::mutex mOutputMutex;
	std::condition_variable mOutputCondvar;

	void runOutputThread();


	const ImuData* mpLastImuData;

	Eigen::Matrix3d mLastRot; 
	Eigen::Matrix3d mRot;
	Eigen::Vector3d mVel;
	Eigen::Vector3d mPos; 
	Eigen::MatrixXd mCovariance;
	Eigen::MatrixXd mCovNoise;

	Eigen::Vector3d mLastFusedPos;
	Eigen::Matrix3d mRvw;

	void incrementalIntegrate(const ImuData& imuData, double dt);

	// vision pose
	std::mutex mVisionMutex;
	DeltaPose mDeltaPose;
	FusionPose mFusionPose;
	bool mbNewFusionPose; 
	bool getFusionPose(FusionPose& fusionPose, double timestamp);

public:
	void setFusionPose(const FusionPose& fusionPose, const DeltaPose& deltaPose, 
		double timestamp, bool bHaveEstimateGravity);

};


}
