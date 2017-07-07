#pragma once

#include <Eigen/Dense>
#include "IMU/Utils/TypeDefs.h"
#include "ImuBuffer.h"
#include "ImuEstimator.h"
#include "ImuManager.h"
#include "sophus/se3.h"


namespace slam {

class TrackingFilter
{
public:
	TrackingFilter();
	~TrackingFilter();

	void Reset(); 

private:
	bool bInitilized;

	ImuBuffer* mpImuBuffer;

	// relationship between imu and camera frame
	//Eigen::Matrix3d mRci;
	Eigen::Vector3d mPci;

	// gravity vector in vision frame
	Eigen::Vector3d mGravity;
	// vision scale
	double mScale;


//--- predict with imu
public:
	void fusion(VisionPose& visionPose, const DeltaPose& deltaPose, double timestamp);


//--- kalman filter
private:
	Eigen::VectorXd x;
	Eigen::MatrixXd P;
	
	void KalmanFilter(Eigen::VectorXd& x, Eigen::MatrixXd& P, const Eigen::MatrixXd& F,
		const Eigen::VectorXd& u, const Eigen::MatrixXd& G, const Eigen::MatrixXd& Q,
		const Eigen::VectorXd& z, const Eigen::MatrixXd& H, const Eigen::MatrixXd& V, const Eigen::MatrixXd& R);

//--- tracking reference, from camera frame to vision frame
private:
	Eigen::Matrix3d mRefRot;		// Rvc
	Eigen::Vector3d mRefVel;		// Vvi
	Eigen::Vector3d mRefPos;		// Pvc
	double mRefTime;

	Eigen::Vector3d mLastGoodPos;

private:
	void init(const VisionPose& visionPose, double timestamp);

private:
	bool checkVisionQuality(const VisionPose& visionPose);

};

}
