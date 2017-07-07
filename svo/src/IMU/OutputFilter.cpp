#include "stdafx.h"

#include "OutputFilter.h"
//#include "IMU/Utils/TimeUtils.h"
#include "IMU/Utils/MathUtils.h"
#include "ImuManager.h"

#include <iostream>

namespace slam {

//-----------------------------------------------------------------------------
OutputFilter::OutputFilter()
{
	mpImuBuffer = ImuMgr.getImuBuffer();
	//mRci = ImuMgr.getImuToCameraRotation();
	mPci = ImuMgr.getImuToCameraTranslation();

	mpOutputBuffer = &(OutputBuffer::GetInstance());

	mbNewFusionPose = false;

	mCovariance = Eigen::MatrixXd::Zero(9, 9); 

	// covariance
	Eigen::VectorXd pv = Eigen::VectorXd(6, 1);
#ifdef PLATFORM_ANDROID
	pv << 2e-4, 2e-4, 2e-4, 5e-3, 5e-3, 5e-3;
#else
	pv << 2e-5, 2e-5, 2e-5, 5e-4, 5e-4, 5e-4;
#endif
	//pv = pv.cwiseProduct(pv);
	mCovNoise = pv.asDiagonal();

	reset(); 

	mbDone = false;
	mOutputThread = std::thread([&] {this->runOutputThread(); });

	mpImuBuffer->AddListener(this); 
}

//-----------------------------------------------------------------------------
OutputFilter::~OutputFilter()
{
	mbDone = true;
	mOutputCondvar.notify_all(); 
	mOutputThread.join(); 
}

//-----------------------------------------------------------------------------
void OutputFilter::reset()
{
	mbInitilized = false;
	mbHaveEstimateGravity = false;
}

//-----------------------------------------------------------------------------
void OutputFilter::init(const FusionPose& fusionPose, double timestamp)
{
	//ImuEstimator::GetScaleAndGravity(mScale, mGravity); 

	// state covariance
	Eigen::VectorXd pv = Eigen::VectorXd(9, 1);
	pv << 0.1, 0.1, 0.1, 1, 1, 1, 0.1, 0.1, 0.1;
	pv = pv.cwiseProduct(pv);
	P = pv.asDiagonal();

	// state varibles
	x = Eigen::VectorXd::Zero(9); 

	double dt = mDeltaPose.deltaTime;
	mRot = fusionPose.rot * mDeltaPose.deltaRot;
	mVel = fusionPose.vel + fusionPose.rot * mDeltaPose.deltaVel + mGravity * dt;
	mPos = fusionPose.pos + fusionPose.rot * mDeltaPose.deltaPos + fusionPose.vel * dt
		+ mGravity * (0.5 * dt * dt);
	mCovariance = fusionPose.covariance;

	mLastRot = mRot;
	mLastFusionTime = timestamp - (long long)(0.016 * 1e9); 
}

//-----------------------------------------------------------------------------
void OutputFilter::Fusion(const ImuData& imuData, FusionPose& fusionPose, long long timestamp)
{
	double dt = timestamp - mLastFusionTime; 

	Eigen::Matrix3d& Ri = mLastRot;
	Eigen::Matrix3d& Rj = mRot;

	// fusion parameters
	static Eigen::MatrixXd F = Eigen::MatrixXd::Identity(9, 9);
	F.block<3,3>(0,0) = Rj.transpose() * Ri;
	F.block<3,3>(6,3) = Eigen::Matrix3d::Identity() * dt;

	Eigen::MatrixXd& Q = mCovariance;

	static Eigen::VectorXd z = Eigen::VectorXd::Zero(9);
	z.block<3, 1>(0, 0) = Log_s(Rj.transpose() * fusionPose.rot);
	z.block<3, 1>(3, 0) = fusionPose.vel;
	z.block<3, 1>(6, 0) = fusionPose.pos;

	Eigen::MatrixXd R = fusionPose.covariance * 4;

	// Kalman filter
	x.block<3, 1>(0, 0) = Eigen::Vector3d::Zero();
	x.block<3, 1>(3, 0) = mVel;
	x.block<3, 1>(6, 0) = mPos;
	KalmanFilter(x, P, F, Q, z, R);

	// 
	mLastRot = mRot; 
	mRot = Rj * so3Exp(x.block<3, 1>(0, 0));
	mVel = x.block<3,1>(3,0);
	mPos = x.block<3,1>(6,0);

	mLastFusionTime = timestamp;
	mLastFusedPos = mPos;
	mRvw = mRot * imuData.q.cast<double>().matrix().transpose();
}

//-----------------------------------------------------------------------------
void OutputFilter::KalmanFilter(Eigen::VectorXd& x, Eigen::MatrixXd& P, 
	const Eigen::MatrixXd& F, const Eigen::MatrixXd& Q, 
	const Eigen::VectorXd& z, const Eigen::MatrixXd& R)
{
	// predict
	Eigen::VectorXd& x_ = x; // not F * x + u, because x has predicted incrementally 
	Eigen::MatrixXd P_ = F * P * F.transpose() + Q;

	// update
	Eigen::MatrixXd K = P_ * (P_ + R).inverse();
	x = x_ + K * (z - x_);
	P = P_ - K * P_;
}


//-------------------------------------------------------------------------------
void OutputFilter::onImuDataAdded(const ImuData& imuData)
{
	std::lock_guard<std::mutex> ulock(mOutputMutex);
	mpLastImuData = &imuData;
	mOutputCondvar.notify_all(); 
}


//-----------------------------------------------------------------------------
void OutputFilter::runOutputThread()
{
	ImuData imuData;
	FusionPose fusionPose;
	long long timestamp; 
	Eigen::Matrix3d Rvc;
	Eigen::Vector3d Pvc, Vvc, Avc, Gvc;
	while (!mbDone) {
		{
			std::unique_lock<std::mutex> ulock(mOutputMutex);
			mOutputCondvar.wait(ulock);
			if (mbDone)
				break;

			imuData = *mpLastImuData;
		}

		timestamp = imuData.timestamp;
		static double lastTimestamp = imuData.timestamp;
		double dt = timestamp - lastTimestamp;

		if (mbHaveEstimateGravity) {
			// calculate next output pose and covariance
			incrementalIntegrate(imuData, dt);

			// get vision pose
			if (getFusionPose(fusionPose, timestamp)) {
				// fusion here
				Fusion(imuData, fusionPose, timestamp);
			}
			Rvc = mRot;
			Pvc = (mPos - Rvc * mPci) / mScale;
			Vvc = mVel / mScale; 

			// linear acceleration
			Avc = mRot * imuData.acc.cast<double>() + mGravity;
			Gvc = mRot * imuData.gyro.cast<double>();

			// add to output buffer
			Eigen::Quaterniond q(Rvc);
			//mOutputBuffer.AddNode(timestamp, Pvc, Vvc, q, 2);
			mpOutputBuffer->addNode(timestamp, Pvc, Vvc, Avc, Gvc, q, 2);
		}
		else {
			// get vision pose
			if (getFusionPose(fusionPose, timestamp)) {
				// add to output buffer
				Eigen::Quaterniond q(fusionPose.rot); 
				mpOutputBuffer->addNode(timestamp, fusionPose.pos, fusionPose.vel, 
					q, 2);
			}
		}

		//
		lastTimestamp = timestamp;
	}
}


//-----------------------------------------------------------------------------
bool OutputFilter::getFusionPose(FusionPose& fusionPose, double timestamp)
{
	if (!mbNewFusionPose)
		return false;

	// 
	std::lock_guard<std::mutex> ulock(mVisionMutex);

	// get delta pose
	mDeltaPose.startTime = mDeltaPose.endTime;
	mDeltaPose.endTime = timestamp; 
	mpImuBuffer->ImuPreIntegrate(mDeltaPose, false, false); 

	if (mbHaveEstimateGravity) {
		// predict fusion pose to the current imu time
		double dt = mDeltaPose.deltaTime;
		fusionPose.rot = mFusionPose.rot * mDeltaPose.deltaRot;
		fusionPose.vel = mFusionPose.vel + mFusionPose.rot * mDeltaPose.deltaVel + mGravity * dt;
		fusionPose.pos = mFusionPose.pos + mFusionPose.rot * mDeltaPose.deltaPos + mFusionPose.vel * dt
			+ mGravity * (0.5 * dt * dt);
		fusionPose.covariance = mFusionPose.covariance;
	}
	else {
		fusionPose.rot = mFusionPose.rot.transpose();
		fusionPose.pos = -fusionPose.rot * mFusionPose.pos;
		fusionPose.vel = -fusionPose.rot * mFusionPose.vel;
		fusionPose.covariance = mFusionPose.covariance;
	}

	// finally, set mbNewVisionPose
	mbNewFusionPose = false;

	return true;
}


//-----------------------------------------------------------------------------
void OutputFilter::setFusionPose(const FusionPose& fusionPose, const DeltaPose& deltaPose, 
	double timestamp, bool bHaveEstimateGravity)
{
	// get scale and gravity
	ImuEstimator::GetScaleAndGravity(mScale, mGravity);
	// get imu bias
	ImuEstimator::GetImuBias(mGyroBias, mAccBias);

	std::lock_guard<std::mutex> ulock(mVisionMutex); 

	mFusionPose = fusionPose;

	if (!mbInitilized) {
		mbInitilized = true;

		mDeltaPose.Reset();
		mDeltaPose.startTime = mDeltaPose.endTime = timestamp;
	}
	else {
		//// substract deltaPose
		//mDeltaPose.deltaTime -= deltaPose.deltaTime;
		//double dt = TimestampToSecond(mDeltaPose.deltaTime);
		//mDeltaPose.deltaRot = deltaPose.deltaRot.transpose() * mDeltaPose.deltaRot;
		//mDeltaPose.deltaVel = deltaPose.deltaRot.transpose() * (mDeltaPose.deltaVel - deltaPose.deltaVel);
		//mDeltaPose.deltaPos = deltaPose.deltaRot.transpose() * (mDeltaPose.deltaPos - deltaPose.deltaPos 
		//	- deltaPose.deltaVel * dt);

		mDeltaPose.Reset();
		mDeltaPose.startTime = mDeltaPose.endTime = timestamp;
	}

	// preintegrate delta pose to latest imu
	mDeltaPose.startTime = mDeltaPose.endTime;
	mDeltaPose.endTime = mpImuBuffer->GetHeadTimestamp();
	mpImuBuffer->ImuPreIntegrate(mDeltaPose, false, false); 

	if (mbHaveEstimateGravity == false && bHaveEstimateGravity == true) {
		mbHaveEstimateGravity = true;

		init(fusionPose, timestamp);
	}

	// finally, set mbNewVisionPose
	mbNewFusionPose = true; 
}


//-----------------------------------------------------------------------------
void OutputFilter::incrementalIntegrate(const ImuData& imuData, double dt)
{
	Eigen::Vector3d acc = imuData.acc.cast<double>() - mAccBias;
	Eigen::Vector3d gyro = imuData.gyro.cast<double>() - mGyroBias;

	Eigen::Matrix3d ddRot = so3Exp(gyro * dt);

	/// some common matrix
	// Jr * dt
	Eigen::Matrix3d Jr_dt = VectorToJacobianMatrix(gyro * dt) * dt;
	// - Rik * (ak - ba)^ * dt
	Eigen::Matrix3d Ra_dt = mRot * VectorToCrossFormMatrix<double>(acc * (-dt));
	//  - 1/2 * Rik * (ak - ba)^ * dt^2
	Eigen::Matrix3d Ra_dtdt = Ra_dt * (0.5 * dt);

	// covariance matrix
	static Eigen::MatrixXd A = Eigen::MatrixXd::Zero(9, 9);
	static Eigen::MatrixXd B = Eigen::MatrixXd::Zero(9, 6);
	A.block<3, 3>(3, 3) = Eigen::Matrix3d::Identity();
	A.block<3, 3>(6, 6) = Eigen::Matrix3d::Identity();
	A.block<3, 3>(0, 0) = ddRot.transpose();
	A.block<3, 3>(3, 0) = Ra_dt;
	A.block<3, 3>(6, 0) = Ra_dtdt;
	A.block<3, 3>(6, 3) = Eigen::Matrix3d::Identity() * dt;

	B.block<3, 3>(0, 0) = Jr_dt;
	B.block<3, 3>(3, 3) = mRot * dt;
	B.block<3, 3>(6, 3) = mRot * (0.5 * dt * dt);

	//// todo: use sparse matrix to reduce computation
	mCovariance = A * mCovariance * A.transpose() + B * mCovNoise * B.transpose();

	/// calculate next output pose
	mPos += mVel * dt + (mRot * acc + mGravity) * (0.5 * dt * dt);
	mVel += mRot * acc * dt + mGravity * dt;
	mRot = mRot * ddRot;
}


}