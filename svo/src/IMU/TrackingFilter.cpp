#include "stdafx.h"
#include "TrackingFilter.h"
#include "IMU/Utils/TimeUtils.h"
#include "IMU/Utils/MathUtils.h"

#include <iostream>

namespace slam {

//-----------------------------------------------------------------------------
TrackingFilter::TrackingFilter()
{
	mpImuBuffer = ImuMgr.getImuBuffer();
	//mRci = ImuMgr.getImuToCameraRotation();
	mPci = ImuMgr.getImuToCameraTranslation();

	bInitilized = false;

}

//-----------------------------------------------------------------------------
TrackingFilter::~TrackingFilter()
{
}

//-----------------------------------------------------------------------------
void TrackingFilter::Reset()
{
	bInitilized = false;
}

//-----------------------------------------------------------------------------
void TrackingFilter::init(const VisionPose& visionPose, double timestamp)
{
	ImuEstimator::GetScaleAndGravity(mScale, mGravity); 

	// state covariance
	Eigen::VectorXd pv = Eigen::VectorXd(9, 1);
	pv << 0.1, 0.1, 0.1, 1, 1, 1, 0.1, 0.1, 0.1;
	pv = pv.cwiseProduct(pv);
	P = pv.asDiagonal();

	// vision pose
	const Eigen::Matrix3d& Rvc = visionPose.rot;
	const Eigen::Vector3d& Pvc = visionPose.pos;
	Eigen::Vector3d Pvi = mScale * Pvc + Rvc * mPci;

	// state varibles
	x = Eigen::VectorXd::Zero(9);
	x.block<3, 1>(3, 0) = visionPose.vel; 
	x.block<3, 1>(6, 0) = Pvi;

	// 
	mRefTime = timestamp;	
	mRefVel = visionPose.vel;
	mRefRot = Rvc;
	mRefPos = Pvc;

	mLastGoodPos = Pvc;
}


//-----------------------------------------------------------------------------
void TrackingFilter::fusion(VisionPose& visionPose, const DeltaPose& deltaPose, double timestamp)
{
	ImuEstimator::GetScaleAndGravity(mScale, mGravity);

	if (!bInitilized) {
		init(visionPose, timestamp);
		bInitilized = true;
		return;
	}

	bool bQualityGood = checkVisionQuality(visionPose);

	float dt = timestamp - mRefTime;

	const Eigen::Matrix3d& dRij = deltaPose.deltaRot;
	const Eigen::Vector3d& dPij = deltaPose.deltaPos;
	const Eigen::Vector3d& dVij = deltaPose.deltaVel;

	Eigen::Matrix3d& Rvc_j = visionPose.rot; 
	Eigen::Vector3d& Pvc_j = visionPose.pos;

	Eigen::Matrix3d& Ri = mRefRot;
	Eigen::Matrix3d Rj = Ri * dRij;

	// velocity
	Eigen::Vector3d Pvi_i = mScale * mRefPos + mRefRot * mPci;
	Eigen::Vector3d Pvi_j = mScale * Pvc_j + Rvc_j * mPci;
	Eigen::Vector3d vel_j = (Pvi_j - Pvi_i - Ri * deltaPose.deltaPos) / dt + 
		Ri * deltaPose.deltaVel + mGravity * (dt * 0.5);
	//Eigen::Vector3d vel_j = visionPose.vel; 
	if (!bQualityGood) {
		vel_j.setZero(); 
	}

	// fusion parameters
	static Eigen::MatrixXd F = Eigen::MatrixXd::Identity(9, 9);
	F.block<3,3>(0,0) = dRij.transpose();
	F.block<3,3>(6,3) = Eigen::Matrix3d::Identity() * dt;

	static Eigen::VectorXd u = Eigen::VectorXd::Zero(9);
	u.block<3, 1>(3, 0) = mGravity * dt + Ri * dVij;
	u.block<3, 1>(6, 0) = mGravity * (0.5*dt*dt) + Ri * dPij;

	static Eigen::MatrixXd G = Eigen::MatrixXd::Identity(9, 9);
	G.block<3,3>(3, 3) = Ri;
	G.block<3,3>(6, 6) = Ri;

	const Eigen::MatrixXd& Q = deltaPose.covariance;

	static Eigen::VectorXd z = Eigen::VectorXd::Zero(9);
	if (bQualityGood) {
		mLastGoodPos = Pvc_j;
	}
	z.block<3, 1>(0, 0) = mScale * mLastGoodPos + Rvc_j * mPci;
	z.block<3, 1>(3, 0) = Log_s(Rj.transpose() * Rvc_j);
	z.block<3, 1>(6, 0) = vel_j;

	static Eigen::MatrixXd H = Eigen::MatrixXd::Zero(9, 9);
	H.block<3, 3>(0, 6) = Eigen::Matrix3d::Identity();
	H.block<3, 3>(3, 0) = Eigen::Matrix3d::Identity();
	H.block<3, 3>(6, 3) = Eigen::Matrix3d::Identity();

	static Eigen::MatrixXd V = Eigen::MatrixXd::Identity(9, 9);
	V.block<3, 3>(0, 0) = Eigen::Matrix3d::Identity() * mScale;

	static Eigen::MatrixXd R = Eigen::MatrixXd::Identity(9, 9);
	double vel_coeff = 2e-1 * mScale * mScale / (dt * dt);
	if (bQualityGood) {
		//R = visionPose.invCovariance.inverse();
		for (int i = 0; i < 6; i++) {
			R(i, i) = 1 / visionPose.invCovariance(i, i);
		}
		
		for (int i = 6; i < 9; i++) {
			R(i, i) = R(i - 6, i - 6) * vel_coeff;
		}
	}
	else {
		//R.setIdentity();
		for (int i = 0; i < 3; i++) {
			R(i, i) = 10 / visionPose.invCovariance(i, i);
		}
		for (int i = 3; i < 6; i++) {
			R(i, i) = 1;
		}
		for (int i = 6; i < 9; i++) {
			R(i, i) = R(i - 6, i - 6) * vel_coeff;
		}
	}

	// set x
	//x.block<3, 1>(3, 0) = mRefVel;
	x.block<3, 1>(6, 0) = Pvi_i; 

	// Kalman filter
	KalmanFilter(x, P, F, u, G, Q, z, H, V, R);

	// 
	mRefRot = Rj * so3Exp(x.block<3, 1>(0, 0));
	x.block<3, 1>(0, 0) = Eigen::Vector3d::Zero();
	mRefVel = x.block<3,1>(3,0);
	mRefPos = (x.block<3, 1>(6, 0) - mRefRot * mPci) / mScale;
	mRefTime = timestamp; 

	// set back
	visionPose.rot = mRefRot;
	visionPose.pos = mRefPos;
	visionPose.vel = mRefVel;


	if (SLAM_CONTEXT.debug_log_level >= 1) {
		Eigen::Quaterniond q(mRefRot); 
		LogMgr.fusion_log << (long long)(timestamp * 1e9) << ","
			<< visionPose.pos(0) << "," << visionPose.pos(1) << "," << visionPose.pos(2) << ","
			<< mRefVel(0) << "," << mRefVel(1) << "," << mRefVel(2) << "," 
			<< q.w() << "," << q.x() << "," << q.y() << "," << q.z() << "," 
			<< std::endl;
	}

}

//-----------------------------------------------------------------------------
void TrackingFilter::KalmanFilter(Eigen::VectorXd& x, Eigen::MatrixXd& P, const Eigen::MatrixXd& F,
	const Eigen::VectorXd& u, const Eigen::MatrixXd& G, const Eigen::MatrixXd& Q,
	const Eigen::VectorXd& z, const Eigen::MatrixXd& H, const Eigen::MatrixXd& V, const Eigen::MatrixXd& R)
{
	// predict
	Eigen::VectorXd x_ = F * x + u;
	Eigen::MatrixXd P_ = F * P * F.transpose() + G * Q * G.transpose();

	// update
	Eigen::MatrixXd K = P_ * H.transpose() * (H * P_ * H.transpose() + V * R * V.transpose()).inverse();
	x = x_ + K * (z - H * x_);
	P = (Eigen::MatrixXd::Identity(9,9) - K * H) * P_;
}


//-----------------------------------------------------------------------------
// check vision pose quality, to decide whether send to fusion model
bool TrackingFilter::checkVisionQuality(const VisionPose& visionPose)
{
	// position covariance
	double posCov = visionPose.invCovariance(0, 0) + visionPose.invCovariance(1, 1)
		+ visionPose.invCovariance(2, 2);
	// rotation covariance
	double rotCov = visionPose.invCovariance(3, 3) + visionPose.invCovariance(4, 4)
		+ visionPose.invCovariance(5, 5);

	//printf("posCov = %g, rotCov = %g, quality = %d\n", posCov, rotCov, visionPose.quality);
	//printf("quality = %d\n", visionPose.quality);

	// check quality and covariance
	if (visionPose.quality >= 2) {		// good
		if (posCov < 1e6 || rotCov < 1e6)
			return false;
		else
			return true;
	}

	return false;
}

}
