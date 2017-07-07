#include "stdafx.h"

#include "ImuEstimator.h"
#include <iostream>

#include "IMU/Utils/MathUtils.h"
#include "opencv2/opencv.hpp"
#include <fstream>

#define USE_ITERATIVE_LS		1
#define _GRAVITY				9.81f

namespace slam {

// static member variables
Eigen::Vector3d ImuEstimator::mGravity;
double ImuEstimator::mScale = 1;
Eigen::Matrix3d ImuEstimator::mRvw;

bool ImuEstimator::mbHaveEstimateScale = false;

// gyro bias
Eigen::Vector3d ImuEstimator::mGyroBias = Eigen::Vector3d::Zero();
// acc bias
Eigen::Vector3d ImuEstimator::mAccBias = Eigen::Vector3d::Zero();

std::mutex ImuEstimator::mBiasMutex;


//-----------------------------------------------------------------------------
ImuEstimator::ImuEstimator()
{
	mbUseImu = SLAM_CONTEXT.use_imu;
	if (mbUseImu) {
		mpImuBuffer = ImuMgr.getImuBuffer();
		//mRci = ImuMgr.getImuToCameraRotation();
		mPci = ImuMgr.getImuToCameraTranslation();
	}

	// 
	Eigen::Matrix3d Rvw;
	Rvw << 1, 0, 0, 0, 0, -1, 0, 1, 0;
	SetWorldToVisionRotation(Rvw);

	mpBiasThread = nullptr;
	mbThreadReset = true;
}

//-----------------------------------------------------------------------------
ImuEstimator::~ImuEstimator()
{
	StopEstimateBias(); 
}


//-----------------------------------------------------------------------------
void ImuEstimator::Reset()
{
	if (!mbUseImu)
		return; 

	mbThreadReset = true;

	// start bias thread
	StartEstimateBias(); 
}


//-----------------------------------------------------------------------------
void ImuEstimator::GetImuBias(Eigen::Vector3d& gyroBias, Eigen::Vector3d& accBias)
{
	mBiasMutex.lock();

	gyroBias = mGyroBias;
	accBias = mAccBias;

	mBiasMutex.unlock();
}

//-----------------------------------------------------------------------------
void ImuEstimator::SetImuBias(const Eigen::Vector3d& gyroBias, const Eigen::Vector3d& accBias)
{
	mBiasMutex.lock();

	mGyroBias = gyroBias;
	mAccBias = accBias;

	mBiasMutex.unlock();
}

//-----------------------------------------------------------------------------
// get scale and gravity
void ImuEstimator::GetScaleAndGravity(double& scale, Eigen::Vector3d& gravity)
{
	mBiasMutex.lock();

	scale = mScale;
	gravity = mGravity;

	mBiasMutex.unlock();
}


//-----------------------------------------------------------------------------
// set scale and gravity
void ImuEstimator::SetScale(double scale)
{
	mBiasMutex.lock();

	mScale = scale;

	mBiasMutex.unlock();
}

//-----------------------------------------------------------------------------
// get imu world to vision frame rotation matrix 
void ImuEstimator::GetWorldToVisionRotation(Eigen::Matrix3d& Rvw)
{
	mBiasMutex.lock();

	Rvw = mRvw;

	mBiasMutex.unlock();
}

//-----------------------------------------------------------------------------
// set imu world to vision frame rotation matrix 
void ImuEstimator::SetWorldToVisionRotation(const Eigen::Matrix3d& Rvw)
{
	mBiasMutex.lock();

	mRvw = Rvw;
	mGravity = mRvw * Eigen::Vector3d(0, 0, -_GRAVITY); 

	mBiasMutex.unlock();
}


// start bias thread
void ImuEstimator::StartEstimateBias()
{
	// start thread
	mbThreadDone = false;
	if (mpBiasThread == nullptr) {
		mpBiasThread = new std::thread([&] {this->RunBiasThread(); });
	}
}


// stop bias thread
void ImuEstimator::StopEstimateBias()
{
	mbThreadDone = true;
	if (mpBiasThread != nullptr) {
		mBiasCondvar.notify_all();
		mpBiasThread->join();
		delete mpBiasThread;
		mpBiasThread = nullptr;
	}
}


//-----------------------------------------------------------------------------
// check vision pose quality, to decide whether send to fusion model
bool ImuEstimator::CheckVisionQuality(const VisionPose& visionPose)
{
	// check quality
	if (visionPose.quality == 3) {			// Great
		return true;
	}

	return false;
}


//-----------------------------------------------------------------------------
// get and check ahrs pose
bool ImuEstimator::GetAndCheckAhrsPose(AHRSPose& ahrsPose, double timestamp)
{
	static ImuData imuData;

	// get ahrs pose
	mpImuBuffer->GetDataByTimestamp(timestamp, imuData);

	// check ahrs pose
	if (imuData.gyro.norm() < 0.5 && std::abs(imuData.acc.norm() - _GRAVITY) < 0.5)
	{
		ahrsPose.rot = imuData.q.cast<double>().matrix();
		return true;
	}

	return false;
}


//-----------------------------------------------------------------------------
void ImuEstimator::ResetThreadVariables()
{
	// reset thread variables
	_node.Reset();

	_bReady = false;
	mbHaveEstimateScale = false;
	_nReadyCnt = 0;

	_bFirstTime_gyro = true;
	_Q_gyro = Eigen::Matrix3d::Identity() * 1e-5;

	_bFirstTime_acc = true;

	_nAhrsCnt = 1;

	_visionPose.quality = 0;
}

//-----------------------------------------------------------------------------
void ImuEstimator::notifyToEstimateBias(const VisionPose& visionPose, double timestamp)
{
	if (!mbUseImu)
		return;

	std::lock_guard<std::mutex> ulock(mThreadMutex);
	mVisionPose_Bias = visionPose;
	mTimestamp_Bias = timestamp;

	mBiasCondvar.notify_all();
}


//-----------------------------------------------------------------------------
void ImuEstimator::RunBiasThread()
{
	VisionPose _lastVisionPose;

	while (!mbThreadDone) {
		if (mbThreadReset) {
			ResetThreadVariables();
			mbThreadReset = false;
		}

		{
			std::unique_lock<std::mutex> ulock(mThreadMutex);
			// wait until new vision data arrives
			mBiasCondvar.wait(ulock);

			if (mbThreadDone)
				break; 

			_lastVisionPose = _visionPose; 
			_visionPose = mVisionPose_Bias;
			_timestamp = mTimestamp_Bias;
		}

		if (!mbHaveEstimateScale && _nAhrsCnt <= 10) {
			// refine gravity
			AHRSPose ahrsPose;
			if (CheckVisionQuality(_visionPose) && GetAndCheckAhrsPose(ahrsPose, _timestamp)) {
				EstimateGravity(_visionPose, ahrsPose, _nAhrsCnt);
				printf("mGravity: %.6f, %.6f, %.6f\n", mGravity(0), mGravity(1), mGravity(2)); 
			}
		}

		if (!mvBiasNode.empty() && (_timestamp - mvBiasNode[0].startTime) > 15) {
			_node.Reset();
			mvBiasNode.clear();
			continue;
		}

		_node.cnt++;
		if (_node.cnt == 1) {
			if (!CheckVisionQuality(_visionPose)) {
				_node.cnt = 0;
				continue;
			}

			//printf("add start node\n");

			_node.startTime = _timestamp;
			_node.visionStart = _visionPose;

			_node.dPose.Reset();
			_node.dPose.startTime = _timestamp;
			_node.dPose.endTime = _timestamp;

			_bReady = false;
		}
		else {
			_node.dPose.startTime = _node.dPose.endTime;
			_node.dPose.endTime = _timestamp;

			double dt = _timestamp - _node.startTime;
			if (dt > 0.5) {
				// drop
				_node.Reset();
				mvBiasNode.clear();
				continue;
			}

			mpImuBuffer->ImuPreIntegrate(_node.dPose, true, false);
			if (dt > 0.15 && CheckVisionQuality(_visionPose) && CheckVisionQuality(_lastVisionPose)) {
				//printf("add end node\n");

				// add node to vector
				_node.endTime = _timestamp;
				_node.visionEnd = _visionPose;
				mvBiasNode.push_back(_node);

				// set as start of next node 
				_node.Reset();
				_node.startTime = _timestamp;
				_node.visionStart = _visionPose;
				_node.dPose.startTime = _timestamp;
				_node.dPose.endTime = _timestamp;
				_node.cnt = 1;

				int minNum = 15;
				if (!mbHaveEstimateScale) {
					minNum = 20;
				}
				if (mvBiasNode.size() >= minNum || (mvBiasNode.size() > minNum - 5
					&& (_timestamp - mvBiasNode[0].startTime) > minNum * 0.2)
					)
				{
					_bReady = true;
				}

			}

		}

		// if ready, estimate bias
		if (_bReady) {
			_nReadyCnt++;

			static Eigen::Vector3d dGyroBias, dAccBias, gravity;
			static double scale;
			static Eigen::Matrix3d Rvw;

			// estimate gyro bias
			EstimateGyroBias(dGyroBias, _P_gyro, _Q_gyro, _bFirstTime_gyro);

			// estimate scale and gravity
			if (EstimateScaleAndGravity(dGyroBias, scale, gravity, Rvw, 
				dAccBias, _P_acc, _Q_acc, _bFirstTime_acc)) 
			{
				SetScale(scale);
				SetWorldToVisionRotation(Rvw); 
				mbHaveEstimateScale = true;
			}

			// set imu bias
			Eigen::Vector3d gyro_bias, acc_bias;
			GetImuBias(gyro_bias, acc_bias);
			gyro_bias += dGyroBias;
			dGyroBias.setZero();
#if USE_ITERATIVE_LS
			acc_bias += dAccBias;
#endif
			dAccBias.setZero();
			SetImuBias(gyro_bias, acc_bias);

			// log to file
			if (SLAM_CONTEXT.debug_log_level >= 1) {
				LogMgr.imu_estimator_log << (long long)(_timestamp * 1e9) << "," << mScale << ","
					<< mGravity(0) << "," << mGravity(1) << "," << mGravity(2) << ","
					<< gyro_bias(0) << "," << gyro_bias(1) << "," << gyro_bias(2) << ","
					<< acc_bias(0) << "," << acc_bias(1) << "," << acc_bias(2) << ","
					<< std::endl; 
			}

			// reset
			mvBiasNode.clear();
			_node.Reset();
			_bReady = false;
		}
	}
}


// estimate gyro bias
bool ImuEstimator::EstimateGyroBias(Eigen::Vector3d& dGyroBias, Eigen::Matrix3d& P, 
	Eigen::Matrix3d& Q, bool& bFirstTime)
{
	int N = mvBiasNode.size();
	Eigen::Matrix3d Ri, Rj, dRij, JRg;
	Eigen::MatrixXd H(3 * N, 3);
	Eigen::VectorXd z(3 * N);
	// estimate gyro bias
	for (int i = 0; i < N; i++) {
		BiasNode& tempNode = mvBiasNode[i];
		Ri = tempNode.visionStart.rot;
		Rj = tempNode.visionEnd.rot;
		dRij = tempNode.dPose.deltaRot;
		JRg = tempNode.dPose.dRg;

		H.block<3, 3>(i * 3, 0) = JRg;
		z.block<3, 1>(i * 3, 0) = Log_s(dRij.transpose() * Ri.transpose() * Rj);
	}

	Eigen::MatrixXd J = H.transpose() * H;
	// todo: check J

	if (bFirstTime) {
		P = J.inverse();
		dGyroBias = P * (H.transpose() * z);

		bFirstTime = false;
	}
	else {
		double dt = mvBiasNode[N - 1].endTime - mvBiasNode[0].startTime;
		P = ((P + Q * dt).inverse() + J).inverse();
		dGyroBias = dGyroBias + P * (H.transpose() * z - J * dGyroBias); 
	}
	
	printf("gyro bias:%.6f,%.6f,%.6f; ", dGyroBias(0), dGyroBias(1), dGyroBias(2));

	return true;
}

// estimate scale, gravity and acc bias
bool ImuEstimator::EstimateScaleAndGravity(const Eigen::Vector3d& dGyroBias,
	double& scale, Eigen::Vector3d& gravity, Eigen::Matrix3d& Rvw, Eigen::Vector3d& dAccBias,
	Eigen::MatrixXd& P, Eigen::MatrixXd& Q, bool& bFirstTime)
{
	// todo: check whether motion is enough

	Eigen::Vector3d bg = dGyroBias;
	//bg << 0, 0, 0;

	Eigen::Vector3d gw;
	gw << 0, 0, -_GRAVITY;

	int nNode = mvBiasNode.size();
	int N = 0;
	for (int i = 0; i < nNode - 1; i++) {
		BiasNode& node_1 = mvBiasNode[i];
		BiasNode& node_2 = mvBiasNode[i + 1];

		if (node_1.endTime != node_2.startTime)
			continue;
		N++;
	}
	//printf("nNode = %d, N = %d\n", nNode, N); 

	Eigen::Matrix3d R_cw_1, R_cw_2, R_cw_3;
	Eigen::Vector3d P_cw_1, P_cw_2, P_cw_3;
	Eigen::Matrix3d R_iw_1, R_iw_2;

	Eigen::MatrixXd H(3 * N, 4);
	Eigen::VectorXd z(3 * N);

	// estimate scale bias and gravity
	if (true) {
		int index = 0;
		for (int i = 0; i < nNode - 1; i++) {
			BiasNode& node_1 = mvBiasNode[i];
			BiasNode& node_2 = mvBiasNode[i + 1];

			if (node_1.endTime != node_2.startTime)
				continue; 

			double dt12 = node_1.endTime - node_1.startTime;
			double dt23 = node_2.endTime - node_2.startTime;

			R_cw_1 = node_1.visionStart.rot;
			R_cw_2 = node_1.visionEnd.rot;
			R_cw_3 = node_2.visionEnd.rot;

			P_cw_1 = node_1.visionStart.pos;
			P_cw_2 = node_1.visionEnd.pos;
			P_cw_3 = node_2.visionEnd.pos;

			R_iw_1 = R_cw_1;
			R_iw_2 = R_cw_2;

			Eigen::Matrix3d& dRot_12 = node_1.dPose.deltaRot;
			Eigen::Vector3d& dVel_12 = node_1.dPose.deltaVel;
			Eigen::Vector3d& dPos_12 = node_1.dPose.deltaPos;
			Eigen::Matrix3d& dRot_23 = node_2.dPose.deltaRot;
			Eigen::Vector3d& dVel_23 = node_2.dPose.deltaVel;
			Eigen::Vector3d& dPos_23 = node_2.dPose.deltaPos;

			Eigen::Matrix3d& Jpg12 = node_1.dPose.dPg;
			Eigen::Matrix3d& Jvg12 = node_1.dPose.dVg;
			Eigen::Matrix3d& Jpg23 = node_2.dPose.dPg;
			Eigen::Matrix3d& Jvg23 = node_2.dPose.dVg;

			Eigen::Vector3d alpha = (P_cw_2 - P_cw_1) * dt23 - (P_cw_3 - P_cw_2) * dt12;
			Eigen::Matrix3d beta = 0.5 * dt12 * dt23 * (dt12 + dt23) * Eigen::Matrix3d::Identity();
			Eigen::Vector3d gamma = (R_cw_3 - R_cw_2) * mPci * dt12 - (R_cw_2 - R_cw_1) * mPci * dt23
				- R_iw_1 * (dVel_12 + Jvg12 * bg) * dt12 * dt23
				+ R_iw_1 * (dPos_12 + Jpg12 * bg) * dt23 - R_iw_2 * (dPos_23 + Jpg23 * bg) * dt12;

			H.block<3, 1>(index * 3, 0) = alpha;
			H.block<3, 3>(index * 3, 1) = beta;
			z.block<3, 1>(index * 3, 0) = gamma;
			index++; 
		}

		Eigen::VectorXd y = (H.transpose() * H).inverse() * (H.transpose() * z);

		scale = y(0);
		gravity = y.block<3, 1>(1, 0);

		//std::cout << "scale: " << scale << std::endl;
		//std::cout << "gravity: " << gravity << std::endl;

		// refine gravity
		double g_norm = gravity.norm();
		Eigen::Vector3d v = gw.cross(gravity / g_norm);
		double theta = atan2(v.norm(), gw.dot(gravity / g_norm));
		// world frame to vision frame
		Rvw = so3Exp(v / v.norm() * theta);
	}
	else {
		GetWorldToVisionRotation(Rvw);
	}

	// estimate scale bias, gravity and acc bias
	H.resize(3 * N, 6);
	for (int i = 0; i < nNode - 1; i++) {
		BiasNode& node_1 = mvBiasNode[i];
		BiasNode& node_2 = mvBiasNode[i + 1];

		if (node_1.endTime != node_2.startTime)
			continue;

		double dt12 = node_1.endTime - node_1.startTime;
		double dt23 = node_2.endTime - node_2.startTime;

		R_cw_1 = node_1.visionStart.rot;
		R_cw_2 = node_1.visionEnd.rot;
		R_cw_3 = node_2.visionEnd.rot;

		P_cw_1 = node_1.visionStart.pos;
		P_cw_2 = node_1.visionEnd.pos;
		P_cw_3 = node_2.visionEnd.pos;

		R_iw_1 = R_cw_1;
		R_iw_2 = R_cw_2;

		Eigen::Matrix3d& dRot_12 = node_1.dPose.deltaRot;
		Eigen::Vector3d& dVel_12 = node_1.dPose.deltaVel;
		Eigen::Vector3d& dPos_12 = node_1.dPose.deltaPos;
		Eigen::Matrix3d& dRot_23 = node_2.dPose.deltaRot;
		Eigen::Vector3d& dVel_23 = node_2.dPose.deltaVel;
		Eigen::Vector3d& dPos_23 = node_2.dPose.deltaPos;

		Eigen::Matrix3d& Jp12 = node_1.dPose.dPa;
		Eigen::Matrix3d& Jv12 = node_1.dPose.dVa;
		Eigen::Matrix3d& Jp23 = node_2.dPose.dPa;
		Eigen::Matrix3d& Jv23 = node_2.dPose.dVa;

		Eigen::Matrix3d& Jpg12 = node_1.dPose.dPg;
		Eigen::Matrix3d& Jvg12 = node_1.dPose.dVg;
		Eigen::Matrix3d& Jpg23 = node_2.dPose.dPg;
		Eigen::Matrix3d& Jvg23 = node_2.dPose.dVg;

		Eigen::Vector3d alpha = (P_cw_2 - P_cw_1) * dt23 - (P_cw_3 - P_cw_2) * dt12;
		Eigen::Matrix3d beta = -0.5 * dt12 * dt23 * (dt12 + dt23) * Rvw * VectorToCrossFormMatrix(gw);
		Eigen::Matrix3d gamma = R_iw_2 * Jp23 * dt12 - R_iw_1 * Jp12 * dt23 + R_iw_1 * Jv12 * dt12 * dt23;
		Eigen::Vector3d zeta = (R_cw_3 - R_cw_2) * mPci * dt12 - (R_cw_2 - R_cw_1) * mPci * dt23
			- R_iw_1 * (dVel_12 + Jvg12 * bg) * dt12 * dt23
			+ R_iw_1 * (dPos_12 + Jpg12 * bg) * dt23 - R_iw_2 * (dPos_23 + Jpg23 * bg) * dt12
			- 0.5 * Rvw * gw * dt12 * dt23 * (dt12 + dt23);

		H.block<3, 1>(i * 3, 0) = alpha;
		H.block<3, 2>(i * 3, 1) = beta.block<3, 2>(0, 0);
		H.block<3, 3>(i * 3, 3) = gamma;
		z.block<3, 1>(i * 3, 0) = zeta;
	}

	Eigen::MatrixXd J = H.transpose() * H;
	// todo: check J

	Eigen::VectorXd x = Eigen::VectorXd::Zero(6);
	if (bFirstTime) {
#if USE_ITERATIVE_LS
		bFirstTime = false;
#endif

		P = J.inverse();
		x = P * (H.transpose() * z);
	}
	else {
		x(0) = scale;
		x.block<3, 1>(3, 0) = dAccBias;

		P = (P.inverse() + J).inverse();
		x = x + P * (H.transpose() * z - J * x);
	}

	// scale
	scale = x(0);
	// gravity
	Eigen::Vector3d dTheta;
	dTheta << x(1), x(2), 0;
	Rvw = Rvw * so3Exp(dTheta);
	gravity = Rvw * gw;
	// acc bias
	dAccBias = x.block<3, 1>(3, 0);

	printf("scale:%.6f; gravity:%.6f,%.6f,%.6f; acc bias:%.6f,%.6f,%.6f\n",
		scale, gravity(0), gravity(1), gravity(2), 
		dAccBias(0), dAccBias(1), dAccBias(2));

	return true;
}

// estimate acc bias and scale
void ImuEstimator::EstimateScaleAndAccBias(const Eigen::Vector3d& dGyroBias,
	double& scale, Eigen::Vector3d& dAccBias)
{
	Eigen::Vector3d bg = dGyroBias;
	//bg << 0, 0, 0;

	int N = mvBiasNode.size();
	Eigen::Matrix3d R_cw_1, R_cw_2, R_cw_3;
	Eigen::Vector3d P_cw_1, P_cw_2, P_cw_3;
	Eigen::Matrix3d R_iw_1, R_iw_2;

	Eigen::MatrixXd H(3 * (N - 1), 4);
	Eigen::VectorXd z(3 * (N - 1));

	// estimate scale bias, gravity and acc bias
	for (int i = 0; i < N - 1; i++) {
		BiasNode& node_1 = mvBiasNode[i];
		BiasNode& node_2 = mvBiasNode[i + 1];

		double dt12 = node_1.endTime - node_1.startTime;
		double dt23 = node_2.endTime - node_2.startTime;

		R_cw_1 = node_1.visionStart.rot;
		R_cw_2 = node_1.visionEnd.rot;
		R_cw_3 = node_2.visionEnd.rot;

		P_cw_1 = node_1.visionStart.pos;
		P_cw_2 = node_1.visionEnd.pos;
		P_cw_3 = node_2.visionEnd.pos;

		R_iw_1 = R_cw_1;
		R_iw_2 = R_cw_2;

		Eigen::Matrix3d& dRot_12 = node_1.dPose.deltaRot;
		Eigen::Vector3d& dVel_12 = node_1.dPose.deltaVel;
		Eigen::Vector3d& dPos_12 = node_1.dPose.deltaPos;
		Eigen::Matrix3d& dRot_23 = node_2.dPose.deltaRot;
		Eigen::Vector3d& dVel_23 = node_2.dPose.deltaVel;
		Eigen::Vector3d& dPos_23 = node_2.dPose.deltaPos;

		Eigen::Matrix3d& Jp12 = node_1.dPose.dPa;
		Eigen::Matrix3d& Jv12 = node_1.dPose.dVa;
		Eigen::Matrix3d& Jp23 = node_2.dPose.dPa;
		Eigen::Matrix3d& Jv23 = node_2.dPose.dVa;

		Eigen::Matrix3d& Jpg12 = node_1.dPose.dPg;
		Eigen::Matrix3d& Jvg12 = node_1.dPose.dVg;
		Eigen::Matrix3d& Jpg23 = node_2.dPose.dPg;
		Eigen::Matrix3d& Jvg23 = node_2.dPose.dVg;

		Eigen::Vector3d alpha = (P_cw_2 - P_cw_1) * dt23 - (P_cw_3 - P_cw_2) * dt12;
		Eigen::Matrix3d beta = R_iw_2 * Jp23 * dt12 - R_iw_1 * Jp12 * dt23 + R_iw_1 * Jv12 * dt12 * dt23;
		Eigen::Vector3d gamma = (R_cw_3 - R_cw_2) * mPci * dt12 - (R_cw_2 - R_cw_1) * mPci * dt23
			- R_iw_1 * (dVel_12 + Jvg12 * bg) * dt12 * dt23
			+ R_iw_1 * (dPos_12 + Jpg12 * bg) * dt23 - R_iw_2 * (dPos_23 + Jpg23 * bg) * dt12
			- 0.5 * mGravity * dt12 * dt23 * (dt12 + dt23);

		H.block<3, 1>(i * 3, 0) = alpha;
		H.block<3, 3>(i * 3, 1) = beta;
		z.block<3, 1>(i * 3, 0) = gamma;
	}

	Eigen::VectorXd x = (H.transpose() * H).inverse() * (H.transpose() * z);

	// scale
	scale = x(0);
	// acc bias
	dAccBias = x.block<3, 1>(1, 0);

	//std::cout << "scale: " << scale << std::endl;
	//std::cout << "acc bias: " << dAccBias << std::endl;
}

// estimate acc bias
void ImuEstimator::EstimateAccBias(const Eigen::Vector3d& dGyroBias, Eigen::Vector3d& dAccBias)
{
	Eigen::Vector3d bg = dGyroBias;
	//bg << 0, 0, 0;

	int N = mvBiasNode.size();
	Eigen::Matrix3d R_cw_1, R_cw_2, R_cw_3;
	Eigen::Vector3d P_cw_1, P_cw_2, P_cw_3;
	Eigen::Matrix3d R_iw_1, R_iw_2;

	Eigen::MatrixXd H(3 * (N - 1), 3);
	Eigen::VectorXd z(3 * (N - 1));

	// estimate scale bias, gravity and acc bias
	for (int i = 0; i < N - 1; i++) {
		BiasNode& node_1 = mvBiasNode[i];
		BiasNode& node_2 = mvBiasNode[i + 1];

		double dt12 = node_1.endTime - node_1.startTime;
		double dt23 = node_2.endTime - node_2.startTime;

		R_cw_1 = node_1.visionStart.rot;
		R_cw_2 = node_1.visionEnd.rot;
		R_cw_3 = node_2.visionEnd.rot;

		P_cw_1 = node_1.visionStart.pos;
		P_cw_2 = node_1.visionEnd.pos;
		P_cw_3 = node_2.visionEnd.pos;

		R_iw_1 = R_cw_1;
		R_iw_2 = R_cw_2;

		Eigen::Matrix3d& dRot_12 = node_1.dPose.deltaRot;
		Eigen::Vector3d& dVel_12 = node_1.dPose.deltaVel;
		Eigen::Vector3d& dPos_12 = node_1.dPose.deltaPos;
		Eigen::Matrix3d& dRot_23 = node_2.dPose.deltaRot;
		Eigen::Vector3d& dVel_23 = node_2.dPose.deltaVel;
		Eigen::Vector3d& dPos_23 = node_2.dPose.deltaPos;

		Eigen::Matrix3d& Jp12 = node_1.dPose.dPa;
		Eigen::Matrix3d& Jv12 = node_1.dPose.dVa;
		Eigen::Matrix3d& Jp23 = node_2.dPose.dPa;
		Eigen::Matrix3d& Jv23 = node_2.dPose.dVa;

		Eigen::Matrix3d& Jpg12 = node_1.dPose.dPg;
		Eigen::Matrix3d& Jvg12 = node_1.dPose.dVg;
		Eigen::Matrix3d& Jpg23 = node_2.dPose.dPg;
		Eigen::Matrix3d& Jvg23 = node_2.dPose.dVg;

		Eigen::Vector3d alpha = (P_cw_2 - P_cw_1) * dt23 - (P_cw_3 - P_cw_2) * dt12;
		Eigen::Matrix3d beta = R_iw_2 * Jp23 * dt12 - R_iw_1 * Jp12 * dt23 + R_iw_1 * Jv12 * dt12 * dt23;
		Eigen::Vector3d gamma = (R_cw_3 - R_cw_2) * mPci * dt12 - (R_cw_2 - R_cw_1) * mPci * dt23
			- R_iw_1 * (dVel_12 + Jvg12 * bg) * dt12 * dt23
			+ R_iw_1 * (dPos_12 + Jpg12 * bg) * dt23 - R_iw_2 * (dPos_23 + Jpg23 * bg) * dt12
			- 0.5 * mGravity * dt12 * dt23 * (dt12 + dt23) - alpha * mScale;

		H.block<3, 3>(i * 3, 0) = beta;
		z.block<3, 1>(i * 3, 0) = gamma;
	}

	Eigen::VectorXd x = (H.transpose() * H).inverse() * (H.transpose() * z);

	// acc bias
	dAccBias = x;

	//std::cout << "acc bias: " << dAccBias << std::endl;
}


//-----------------------------------------------------------------------------
// estimate gravity 
void ImuEstimator::EstimateGravity(const VisionPose& visionPose, const AHRSPose& ahrsPose, int& nAhrsCnt)
{
	static Eigen::Matrix3d Rvw;
	GetWorldToVisionRotation(Rvw); 

	const Eigen::Matrix3d& Rwi = ahrsPose.rot;
	const Eigen::Matrix3d& Rvc = visionPose.rot;
	Eigen::Vector3d phi = Log_s(Rvw.transpose() * Rvc * Rwi.transpose());
	
	if (++nAhrsCnt > 100) {
		nAhrsCnt = 100; 
	}
	phi = phi / nAhrsCnt;
	Rvw = Rvw * so3Exp(phi);
	
	SetWorldToVisionRotation(Rvw); 
}

}
