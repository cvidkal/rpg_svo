#include "stdafx.h"

#include "ImuBuffer.h"
#include "IMU/ImuEstimator.h"
#include "IMU/Utils/MathUtils.h"
#include <iostream>

namespace slam {

//-----------------------------------------------------------------------------
ImuBuffer::ImuBuffer(InertialSensor* pImu)
	: RingBuffer<ImuData, IMU_EXP>(), mpImu(pImu)
{
	if (mpImu != nullptr) {
		// add listener
		mpImu->AddListener(this);

		// set mCovNoise
		mCovNoise = mpImu->GetImuNoiseCovariance();
	}
	else {
		// covariance
		Eigen::VectorXd pv = Eigen::VectorXd(6, 1);
		pv << 2e-4, 2e-4, 2e-4, 5e-3, 5e-3, 5e-3;

		//pv = pv.cwiseProduct(pv);
		mCovNoise = pv.asDiagonal();
	}

}

//-----------------------------------------------------------------------------
ImuBuffer::~ImuBuffer()
{
}

//-----------------------------------------------------------------------------
/// Add a listener.  
void ImuBuffer::AddListener(Listener* l)
{
	// check whether it is added
	for (auto& listener : mListeners) {
		if (l == listener) {
			return;
		}
	}
	mListeners.push_back(l);
}


////-----------------------------------------------------------------------------
//void ImuBuffer::NotifyListeners(double timestamp)
//{
//	for (auto listener : mListeners) {
//		listener->onImuDataAdded(this, timestamp);
//	}
//}

//-----------------------------------------------------------------------------
void ImuBuffer::onInertialSensorData(
	float gx, float gy, float gz,
	float ax, float ay, float az,
	float mx, float my, float mz,
	float qw, float qx, float qy, float qz, 
	double timestamp)
{
	// add to buffer
	AddData(gx, gy, gz, ax, ay, az, mx, my, mz, qw, qx, qy, qz, timestamp);
}

//-----------------------------------------------------------------------------
void ImuBuffer::AddData(
	float gx, float gy, float gz,
	float ax, float ay, float az,
	float mx, float my, float mz,
	float qw, float qx, float qy, float qz, 
	double timestamp)
{
	auto& imuData = _buffer[head];
	imuData.timestamp = timestamp;

	imuData.gyro << gx, gy, gz;
	imuData.acc << ax, ay, az;
	imuData.mag << mx, my, mz;
	imuData.q = Eigen::Quaternionf(qw, qx, qy, qz);

	IndexGoAhead();

	// log to file
	static const int debug_log_level = SLAM_CONTEXT.debug_log_level;
	if (debug_log_level >= 1) {
		LogMgr.imu_log << (long long)(timestamp * 1e9) << ","
			<< gx << "," << gy << "," << gz << ","
			<< ax << "," << ay << "," << az << ","
			<< mx << "," << my << "," << mz << ","
			<< qw << "," << qx << "," << qy << "," << qz << std::endl;
	}

	// notify listeners
	for (auto listener : mListeners) {
		listener->onImuDataAdded(imuData);
	}
}

//-----------------------------------------------------------------------------
// always return lower index
int ImuBuffer::BinarySearch(double timestamp) const
{
	// check
	if (Size() < 100)
		return -1;

	int head_1 = Index(head, -1);
	// make sure buffer > 60
	int tail_1 = Index(head, -50);
	if (mBufferSize < 60) {
		tail_1 = Index(tail, 10);
	}

	// not in range
	if (timestamp > _buffer[head_1].timestamp)
		return -2;
	if (timestamp < _buffer[tail_1].timestamp) {
		tail_1 = Index(tail, 10);
		if (timestamp < _buffer[tail_1].timestamp)
			return -2;
	}

	// determine low and high index of search range
	int low, high;
	if (head_1 > tail_1) {
		low = tail_1;
		high = head_1;
	} 
	else {
		if (_buffer[0].timestamp > timestamp) {
			low = tail_1;
			high = mBufferSize - 1;
		}
		else {
			low = 0;
			high = head_1;
		}
	}

	// binary search
	int mid;
	while (low + 1 < high) {
		mid = (low + high) / 2;
		if (_buffer[mid].timestamp == timestamp)
			return mid;
		else if (_buffer[mid].timestamp > timestamp)
			high = mid;
		else
			low = mid;
	}

	//printf("BinarySearch: index = %d, head = %d\n", low, head_1);
	//printf(" cam timestamp = %f\nhead timestamp = %f\n", timestamp, _buffer[head_1].timestamp);

	// todo: check camera timestamp and buffer head timestamp difference

	// return low index
	return low;
}

//-----------------------------------------------------------------------------
// 
int ImuBuffer::GetDataByTimestamp(double timestamp, ImuData& imuData) const
{
	int index = BinarySearch(timestamp);
	if (index < 0) {
		GetData(imuData);
		return index;
	}

	// get nearest two samples
	auto& imuData_1 = _buffer[index];
	auto& imuData_2 = _buffer[Index(index, 1)];

	// linear interpolation
	float k = (timestamp - imuData_1.timestamp) * 1.0 / (imuData_2.timestamp - imuData_1.timestamp);
	imuData.gyro = (1 - k) * imuData_1.gyro + k * imuData_2.gyro;
	imuData.acc = (1 - k) * imuData_1.acc + k * imuData_2.acc;
	imuData.mag = (1 - k) * imuData_1.mag + k * imuData_2.mag;

	// predict quaternion
	float dt = timestamp - imuData_1.timestamp;
	Eigen::Vector3f dPhi = imuData.gyro * (0.5f * dt);//constant angular acceleration
	Eigen::Quaternionf dq(1, dPhi[0], dPhi[1], dPhi[2]);
	imuData.q = imuData_1.q * dq;

	return index;
}


//-----------------------------------------------------------------------------
int ImuBuffer::GetData(ImuData& imuData, int index) const
{
	if (index > 0)
		index = 0;
	int head_1 = Index(head, index-1);
	imuData = _buffer[head_1];
	return head_1;
}

//-----------------------------------------------------------------------------
double ImuBuffer::GetHeadTimestamp() const
{
	int head_1 = Index(head, -1);
	return _buffer[head_1].timestamp;
}

//-----------------------------------------------------------------------------
bool ImuBuffer::GetDataByIndex(int& index, ImuData& imuData) const
{
	if (index < 0 || index >= mBufferSize)
		return false;
	// check full
	//if (!Full())
	//	return false;
	if (Size() < 100)
		return false;

	int head_1 = head;
	int tail_1 = Index(tail, 10);
	if (head_1 > tail_1) {
		if (index >= head_1 || index < tail_1)
			return false;
	}
	else {
		if (index >= head_1 && index < tail_1)
			return false;
	}

	imuData = _buffer[index];
	index = Index(index, 1);
	return true;
}

//----------------------------------------------------------------------------
// Implementation of "IMU Preintegration on Manifold for Efficient
// Visual-Inertial Maximum-a-Posteriori Estimation"
int ImuBuffer::ImuPreIntegrate(DeltaPose& dPose, bool bJacobian, bool bCovariance) const
{
	if (dPose.startTime >= dPose.endTime) {
		return -1;
	}
	int startIndex = BinarySearch(dPose.startTime);
	int endIndex = BinarySearch(dPose.endTime);
	if (endIndex < 0) {
		endIndex = Index(head, -1); 
	}
	if (startIndex < 0 || endIndex < 0) {
		return -1;
	}

	// inverse
	dPose.bInverse = 0;

	/// preintegration
	Eigen::Matrix3d& dRot = dPose.deltaRot;
	Eigen::Vector3d& dVel = dPose.deltaVel;
	Eigen::Vector3d& dPos = dPose.deltaPos;
	Eigen::Matrix3d dRot_0 = dRot;
	Eigen::Vector3d dVel_0 = dVel;

	// bias jacobian
	Eigen::Matrix3d& dRg = dPose.dRg;
	Eigen::Matrix3d& dVa = dPose.dVa;
	Eigen::Matrix3d& dVg = dPose.dVg;
	Eigen::Matrix3d& dPa = dPose.dPa;
	Eigen::Matrix3d& dPg = dPose.dPg;

	// covariance matrix
	Eigen::Matrix<double, 9, 9>& cov = dPose.covariance;
	static Eigen::MatrixXd A = Eigen::MatrixXd::Zero(9, 9);
	static Eigen::MatrixXd B = Eigen::MatrixXd::Zero(9, 6);
	A.block<3, 3>(3, 3) = Eigen::Matrix3d::Identity();
	A.block<3, 3>(6, 6) = Eigen::Matrix3d::Identity();

	// get imu bias
	Eigen::Vector3d gyroBias, accBias;
	ImuEstimator::GetImuBias(gyroBias, accBias);
	// todo: need to check whether bias is correct!!!

	int endIndex_1 = Index(endIndex, 1);
	if (_buffer[endIndex].timestamp == dPose.endTime) {
		endIndex_1 = endIndex;
	}
	int index = startIndex;
	int index_1;
	double dt;
	while (index != endIndex_1) {
		index_1 = Index(index, 1);

		auto& imuData = _buffer[index];
		auto& imuData_1 = _buffer[index_1];

		Eigen::Vector3d gyro, acc;
		if (index == startIndex) {
			dt = imuData_1.timestamp - dPose.startTime;
			double t = (dPose.startTime + imuData_1.timestamp) / 2;
			float k1, k2;
			if (imuData_1.timestamp - imuData.timestamp == 0) {
				k1 = k2 = 0.5;
			}
			else {
				k1 = (imuData_1.timestamp - t) / (imuData_1.timestamp - imuData.timestamp);
				k2 = 1 - k1;
			}
			gyro = (k1 * imuData.gyro + k2 * imuData_1.gyro).cast<double>();
			acc = (k1 * imuData.acc + k2 * imuData_1.acc).cast<double>();
		}
		else if (index == endIndex) {
			dt = dPose.endTime - imuData.timestamp;
			double t = (dPose.endTime + imuData.timestamp) / 2;
			float k1, k2;
			if (imuData_1.timestamp - imuData.timestamp == 0) {
				k1 = k2 = 0.5;
			}
			else {
				k1 = (imuData_1.timestamp - t) / (imuData_1.timestamp - imuData.timestamp);
				k2 = 1 - k1;
			}
			gyro = (k1 * imuData.gyro + k2 * imuData_1.gyro).cast<double>();
			acc = (k1 * imuData.acc + k2 * imuData_1.acc).cast<double>();
		}
		else {
			dt = imuData_1.timestamp - imuData.timestamp;
			gyro = ((imuData.gyro + imuData_1.gyro) / 2).cast<double>();
			acc = ((imuData.acc + imuData_1.acc) / 2).cast<double>();
		}

		// abstract bias
		gyro -= gyroBias;
		acc -= accBias;

		/// incrementally compute delta pose
		// attitude
		Eigen::Matrix3d ddRot = Exp_s(gyro * dt);
		dRot = dRot_0 * ddRot;
		// velocity
		Eigen::Vector3d ddVel = dRot_0 * acc * dt;
		dVel = dVel_0 + ddVel;
		// position
		dPos += dVel_0 * dt + ddVel * (0.5 * dt);

		if (bJacobian || bCovariance) {
			/// some common matrix
			// Jr * dt
			Eigen::Matrix3d Jr_dt = VectorToJacobianMatrix(gyro * dt) * dt;
			// - Rik * (ak - ba)^ * dt
			Eigen::Matrix3d Ra_dt = dRot_0 * VectorToCrossFormMatrix<double>(acc * (-dt));
			//  - 1/2 * Rik * (ak - ba)^ * dt^2
			Eigen::Matrix3d Ra_dtdt = Ra_dt * (0.5 * dt);

			/// covariance matrix
			if (bCovariance) {
				A.block<3, 3>(0, 0) = ddRot.transpose();
				A.block<3, 3>(3, 0) = Ra_dt;
				A.block<3, 3>(6, 0) = Ra_dtdt;
				A.block<3, 3>(6, 3) = Eigen::Matrix3d::Identity() * dt;

				B.block<3, 3>(0, 0) = Jr_dt;
				B.block<3, 3>(3, 3) = dRot_0 * dt;
				B.block<3, 3>(6, 3) = dRot_0 * (0.5 * dt * dt);

				//// todo: use sparse matrix to reduce computation
				cov = A * cov * A.transpose() + B * mCovNoise * B.transpose();
			}

			/// bias jacobian
			if (bJacobian) {
				dPa += -dVa * dt - dRot_0 * (0.5 * dt * dt);
				dPg += -dVg * dt + Ra_dtdt * dRg;
				dVa -= dRot_0 * dt;
				dVg += Ra_dt * dRg;
				dRg = ddRot.transpose() * dRg - Jr_dt;

				//dPa += dVa * dt - dRot_0 * (0.5 * dt * dt);
				//dPg += dVg * dt + Ra_dtdt * dRg;
				//dVa -= dRot_0 * dt;
				//dVg += Ra_dt * dRg;
				//dRg = ddRot.transpose() * dRg - Jr_dt;
			}

		}

		// backup
		dRot_0 = dRot;
		dVel_0 = dVel;

		// index forward
		index = index_1;
	}

	// delta time
	dPose.deltaTime += dPose.endTime - dPose.startTime;

	return DeltaSize(startIndex, endIndex) - 1;
}


//----------------------------------------------------------------------------
// inverse preintegration
int ImuBuffer::invPreIntegrate(DeltaPose& dPose, bool bJacobian, bool bCovariance) const
{
	if (dPose.startTime >= dPose.endTime) {
		return -1;
	}
	int startIndex = BinarySearch(dPose.endTime);
	int endIndex = BinarySearch(dPose.startTime);

	if (_buffer[startIndex].timestamp != dPose.endTime) {
		startIndex = Index(startIndex, 1);
	}
	endIndex = Index(endIndex, 1); 

	//if (startIndex < 0) {
	//	startIndex = Index(head, -1);
	//}
	if (startIndex < 0 || endIndex < 0) {
		return -1;
	}

	// inverse
	dPose.bInverse = 1;

	/// preintegration
	Eigen::Matrix3d& dRot = dPose.deltaRot;
	Eigen::Vector3d& dVel = dPose.deltaVel;
	Eigen::Vector3d& dPos = dPose.deltaPos;
	Eigen::Matrix3d dRot_0 = dRot;
	Eigen::Vector3d dVel_0 = dVel;

	// bias jacobian
	Eigen::Matrix3d& dRg = dPose.dRg;
	Eigen::Matrix3d& dVa = dPose.dVa;
	Eigen::Matrix3d& dVg = dPose.dVg;
	Eigen::Matrix3d& dPa = dPose.dPa;
	Eigen::Matrix3d& dPg = dPose.dPg;

	// covariance matrix
	Eigen::Matrix<double, 9, 9>& cov = dPose.covariance;
	static Eigen::MatrixXd A = Eigen::MatrixXd::Zero(9, 9);
	static Eigen::MatrixXd B = Eigen::MatrixXd::Zero(9, 6);
	A.block<3, 3>(3, 3) = Eigen::Matrix3d::Identity();
	A.block<3, 3>(6, 6) = Eigen::Matrix3d::Identity();

	// get imu bias
	Eigen::Vector3d gyroBias, accBias;
	ImuEstimator::GetImuBias(gyroBias, accBias);
	// todo: need to check whether bias is correct!!!

	int endIndex_1 = Index(endIndex, -1);
	if (_buffer[endIndex].timestamp == dPose.startTime) {
		endIndex_1 = endIndex;
	}
	int index = startIndex;
	int index_1;

	double dt;
	while (index != endIndex_1) {
		index_1 = Index(index, -1);

		auto& imuData = _buffer[index];
		auto& imuData_1 = _buffer[index_1];

		Eigen::Vector3d gyro, acc;
		if (index == startIndex) {
			dt = imuData_1.timestamp - dPose.endTime;
			double t = (dPose.endTime + imuData_1.timestamp) / 2;
			float k1, k2;
			if (imuData_1.timestamp - imuData.timestamp == 0) {
				k1 = k2 = 0.5;
			}
			else {
				k1 = (imuData_1.timestamp - t) / (imuData_1.timestamp - imuData.timestamp);
				k2 = 1 - k1;
			}
			gyro = (k1 * imuData.gyro + k2 * imuData_1.gyro).cast<double>();
			acc = (k1 * imuData.acc + k2 * imuData_1.acc).cast<double>();
		}
		else if (index == endIndex) {
			dt = dPose.startTime - imuData.timestamp;
			double t = (dPose.startTime + imuData.timestamp) / 2;
			float k1, k2;
			if (imuData_1.timestamp - imuData.timestamp == 0) {
				k1 = k2 = 0.5;
			}
			else {
				k1 = (imuData_1.timestamp - t) / (imuData_1.timestamp - imuData.timestamp);
				k2 = 1 - k1;
			}
			gyro = (k1 * imuData.gyro + k2 * imuData_1.gyro).cast<double>();
			acc = (k1 * imuData.acc + k2 * imuData_1.acc).cast<double>();
		}
		else {
			dt = imuData_1.timestamp - imuData.timestamp;
			gyro = ((imuData.gyro + imuData_1.gyro) / 2).cast<double>();
			acc = ((imuData.acc + imuData_1.acc) / 2).cast<double>();
		}

		// abstract bias
		gyro -= gyroBias;
		acc -= accBias;

		/// incrementally compute delta pose
		// attitude
		Eigen::Matrix3d ddRot = Exp_s(gyro * dt);
		dRot = dRot_0 * ddRot;
		// velocity
		Eigen::Vector3d ddVel = dRot_0 * acc * dt;
		dVel = dVel_0 + ddVel;
		// position
		dPos += dVel_0 * dt + ddVel * (0.5 * dt);

		if (bJacobian || bCovariance) {
			/// some common matrix
			// Jr * dt
			Eigen::Matrix3d Jr_dt = VectorToJacobianMatrix(gyro * dt) * dt;
			// - Rik * (ak - ba)^ * dt
			Eigen::Matrix3d Ra_dt = dRot_0 * VectorToCrossFormMatrix<double>(acc * (-dt));
			//  - 1/2 * Rik * (ak - ba)^ * dt^2
			Eigen::Matrix3d Ra_dtdt = Ra_dt * (0.5 * dt);

			/// covariance matrix
			if (bCovariance) {
				A.block<3, 3>(0, 0) = ddRot.transpose();
				A.block<3, 3>(3, 0) = Ra_dt;
				A.block<3, 3>(6, 0) = Ra_dtdt;
				A.block<3, 3>(6, 3) = Eigen::Matrix3d::Identity() * dt;

				B.block<3, 3>(0, 0) = Jr_dt;
				B.block<3, 3>(3, 3) = dRot_0 * dt;
				B.block<3, 3>(6, 3) = dRot_0 * (0.5 * dt * dt);

				//// todo: use sparse matrix to reduce computation
				cov = A * cov * A.transpose() + B * mCovNoise * B.transpose();
			}

			/// bias jacobian
			if (bJacobian) {
				//dPa += -dVa * dt - dRot_0 * (0.5 * dt * dt);
				//dPg += -dVg * dt + Ra_dtdt * dRg;
				//dVa -= dRot_0 * dt;
				//dVg += Ra_dt * dRg;
				//dRg = ddRot.transpose() * dRg - Jr_dt;

				dPa += dVa * dt - dRot_0 * (0.5 * dt * dt);
				dPg += dVg * dt + Ra_dtdt * dRg;
				dVa -= dRot_0 * dt;
				dVg += Ra_dt * dRg;
				dRg = ddRot.transpose() * dRg - Jr_dt;
			}

		}

		// backup
		dRot_0 = dRot;
		dVel_0 = dVel;

		// index forward
		index = index_1;
	}

	// delta time
	dPose.deltaTime -= dPose.endTime - dPose.startTime;

	return DeltaSize(endIndex, startIndex) - 1;
}

}
