#pragma once


#include <Eigen/Dense>
#include "IMU/Utils/RingBuffer.h"
#include "IMU/InertialSensor/InertialSensor.h"
#include "IMU/Utils/TypeDefs.h"



#define IMU_EXP 10		// default buffer size: 2 ^ IMU_EXP


namespace slam {

struct ImuData
{
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

	// unit is second
	double timestamp;
	
	Eigen::Vector3f gyro;
	Eigen::Vector3f acc;
	Eigen::Vector3f mag;
	Eigen::Quaternionf q; // w, x, y, z

};


/*
 * IMU Buffer. Do not need to care about multithread.
 */
class ImuBuffer 
	: public RingBuffer<ImuData, IMU_EXP>, public InertialSensor::Listener
{
public:
	class Listener {
	public:
		virtual void onImuDataAdded(const ImuData& imuData) = 0;
	};

	/// Add a listener.  
	void AddListener(Listener* l);

private:
	std::list<Listener*> mListeners;

	//// notify listeners
	//void NotifyListeners(double timestamp);

public:
	ImuBuffer(InertialSensor* pImu = nullptr);
	~ImuBuffer();

	int GetDataByTimestamp(double timestamp, ImuData& imuData) const;
	int GetData(ImuData& imuData, int index = 0) const;
	double GetHeadTimestamp() const;
	bool GetDataByIndex(int& index, ImuData& imuData) const;

	//----------------------------------------------------------------------------
	// Implementation of "IMU Preintegration on Manifold for Efficient
	// Visual-Inertial Maximum-a-Posteriori Estimation"
	int ImuPreIntegrate(DeltaPose& dPose, bool bJacobian = true, bool bCovariance = true) const;

	// inverse preintegration
	int invPreIntegrate(DeltaPose& dPose, bool bJacobian = true, bool bCovariance = true) const;

private:
	// binary search, return lower index
	int BinarySearch(double timestamp) const;

	void onInertialSensorData(
		float gx, float gy, float gz,
		float ax, float ay, float az,
		float mx, float my, float mz,
		float qw, float qx, float qy, float qz, 
		double timestamp) override;

public:
	void AddData(
		float gx, float gy, float gz,
		float ax, float ay, float az,
		float mx, float my, float mz,
		float qw, float qx, float qy, float qz, 
		double timestamp);

private:
	InertialSensor* mpImu;
	Eigen::MatrixXd mCovNoise;

};

}
