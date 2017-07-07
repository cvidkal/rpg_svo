#pragma once

#include "IMU/AHRS/AHRS.h"
#include <string>
#include <list>
#include <Eigen/Dense>

namespace slam {

enum class IMUType
{
	DATASET,
	YEI
};

enum class IMUStatus {
	NOT_FOUND,
	NOT_READY,
	IDLE,
	RUNNING
};

class InertialSensor
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
public:
	class Listener
	{
	public:
		/// unit of timestamp is seconds, start since Epoch (1970-01-01T00:00:00Z)
		virtual void onInertialSensorData(
			float gx, float gy, float gz,
			float ax, float ay, float az,
			float mx, float my, float mz,
			float qw, float qx, float qy, float qz, 
			double timestamp) = 0;
	};

	/// Add a listener.  
	void AddListener(Listener* l);

private:
	std::list<Listener*> mListeners;

	// notify listeners
	void NotifyListeners(
		float gx, float gy, float gz,
		float ax, float ay, float az,
		float mx, float my, float mz,
		float qw, float qx, float qy, float qz, 
		double timestamp);

public:
	InertialSensor(const std::string& configFile);
	virtual ~InertialSensor();

	virtual void Start() = 0;
	virtual void Stop() = 0;

protected:
	void LoadConfigFile(const char* path);

protected:
	double mLastTimestamp;
	IMUStatus mImuStatus;
	
// AHRS
protected:
	AHRS* _mpAHRS;
	static AHRS* AHRSFactory(AHRSType type);

// sample frequency
protected:
	std::string mSensorName;
	std::string mComPort;
	int mSampleFreq;
	float mSampleTime;

public:
	inline int GetSampleFreqency() {
		return mSampleFreq;
	}

	inline float GetSampleTime() {
		return mSampleTime;
	}

//--- calibration
protected:
	bool mbCorrect;

	// calibration data
	Eigen::Vector3f mAccBias;
	Eigen::Matrix3f mAccMat;
	Eigen::Vector3f mGyroBias;
	Eigen::Matrix3f mGyroMat;
	Eigen::Vector3f mMagBias;
	Eigen::Matrix3f mMagMat;

	void LoadCalibrationData(const char* path);
	void CorrectImu(float* gyro, float* acc, float* mag);

public:
	void SetCalibrationFile(const std::string& path); 
	inline void setCorrection(bool bCorrect) {
		mbCorrect = bCorrect; 
	}

//--- imu noise
protected:
	// IMU Noise Parameters
	float gyro_noise_density;	// Gyroscope "white noise". rad/s/sqrt(Hz)
	float gyro_random_walk;		// Gyroscope "random walk". rad/s^2/sqrt(Hz)
	float acc_noise_density;	// Accelerometer "white noise". m/s^2/sqrt(Hz)
	float acc_random_walk;		// Accelerometer "random walk". m/s^3/sqrt(Hz)
	static Eigen::MatrixXd mCovNoise;	// Noise covariance matrix

public:
	inline Eigen::MatrixXd GetImuNoiseCovariance() const {
		return mCovNoise; 
	}

//--- imu camera relationship
public:
	// t_imu - t_cam. unit is second
	double mImuCameraTimeShift;
	// Rotation matrix that transform from imu frame to camera frame
	static Eigen::Matrix3d mRci;
	// postion from imu to camera in camera frame
	static Eigen::Vector3d mPci;

protected:
	void OnImuReceived(float* gyro, float* acc, float* mag, float* quat, double timestamp);

};

}