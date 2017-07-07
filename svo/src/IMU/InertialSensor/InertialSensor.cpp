#include "stdafx.h"

#include "InertialSensor.h"
#include "IMU/AHRS/AHRS_Device.h"
#include "IMU/AHRS/AHRS_Madgwick.h"
//#include "IMU/AHRS/AHRS_Mahony.h"
#include "opencv2/opencv.hpp"
#include <opencv2/core/eigen.hpp>

namespace slam {

Eigen::MatrixXd InertialSensor::mCovNoise = Eigen::MatrixXd::Zero(6,6);

// Rotation matrix that transform from imu frame to camera frame
Eigen::Matrix3d InertialSensor::mRci;
// postion from imu to camera in camera frame
Eigen::Vector3d InertialSensor::mPci;

//-----------------------------------------------------------------------------
InertialSensor::InertialSensor(const std::string& configFile)
	: mbCorrect(true), mLastTimestamp(-1)
{
	_mpAHRS = AHRSFactory(AHRSType::Madgwick);
	//_mpAHRS = AHRSFactory(AHRSType::DeviceAHRS);

	// Load imu config file
	LoadConfigFile(configFile.c_str()); 

	// set initial value for imu calibration coefficients
	mAccBias.setZero();
	mAccMat = mRci.cast<float>();
	mGyroBias.setZero();
	mGyroMat = mRci.cast<float>();
	mMagBias.setZero();
	mMagMat = mRci.cast<float>();

}


//-----------------------------------------------------------------------------
InertialSensor::~InertialSensor()
{
	if (_mpAHRS != nullptr) {
		delete _mpAHRS;
	}
}


//-----------------------------------------------------------------------------
void InertialSensor::OnImuReceived(float* gyro, float* acc, float* mag, float* quat, double timestamp)
{
	if (mbCorrect) {
		// todo: add imu-cam time offset
		timestamp -= mImuCameraTimeShift;
		//printf("correct timestamp\n"); 
	}

	// initialize mLastTimestamp
	if (mLastTimestamp < 0) {
		mLastTimestamp = timestamp - mSampleTime;
	}
	float dt = timestamp - mLastTimestamp;

	// check mag data is new
	bool isMagNew = true;
	if (mag == nullptr) {
		isMagNew = false;
	}
	else {
		static float mx = 0, my = 0, mz = 0;
		if (mx == mag[0] && my == mag[1] && mz == mag[2]) {
			isMagNew = false;
		}
		mx = mag[0]; my = mag[1]; mz = mag[2];
	}
	

	// correct imu data
	if (mbCorrect) {
		CorrectImu(gyro, acc, mag);
	}

	// quaternion
	float qw = 1.0f, qx = 0, qy = 0, qz = 0;
	if (_mpAHRS != nullptr) 
	{
		// imu's coordinate system is: x right, y forward, z up
		// madgwick's coordinate system is: x forward, y left, z up
		switch (_mpAHRS->mType)
		{
		case AHRSType::Madgwick:
			if (isMagNew)
				//_mpAHRS->Update(gyro[1], -gyro[0], gyro[2], acc[1], -acc[0], acc[2], mag[1], -mag[0], mag[2], dt);
				_mpAHRS->Update(gyro[0], gyro[1], gyro[2], acc[0], acc[1], acc[2], mag[0], mag[1], mag[2], dt);
			else
				//_mpAHRS->Update(gyro[1], -gyro[0], gyro[2], acc[1], -acc[0], acc[2], 0, 0, 0, dt);
				_mpAHRS->Update(gyro[0], gyro[1], gyro[2], acc[0], acc[1], acc[2], 0, 0, 0, dt);
			_mpAHRS->GetQuaternion(qw, qx, qy, qz);
			break;
		
		case AHRSType::DeviceAHRS:
			if (quat != nullptr) {
				qx = quat[0]; qy = quat[1]; qz = quat[2]; qw = quat[3];

				// align camera and imu coordinate system
				Eigen::Quaternionf qwi(qw, qx, qy, qz);
				Eigen::Quaternionf qwc = qwi * Eigen::Quaternionf(mRci.transpose().cast<float>());
				qw = qwc.w();
				qx = qwc.x();
				qy = qwc.y();
				qz = qwc.z();
			}
			
			break;

		default:
			break;
		}
	}

	//printf("%f, %f, %f, %f\n", qw, qx, qy, qz); 

	// notify listeners
	NotifyListeners(gyro[0], gyro[1], gyro[2], acc[0], acc[1], acc[2], mag[0], mag[1], mag[2], 
		qw, qx, qy, qz, timestamp);

	// record last timestamp
	mLastTimestamp = timestamp;

}


//-----------------------------------------------------------------------------
void InertialSensor::LoadConfigFile(const char* path)
{
	// default
	mComPort = "";
	mImuCameraTimeShift = 0.02;
	mSampleFreq = 500;
	mSampleTime = 1.0f / mSampleFreq;

	// open config file
	cv::FileStorage fs(path, cv::FileStorage::READ);
	if (!fs.isOpened()) {
		printf("Error opening IMU config file: '%s'\n", path);
		return;
	}

	fs["sensor_name"] >> mSensorName;
	fs["com_port"] >> mComPort;;

	fs["sample_rate"] >> mSampleFreq;
	mSampleTime = 1.0f / mSampleFreq;

	// imu camera relationship
	fs["timeshift_cam_imu"] >> mImuCameraTimeShift; 

	cv::Mat_<double> Rci, Pci;
	fs["Rci"] >> Rci;
	fs["Pci"] >> Pci;
	cv::cv2eigen(Rci, mRci);
	cv::cv2eigen(Pci, mPci); 

	// imu noise
	fs["gyro_noise_density"] >> gyro_noise_density;
	fs["gyro_random_walk"] >> gyro_random_walk;
	fs["acc_noise_density"] >> acc_noise_density;
	fs["acc_random_walk"] >> acc_random_walk;

	// gyro noise variance, discrete
	float gyro_noise = mSampleFreq * gyro_noise_density * gyro_noise_density;
	// acc noise variance, discrete
	float acc_noise = mSampleFreq * acc_noise_density * acc_noise_density;
	for (int i = 0; i < 6; i++) {
		mCovNoise(i, i) = i < 3 ? gyro_noise : acc_noise;
	}

	// release
	fs.release();
}


//-----------------------------------------------------------------------------
void InertialSensor::SetCalibrationFile(const std::string& path)
{
	LoadCalibrationData(path.c_str()); 

	mbCorrect = true;
}

//-----------------------------------------------------------------------------
void InertialSensor::LoadCalibrationData(const char* path)
{
	// open calibration file
	cv::FileStorage fs(path, cv::FileStorage::READ);
	if (!fs.isOpened()) {
		printf("Error opening IMU calibration file: '%s'\n", path);
		return;
	}

	cv::Mat_<float> misMat, scaleMat, biasMat, mat;
	Eigen::Matrix3f eigenMat;

	// acc 
	fs["acc_misMat"] >> misMat;
	fs["acc_scaleMat"] >> scaleMat;
	fs["acc_bias"] >> biasMat;
	mat = misMat * scaleMat;
	cv::cv2eigen(biasMat, mAccBias);
	cv::cv2eigen(mat, eigenMat);
	mAccMat *= eigenMat;

	// gyro
	fs["gyro_misMat"] >> misMat;
	fs["gyro_scaleMat"] >> scaleMat;
	fs["gyro_bias"] >> biasMat;
	mat = misMat * scaleMat;
	cv::cv2eigen(biasMat, mGyroBias);
	cv::cv2eigen(mat, eigenMat);
	mGyroMat *= eigenMat;

	// mag
	fs["mag_misMat"] >> misMat;
	fs["mag_scaleMat"] >> scaleMat;
	fs["mag_bias"] >> biasMat;
	mat = misMat * scaleMat;
	cv::cv2eigen(biasMat, mMagBias);
	cv::cv2eigen(mat, eigenMat);
	mMagMat *= eigenMat;
}


//-----------------------------------------------------------------------------
void InertialSensor::CorrectImu(float* gyro, float* acc, float* mag)
{
	Eigen::Map<Eigen::Vector3f> gyroVec(gyro);
	Eigen::Map<Eigen::Vector3f> accVec(acc);
	Eigen::Map<Eigen::Vector3f> magVec(mag);

	// gyro
	gyroVec = mGyroMat * (gyroVec - mGyroBias);
	// acc
	accVec = mAccMat * (accVec - mAccBias);
	// mag
	magVec = mMagMat * (magVec - mMagBias);
}


//-----------------------------------------------------------------------------
AHRS* InertialSensor::AHRSFactory(AHRSType type)
{
	AHRS* pAHRS = nullptr;
	switch (type)
	{
	case AHRSType::DeviceAHRS:
		pAHRS = new DeviceAHRS();
		break;
	case AHRSType::GyroIntegration:
		break;
	case AHRSType::Madgwick:
		pAHRS = new MadgwickAHRS();
		break;
	case AHRSType::Mahony:
		//pAHRS = new MahonyAHRS();
		break;
	default:
		pAHRS = new MadgwickAHRS();
		break;
	}

	return pAHRS;
}


//-----------------------------------------------------------------------------
/// Add a listener.  
void InertialSensor::AddListener(Listener* l)
{
	// check whether it is added
	for (auto& listener : mListeners) {
		if (l == listener) {
			return;
		}
	}
	mListeners.push_back(l);
}


//-----------------------------------------------------------------------------
void InertialSensor::NotifyListeners(
	float gx, float gy, float gz,
	float ax, float ay, float az,
	float mx, float my, float mz,
	float qw, float qx, float qy, float qz,
	double timestamp)
{
	for (auto listener : mListeners) {
		listener->onInertialSensorData(gx, gy, gz, ax, ay, az, mx, my, mz, qw, qx, qy, qz, timestamp);
	}
}


}