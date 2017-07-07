#pragma once

#include <Eigen/Dense>

/// IMU delta pose type
struct DeltaPose
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW; 

	// is inverse preintegration? 
	int bInverse;

	// start timestamp. unit is second
	double startTime;
	// end timestamp. unit is second
	double endTime;

	// delta time
	double deltaTime;

	// delta rotation matrix
	Eigen::Matrix3d deltaRot;
	// delta velocity
	Eigen::Vector3d deltaVel;
	// delta position
	Eigen::Vector3d deltaPos;

	// covariance 
	Eigen::Matrix<double, 9, 9> covariance;

	// bias Jacobian
	Eigen::Matrix3d dRg;
	Eigen::Matrix3d dVa;
	Eigen::Matrix3d dVg;
	Eigen::Matrix3d dPa;
	Eigen::Matrix3d dPg;

public:
	void Reset() {
		bInverse = 0;
		deltaTime = 0;

		deltaRot = Eigen::Matrix3d::Identity();
		deltaVel << 0, 0, 0;
		deltaPos << 0, 0, 0;
		covariance = Eigen::MatrixXd::Zero(9, 9);

		dRg = Eigen::Matrix3d::Zero();
		dVg = Eigen::Matrix3d::Zero();
		dVa = Eigen::Matrix3d::Zero();
		dPg = Eigen::Matrix3d::Zero();
		dPa = Eigen::Matrix3d::Zero();
	}
};


struct AHRSPose
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW; 

	// rotation matrix: Rwi
	Eigen::Matrix3d rot;

	// covariance 
	Eigen::Matrix3d covariance;
};


struct VisionPose
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW; 

	// translation vector: Pvc
	Eigen::Vector3d pos;
	// rotation matrix: Rvc
	Eigen::Matrix3d rot;
	// velocity: Vvi
	Eigen::Vector3d vel;

	//// covariance 
	//Eigen::Matrix<double, 9, 9> covariance;

	// inverse covarance
	Eigen::Matrix<double, 9, 9> invCovariance;

	// achieved residual
	float residual;

	// quality: 0 - BAD, 1 - DODGY, 2 - GOOD
	int quality;

};


struct FusionPose
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

	// rotation matrix
	Eigen::Matrix3d rot;
	// velocity
	Eigen::Vector3d vel;
	// position
	Eigen::Vector3d pos;

	// gyro bias
	Eigen::Vector3d gyro_bias;
	// acc bias
	Eigen::Vector3d acc_bias;

	// covariance 
	Eigen::Matrix<double, 9, 9> covariance;
};

