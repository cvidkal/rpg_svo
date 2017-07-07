#pragma once

#include <math.h>
#include <Eigen/Dense>

#define PI 3.14159265359
// gravity
#define GRAVITY	9.81f		


//-----------------------------------------------------------------------------
inline double Deg2Rad(double x) {
	return x * PI / 180.0;
}

//-----------------------------------------------------------------------------
inline double Rad2Deg(double x) {
	return x * 180.0 / PI;
}


//-----------------------------------------------------------------------------
// Fast inverse square-root
// See: http://en.wikipedia.org/wiki/Fast_inverse_square_root

float invSqrt(float x);

//------------------------------------------------------------------------------
Eigen::Vector3d MatrixToAttitudeAngle(const Eigen::Matrix3d& attMat);

//------------------------------------------------------------------------------
Eigen::Vector3d QuaternionToAttitudeAngle(const Eigen::Quaterniond& q);

//------------------------------------------------------------------------------
Eigen::Matrix3d AttitudeAngleToMatrix(const Eigen::Vector3d& attVec);

//------------------------------------------------------------------------------
template<typename FloatType>
inline Eigen::Matrix<FloatType, 3, 3> VectorToCrossFormMatrix(const Eigen::Matrix<FloatType, 3, 1>& x)
{
	Eigen::Matrix<FloatType, 3, 3> X;
	X << FloatType(0), -x(2), x(1),
		x(2), 0, -x(0),
		-x(1), x(0), FloatType(0);

	return X;
}

//------------------------------------------------------------------------------
template<typename FloatType>
inline Eigen::Matrix<FloatType, 3, 1> CrossFormMatrixToVector(const Eigen::Matrix<FloatType, 3, 3>& X)
{
	Eigen::Matrix<FloatType, 3, 1> x;
	x << (X(2, 1) - X(1, 2)) / FloatType(2), (X(0, 2) - X(2, 0)) / FloatType(2), (X(1, 0) - X(0, 1)) / FloatType(2);
	return x;
}

//------------------------------------------------------------------------------
// simplified Exp
inline Eigen::Matrix3d Exp_s(const Eigen::Vector3d& x)
{
	Eigen::Matrix3d X;
	X << 1, -x(2), x(1),
		x(2), 1, -x(0),
		-x(1), x(0), 1;

	return X;
}

//------------------------------------------------------------------------------
// simplified Log
inline Eigen::Vector3d Log_s(const Eigen::Matrix3d& X)
{
	return CrossFormMatrixToVector<double>(X);
}

//------------------------------------------------------------------------------
// Exp
inline Eigen::Matrix3d so3Exp(const Eigen::Vector3d& x)
{
	double phi = x.norm();
	Eigen::Matrix3d phi_cross = VectorToCrossFormMatrix<double>(x);
	Eigen::Matrix3d X;
	if (phi < 1e-5) {
		X = Eigen::Matrix3d::Identity() + phi_cross;
	}
	else {
		X = Eigen::Matrix3d::Identity() + sin(phi) / phi * phi_cross
			+ (1 - cos(phi)) / phi / phi * phi_cross * phi_cross;
	}


	return X;
}

//------------------------------------------------------------------------------
// Log
inline Eigen::Vector3d so3Log(const Eigen::Matrix3d& X)
{
	double phi = acos((X.trace() - 1) / 2);
	Eigen::Vector3d x = CrossFormMatrixToVector<double>(phi * (X - X.transpose()) / 2 / sin(phi));

	return x;
}

//------------------------------------------------------------------------------
// templated Exp
template<typename FloatType>
inline Eigen::Matrix<FloatType, 3, 3> Exp_t(const Eigen::Matrix<FloatType, 3, 1>& x)
{
	FloatType phi = x.norm();
	Eigen::Matrix<FloatType, 3, 3> phi_cross = VectorToCrossFormMatrix<FloatType>(x);
	Eigen::Matrix<FloatType, 3, 3> X = Eigen::Matrix<FloatType, 3, 3>::Identity() + sin(phi) / phi * phi_cross
		+ (FloatType(1) - cos(phi)) / phi / phi * phi_cross * phi_cross;

	return X;
}

//------------------------------------------------------------------------------
// templated Log
template<typename FloatType>
inline Eigen::Matrix<FloatType, 3, 1> Log_t(const Eigen::Matrix<FloatType, 3, 3>& X)
{
	FloatType phi = acos((X.trace() - FloatType(1)) / FloatType(2));
	//printf("phi: %f\n", phi); 
	Eigen::Matrix<FloatType, 3, 1> x = CrossFormMatrixToVector<FloatType>(phi * (X - X.transpose()) / FloatType(2) / sin(phi));

	return x;
}

//------------------------------------------------------------------------------
// simplified Jacobian
inline Eigen::Matrix3d VectorToJacobianMatrix(const Eigen::Vector3d& x)
{
	return Eigen::Matrix3d::Identity() - 0.5 * VectorToCrossFormMatrix(x);
}
