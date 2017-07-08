#pragma once
// ImuEstimator recursively estimates scale, gravity, acc bias and gyro bias

#include <Eigen/Dense>

#include <thread>
#include <mutex>
#include <condition_variable>

#include <vector>

#include "IMU/Utils/TypeDefs.h"
#include "IMU/ImuManager.h"


namespace slam {

	struct BiasNode
	{
		int cnt;
		double startTime, endTime;

		DeltaPose dPose;

		VisionPose visionStart;
		VisionPose visionEnd;

		void Reset() {
			cnt = 0;
			dPose.Reset();
		}
	};


	class ImuEstimator
	{
	public:
		ImuEstimator();
		~ImuEstimator();

	public:
		// reset
		void Reset();

		// push vision pose to estimator thread
		void notifyToEstimateBias(const VisionPose& visionPose, double timestamp);

	private:
		bool mbUseImu;

		ImuBuffer* mpImuBuffer;

	public:
		///@note This line is critical, because we have member variable which is fix sized.
		EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	public:
		// relationship between imu and camera frame
		//Eigen::Matrix3d mRci;
		Eigen::Vector3d mPci;

		// bias ready flag
		static bool mbHaveEstimateScale;

		static inline bool isScaleEstimated() {
			return mbHaveEstimateScale;
		}

		// get imu bias
		static void GetImuBias(Eigen::Vector3d& gyroBias, Eigen::Vector3d& accBias);
		// set imu bias
		static void SetImuBias(const Eigen::Vector3d& gyroBias, const Eigen::Vector3d& accBias);

		// get scale and gravity
		static void GetScaleAndGravity(double& scale, Eigen::Vector3d& gravity);
		// set scale
		static void SetScale(double scale);

		// get imu world to vision frame rotation matrix 
		static void GetWorldToVisionRotation(Eigen::Matrix3d& Rvw);
		// set imu world to vision frame rotation matrix 
		static void SetWorldToVisionRotation(const Eigen::Matrix3d& Rvw);

	private:
		// start bias thread
		void StartEstimateBias();
		// stop bias thread
		void StopEstimateBias();

	private:
		// gyro bias
		static Eigen::Vector3d mGyroBias;
		// acc bias
		static Eigen::Vector3d mAccBias;

		// vision scale
		static double mScale;
		// gravity vector in vision frame
		static Eigen::Vector3d mGravity;
		// rotation matrix from world to vision frame
		static Eigen::Matrix3d mRvw;

		// multi thread
		static std::mutex mBiasMutex;
		bool mbThreadDone;
		bool mbThreadReset;
		std::thread* mpBiasThread;
		std::mutex mThreadMutex;
		std::condition_variable mBiasCondvar;

		// imu delta pose for bias
		DeltaPose mDeltaPose_Bias;
		// vision pose for bias
		VisionPose mVisionPose_Bias;
		// timestamp for bias
		double mTimestamp_Bias;

		// 
		std::vector<BiasNode> mvBiasNode;

		void RunBiasThread();

		// estimate gyro bias
		bool EstimateGyroBias(Eigen::Vector3d& dGyroBias, Eigen::Matrix3d& P,
			Eigen::Matrix3d& Q, bool& bFirstTime);
		// estimate scale, gravity and acc bias
		bool EstimateScaleAndGravity(const Eigen::Vector3d& dGyroBias, double& scale, Eigen::Vector3d& gravity,
			Eigen::Matrix3d& Rvw, Eigen::Vector3d& dAccBias, Eigen::MatrixXd& P, Eigen::MatrixXd& Q, bool& bFirstTime);
		// estimate scale and acc bias
		void EstimateScaleAndAccBias(const Eigen::Vector3d& dGyroBias, double& scale, Eigen::Vector3d& dAccBias);
		// estimate acc bias
		void EstimateAccBias(const Eigen::Vector3d& dGyroBias, Eigen::Vector3d& dAccBias);
		// estimate gravity 
		void EstimateGravity(const VisionPose& visionPose, const AHRSPose& ahrsPose, int& nAhrsCnt);
		// estimate scale
		void EstimateScale();

	private:
		bool CheckVisionQuality(const VisionPose& visionPose);

		// get and check ahrs pose
		bool GetAndCheckAhrsPose(AHRSPose& ahrsPose, double timestamp);


		//--- variables for bias thread
	private:
		BiasNode _node;
		VisionPose _visionPose;
		double _timestamp;

		bool _bReady;
		int _nReadyCnt;

		bool _bFirstTime_gyro;
		Eigen::Matrix3d _P_gyro, _Q_gyro;

		bool _bFirstTime_acc;
		Eigen::MatrixXd _P_acc, _Q_acc;

		int _nAhrsCnt;

		void ResetThreadVariables();
	};


}
