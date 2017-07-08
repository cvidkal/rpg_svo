#pragma once
#include <Eigen/Dense>
#include "ImuBuffer.h"
#include "ImuEstimator.h"
#include "sophus/se3.h"
#include "VideoSource/VideoSource.h"

namespace  slam {
	class SlamSystem
	{

	public:
		SlamSystem(){};
		void Init();

	private:


		VideoSource* video_source_;

	};

}
