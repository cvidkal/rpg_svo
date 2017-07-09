#pragma once
#include <Eigen/Dense>
#include "ImuBuffer.h"
#include "ImuEstimator.h"
#include "sophus/se3.h"
#include "VideoSource/VideoSource.h"
#include <svo/frame_handler_mono.h>
#include <vikit/abstract_camera.h>
namespace  slam {
	class SlamSystem
	{
		enum Status{

		};
	public:
		SlamSystem(){};
		void init();
		void run();
	private:

		VideoSource* video_source_;
		FrameHandlerMono* vision_;
		vk::AbstractCamera* cam0;
	};

}
