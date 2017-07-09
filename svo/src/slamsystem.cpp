#include "stdafx.h"
#include "slamsystem.h"
#include "VideoSource/VideoSource_Euroc.h"
#include <yaml-cpp/yaml.h>
#include <vikit/pinhole_camera.h>
namespace  slam {



	void SlamSystem::init()
	{

		//Init Camera
        {
            //cam0
            cam0  = new vk::PinholeCamera(752, 480,458.654, 457.296, 367.215, 248.375,
                                          -0.28340811, 0.07395907, 0.00019359, 1.76187114e-05);

            Eigen::Vector3d Pic0(-0.0216401454975, -0.064676986768, 0.00981073058949);
            Eigen::Matrix3d Ric0;
            Ric0<<0.0148655429818, -0.999880929698, 0.00414029679422,
                    0.999557249008, 0.0149672133247, 0.025715529948,
                    -0.0257744366974, 0.00375618835797, 0.999660727178;
            cam0->setCameraExtrinsicsToBody(Pic0,Eigen::Quaterniond(Ric0));
            vision_ = new FrameHandlerMono(cam0);
        }

		switch (SLAM_CONTEXT.video_source)
		{
		case 6:
			video_source_ = new VideoSourceEuroc();
		}


	}

	void SlamSystem::run() {



	}


}
