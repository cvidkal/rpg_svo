#pragma once

#include <opencv2/opencv.hpp>
#include "util/Config.h"
#include <sophus/se3.h>

namespace slam {

class VideoSource
{
public:
	virtual ~VideoSource() = default;

	virtual bool GetFrame(cv::Mat& img, double& timestamp, double& exposure) = 0;
	virtual bool GetFrameStereo(cv::Mat& imgLeft, cv::Mat& imgRight, double& timestamp) = 0;
	virtual bool getGroundTruth(double timestamp, Sophus::SE3&Twc)
	{
		return false;
	};
	std::vector<std::pair<double, Sophus::SE3>> poses;
	bool hasGroundTruth = false;
};

}