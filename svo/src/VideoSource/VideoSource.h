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
	virtual bool GetFrame(cv::Mat& img, double& timestamp, double& exposure, Sophus::SE3*gt_Twc){
		bool ret = GetFrame(img, timestamp, exposure);
		if (hasGroundTruth&&gt_Twc){
			if (pose.count(timestamp)){
				*gt_Twc = pose[timestamp];
			}
			else {
				gt_Twc->translation().setZero();
				gt_Twc->setRotationMatrix(Eigen::Matrix3d::Identity());
			}
		}
		return ret;
	}

	std::map<double, Sophus::SE3, std::less<double>, Eigen::aligned_allocator<std::pair<const double, Sophus::SE3>>> pose;
	bool hasGroundTruth = false;
};

}