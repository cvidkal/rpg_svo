//
// Created by fg on 2017/4/6.
//

#ifndef slam_VIDEOSOURCE_TUM_H
#define slam_VIDEOSOURCE_TUM_H

#include <VideoSource/VideoSource.h>
#include <sophus/se3.h>

namespace slam {

class VideoSource_TUM : public VideoSource {
public:
	VideoSource_TUM(const std::string&assoc, const std::string&path);
	virtual bool GetFrame(cv::Mat& img, double& timestamp, double& exposure){
		if (nn >= imgs.size())
			return false;
		img = cv::imread(imgs[nn], 0);
		timestamp = timestamps[nn];
		exposure = 0;
		nn++;
		return true;
	}
	virtual bool GetFrameStereo(cv::Mat& imgLeft, cv::Mat& imgRight, double& timestamp){ return false; }


private:
	int nn = 0;
	std::vector<std::string> imgs;
	std::vector<Sophus::SE3, Eigen::aligned_allocator<Sophus::SE3>> poses;
	std::vector<double> timestamps;
};

}


#endif //slam_VIDEOSOURCE_TUM_H
