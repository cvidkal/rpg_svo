#pragma once

#include <VideoSource/VideoSource.h>

namespace slam {
	class VideoSourceEuroc :public VideoSource
	{
	public:
		VideoSourceEuroc();
		~VideoSourceEuroc(){};
		bool GetFrame(cv::Mat& img, double& timestamp, double& exposure) override;
		virtual bool GetFrameStereo(cv::Mat& imgLeft, cv::Mat& imgRight, double& timestamp);
	private:
		FILE* cam0, *cam1, *gt;
		std::vector<std::pair<double, std::string> > imgLeft;
		std::vector<std::pair<double, std::string> > imgRight;
		int internal_id = 0;
	};

}