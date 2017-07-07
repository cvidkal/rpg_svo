#pragma once
#include "util/Config.h"
#include "VideoSource.h"
#include <vector>
#include <opencv2/opencv.hpp>
using namespace std;

namespace slam {

	class VideoSource_WebCam : public VideoSource
	{
	public:
		VideoSource_WebCam(int id = 0) { vc.open(id); hasGroundTruth = false; };

		~VideoSource_WebCam(){};

		bool GetFrame(cv::Mat& img, double& timestamp, double& exposure) override { vc >> img; cv::cvtColor(img, img, CV_BGR2GRAY); timestamp = 1.0; exposure = 1.0f; return true; }
		bool GetFrameStereo(cv::Mat& imgLeft, cv::Mat& imgRight, double& timestamp) override {
			return false;
		}

	private:
		cv::VideoCapture vc;
	};

}
