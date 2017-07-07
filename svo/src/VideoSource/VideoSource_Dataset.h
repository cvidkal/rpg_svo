#pragma once

#include "VideoSource.h"

#include <vector>
using namespace std;

namespace slam {

class VideoSource_Dataset : public VideoSource
{
public:
	VideoSource_Dataset(const string& path, const char* format = "jpg", bool bStereo = false);
	~VideoSource_Dataset();

	bool GetFrame(cv::Mat& img, double& timestamp, double& exposure) override;
	bool GetFrameStereo(cv::Mat& imgLeft, cv::Mat& imgRight, double& timestamp) override;

private:
	string mImageFormat;
	bool mbStereo;
	int mIndex;

	vector<string> mvStrImageLeft;
	vector<string> mvStrImageRight; 
	vector<double> mvTimestamps;
	vector<double> mvExposures;

	bool LoadImages(const string& strPath);
	bool LoadImagesStereo(const string& strPath);

};

}
