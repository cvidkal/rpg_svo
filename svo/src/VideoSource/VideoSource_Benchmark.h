#pragma once

#include "VideoSource.h"

#include <vector>
using namespace std;

namespace slam {

class VideoSource_Benchmark : public VideoSource
{
public:
	VideoSource_Benchmark(const string& path, const char* format = "jpg", bool bStereo = false);
	~VideoSource_Benchmark();

	bool GetFrame(cv::Mat& img, double& timestamp, double& exposure) override;
	bool GetFrameStereo(cv::Mat& imgLeft, cv::Mat& imgRight, double& timestamp) override;

private:
	string mImageFormat;
	bool mbStereo;
	int mIndex;

	vector<double> mvExposures;

private:
    bool                        m_bUseStereoRectify;
    int                         m_nTestImageSleep;
    std::string                 m_DatasetDir;
    std::string                 m_CameraConfigFile;

    int                         m_nNumOfFrames;
    int                         m_nCurrentIndex;
    std::vector<std::string>    m_vStrImageLeft;
    std::vector<std::string>    m_vStrImageRight;
    std::vector<long long>      m_vTimestampsFingo;
    std::vector<long long>      m_vTimestampsReceived;
};


}