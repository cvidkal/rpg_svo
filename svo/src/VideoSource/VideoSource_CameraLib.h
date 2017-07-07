#pragma once
#include "util/Config.h"
#include "VideoSource.h"

#if USE_CAMERALIB

#include <CameraLib.h>

#include <vector>
using namespace std;

namespace slam {

class VideoSource_CameraLib : public VideoSource
{
public:
	VideoSource_CameraLib(const char* cameraName = "DS",
		const char* configFile = "config/DS_RGB_Config.xml");

	~VideoSource_CameraLib();

	bool GetFrame(cv::Mat& img, double& timestamp, double& exposure) override;
	bool GetFrameStereo(cv::Mat& imgLeft, cv::Mat& imgRight, double& timestamp) override; 

private:
	CameraManager m_CameraMgr;
	MultiCameraBase* m_MultiCamera;
};

}


#endif