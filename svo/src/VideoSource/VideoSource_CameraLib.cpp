#include "stdafx.h"

#if USE_CAMERALIB

#include "VideoSource_CameraLib.h"

namespace slam {

//-----------------------------------------------------------------------------
VideoSource_CameraLib::VideoSource_CameraLib(const char* cameraName, const char* configFile)
{
	hasGroundTruth = false;
	CameraFactory* cameraFactory = m_CameraMgr.getCameraFactory(cameraName);
	m_MultiCamera = cameraFactory->getMultiCamera();
	if (!m_MultiCamera) {
		return;
	}

	m_MultiCamera->setParameter("ConfigFile", configFile);

	int camera0 = 0;
	int camera1 = 1;
	int index[2] = { camera0, camera1 };
	m_MultiCamera->setCameras(index, 2);
	m_MultiCamera->startCapture();
}


//-----------------------------------------------------------------------------
VideoSource_CameraLib::~VideoSource_CameraLib()
{
	if (m_MultiCamera) {
		m_MultiCamera->stopCapture();
	}
	
}


//-----------------------------------------------------------------------------
bool VideoSource_CameraLib::GetFrame(cv::Mat& img, double& timestamp, double& exposure)
{
	if (!m_MultiCamera) {
		return false; 
	}

	m_MultiCamera->waitForAll();
	CameraFrame frame = m_MultiCamera->lockCameraFrame();
	m_MultiCamera->unlockCameraFrame();

	timestamp = frame.timestamps[0] * 1e-9;
	exposure = 1.0;

	if (frame.frames[0].type() == CV_8UC3) {
		cv::cvtColor(frame.frames[0], img, CV_BGR2GRAY);
	}
	else {
		img = frame.frames[0].clone();
	}
	

	return true;
}


//-----------------------------------------------------------------------------
bool VideoSource_CameraLib::GetFrameStereo(cv::Mat& imgLeft, cv::Mat& imgRight, double& timestamp)
{
	if (!m_MultiCamera) {
		return false;
	}

	m_MultiCamera->waitForAll();
	CameraFrame frame = m_MultiCamera->lockCameraFrame();
	m_MultiCamera->unlockCameraFrame();

	timestamp = frame.timestamps[0] * 1e-9;

	if (frame.frames[0].type() == CV_8UC3) {
		cv::cvtColor(frame.frames[0], imgLeft, CV_BGR2GRAY);
		cv::cvtColor(frame.frames[1], imgRight, CV_BGR2GRAY);
	}
	else {
		imgLeft = frame.frames[0].clone(); 
		imgRight = frame.frames[1].clone(); 
	}

	return true;
}

}


#endif