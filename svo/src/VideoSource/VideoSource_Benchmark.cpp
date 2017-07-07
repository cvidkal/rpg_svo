#include "stdafx.h"

#ifdef USE_BENCHMARK
#include "VideoSource_Benchmark.h"
#include <fstream>
#include <sophus/se3.h>

#if USE_IMU
#include "IMU/InertialSensor/InertialSensor_Benchmark.h"
#endif

#include <CameraLib.h>

namespace slam {

MultiCameraBase*        m_pMultiCameraBase;
CameraFactory*          m_pCameraFactory;
CameraManager           m_CameraManager;

//-----------------------------------------------------------------------------
VideoSource_Benchmark::VideoSource_Benchmark(const string& path, const char* format, bool bStereo)
	: mIndex(10), mImageFormat(format), mbStereo(bStereo)
{
    cv::FileStorage fs("Dataset_Config.xml", cv::FileStorage::READ);
    fs["CameraConfigFile"] >> m_CameraConfigFile;
    fs["UseStereoRectify"] >> m_bUseStereoRectify;
    fs["DatasetDir"] >> m_DatasetDir;
    fs["TestImageSleep"] >> m_nTestImageSleep;

    std::string camFileName = m_DatasetDir + "/cam.csv";

    std::ifstream camFile;
    camFile.open(camFileName.c_str());
    if (!camFile.is_open()) {
        cout << "Unable to find cam.csv" << endl;
        return;
    }
    std::string strPrefixLeft = m_DatasetDir + "/cam0/";
    std::string strPrefixRight = m_DatasetDir + "/cam1/";

    while (!camFile.eof())
    {
        std::string s;
        getline(camFile, s);
        stringstream ss(s);
        string imgName;
        string receivedTimestamp;
        ss >> imgName;
        ss >> receivedTimestamp;

        if (!imgName.empty())
        {
            m_vStrImageLeft.push_back(strPrefixLeft + imgName);
            m_vStrImageRight.push_back(strPrefixRight + imgName);
            while (imgName.back() != '.'){
                imgName.pop_back();
            }
            imgName.pop_back();
            long long t = std::stoll(imgName);
            m_vTimestampsFingo.push_back(t);
            m_vTimestampsReceived.push_back(std::stoll(receivedTimestamp));
            mvExposures.push_back(1.0);
        }
    }
    camFile.close();

    m_nNumOfFrames = m_vTimestampsFingo.size();

    // no file at all
    if (m_nNumOfFrames == 0) {
        cout << "Unable to find image in " << m_DatasetDir << endl;
        return;
    }

    m_nCurrentIndex = 0;

    // create multi camera
    m_pCameraFactory = m_CameraManager.getCameraFactory("DS");
    m_pMultiCameraBase = m_pCameraFactory->getMultiCamera();
    m_pMultiCameraBase->setParameter("ConfigFile", m_CameraConfigFile);
    m_pMultiCameraBase->setCameras(0, 0);
}


//-----------------------------------------------------------------------------
VideoSource_Benchmark::~VideoSource_Benchmark()
{
}

//-----------------------------------------------------------------------------
bool VideoSource_Benchmark::GetFrame(cv::Mat& img, double& timestamp, double& exposure)
{

    if (mIndex >= m_vTimestampsFingo.size()){
        cout << "done" << endl;
        exit(0);
        return false;
    }

	static cv::Mat imgRGB;

    timestamp = m_vTimestampsFingo[mIndex] * 1e-9;
	exposure = mvExposures[mIndex];

#if USE_IMU
	if (SLAM_CONTEXT.use_imu)
		InertialSensor_Benchmark::NotifySendImuData(timestamp);
#endif

    imgRGB = cv::imread(m_vStrImageLeft[mIndex], CV_LOAD_IMAGE_UNCHANGED);
	if (imgRGB.empty()) {
		return false;
	}

	if (imgRGB.type() == CV_8UC3) {
		cv::cvtColor(imgRGB, img, CV_BGR2GRAY);
	}
	else {
		img = imgRGB;
	}
	//cv::imshow("left", img);
	//cv::waitKey();
    if (m_bUseStereoRectify){
        m_pMultiCameraBase->rectifyImage(0, img, img);
    }
	mIndex++;
	return true;
}


//-----------------------------------------------------------------------------
bool VideoSource_Benchmark::GetFrameStereo(cv::Mat& imgLeft, cv::Mat& imgRight, double& timestamp)
{
    if (mIndex >= m_vTimestampsFingo.size())
		return false;

	static cv::Mat leftImRGB, rightImRGB;

    timestamp = m_vTimestampsFingo[mIndex];

#if USE_IMU
	if (SLAM_CONTEXT.use_imu)
		InertialSensor_Benchmark::NotifySendImuData(timestamp);
#endif

	leftImRGB = cv::imread(m_vStrImageLeft[mIndex], CV_LOAD_IMAGE_UNCHANGED);
	rightImRGB = cv::imread(m_vStrImageRight[mIndex], CV_LOAD_IMAGE_UNCHANGED);
	if (leftImRGB.empty() || rightImRGB.empty()) {
		return false;
	}

    if (leftImRGB.type() == CV_8UC3) {
        cv::cvtColor(leftImRGB, imgLeft, CV_BGR2GRAY);
        cv::cvtColor(rightImRGB, imgRight, CV_BGR2GRAY);
    }
    else {
        imgLeft = leftImRGB;
        imgRight = rightImRGB;
    }


	//cv::imshow("left", imgLeft);
	//cv::imshow("right", imgRight);
	//cv::waitKey();
    if (m_bUseStereoRectify){
        m_pMultiCameraBase->rectifyImage(0, imgLeft, imgLeft);
        m_pMultiCameraBase->rectifyImage(1, imgRight, imgRight);
    }
	mIndex++;
	return true;
}

}
#endif