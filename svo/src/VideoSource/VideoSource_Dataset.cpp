#include "stdafx.h"
#include "VideoSource_Dataset.h"
#include <fstream>
#include <sophus/se3.h>

#if USE_IMU
#include "IMU/InertialSensor/InertialSensor_Dataset.h"
#endif


namespace slam {

//-----------------------------------------------------------------------------
VideoSource_Dataset::VideoSource_Dataset(const string& path, const char* format, bool bStereo)
	: mIndex(1), mImageFormat(format), mbStereo(bStereo)
{
	bool bLoaded = false;
	if (mbStereo) {
		bLoaded = LoadImagesStereo(string(path));
	}
	else {
		bLoaded = LoadImages(string(path));
	}
	if (!bLoaded) {
		printf("ERROR: Load images failed!! %s\n", path.c_str());
	}
}


//-----------------------------------------------------------------------------
VideoSource_Dataset::~VideoSource_Dataset()
{
}


//-----------------------------------------------------------------------------
bool VideoSource_Dataset::LoadImages(const string& strPath)
{

	string strPrefixLeft = strPath + "/";

	/// load times
	string path = strPath + "/../times.txt";
	ifstream camFile;
	camFile.open(path.c_str());
	if (!camFile.is_open()) {
		cout << "Open File Failed: " << path << endl;
		return false;
	}


	while (!camFile.eof())
	{
		string s;
		string name;
		double t, dt;
		getline(camFile, s);

		if (!s.empty())
		{
			stringstream ss;
			ss << s;

			ss >> name;
			mvStrImageLeft.push_back(strPrefixLeft + name + "." + mImageFormat);
			ss >> t;
			mvTimestamps.push_back(t);
			ss >> dt;
			mvExposures.push_back(dt);
		}
	}



	/// check image
	cv::Mat img = cv::imread(mvStrImageLeft[0], CV_LOAD_IMAGE_UNCHANGED);
	if (img.empty()) {
		return false;
	}

	return true;
}


//-----------------------------------------------------------------------------
bool VideoSource_Dataset::LoadImagesStereo(const string& strPath)
{
	string strPrefixLeft = strPath + "/cam0/";
	string strPrefixRight = strPath + "/cam1/";

	string path = strPath + "/times.txt";
	ifstream camFile;
	camFile.open(path.c_str());
	if (!camFile.is_open()) {
		cout << "Open File Failed: " << path << endl;
		return false;
	}

	while (!camFile.eof())
	{
		string s;
		string name;
		double t;
		getline(camFile, s);

		if (!s.empty())
		{
			stringstream ss;
			ss << s;

			ss >> name;
			mvStrImageLeft.push_back(strPrefixLeft + name + "." + mImageFormat);
			mvStrImageRight.push_back(strPrefixRight + name + "." + mImageFormat);
			ss >> t;
			mvTimestamps.push_back(t);
			mvExposures.push_back(1.0);
		}
	}

	cv::Mat img = cv::imread(mvStrImageLeft[0], CV_LOAD_IMAGE_UNCHANGED);
	if (img.empty()) {
		return false;
	}

	return true;
}


//-----------------------------------------------------------------------------
bool VideoSource_Dataset::GetFrame(cv::Mat& img, double& timestamp, double& exposure)
{
	if (mIndex >= mvTimestamps.size())
		return false;

	static cv::Mat imgRGB;

	timestamp = mvTimestamps[mIndex];
	exposure = mvExposures[mIndex];

#if USE_IMU
	if (SLAM_CONTEXT.use_imu)
		InertialSensor_Dataset::NotifySendImuData(timestamp);
#endif

	imgRGB = cv::imread(mvStrImageLeft[mIndex], CV_LOAD_IMAGE_UNCHANGED);
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

	mIndex++;
	return true;
}


//-----------------------------------------------------------------------------
bool VideoSource_Dataset::GetFrameStereo(cv::Mat& imgLeft, cv::Mat& imgRight, double& timestamp)
{
	if (mIndex >= mvTimestamps.size())
		return false;

	static cv::Mat leftImRGB, rightImRGB;

	timestamp = mvTimestamps[mIndex];

#if USE_IMU
	if (SLAM_CONTEXT.use_imu)
		InertialSensor_Dataset::NotifySendImuData(timestamp);
#endif

	leftImRGB = cv::imread(mvStrImageLeft[mIndex], CV_LOAD_IMAGE_UNCHANGED);
	rightImRGB = cv::imread(mvStrImageRight[mIndex], CV_LOAD_IMAGE_UNCHANGED);
	if (leftImRGB.empty() || rightImRGB.empty()) {
		return false;
	}

	cv::cvtColor(leftImRGB, imgLeft, CV_BGR2GRAY);
	cv::cvtColor(rightImRGB, imgRight, CV_BGR2GRAY);

	//cv::imshow("left", imgLeft);
	//cv::imshow("right", imgRight);
	//cv::waitKey();

	mIndex++;
	return true;
}

}