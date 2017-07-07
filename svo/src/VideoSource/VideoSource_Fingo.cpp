#include "stdafx.h"

#if USE_FINGO

#include "VideoSource_Fingo.h"

#ifdef PLATFORM_WINDOWS
#include <Windows.h>    // for Sleep()
#endif

#ifdef PLATFORM_ANDROID
#ifndef Sleep
#include <unistd.h> // for usleep()
#define Sleep(msec) usleep((msec)*1000)
#endif
#endif

#include <chrono>

#define MAX_WAIT_TIME       (300)

//-----------------------------------------------------------------------------
namespace slam {

//-----------------------------------------------------------------------------
VideoSource_Fingo::VideoSource_Fingo(Fingo::FingoDevice* pFingo)
    : m_pFingo(pFingo), m_Width(0), m_Height(0), m_BytesPerPixel(1)
    , m_bLastImageConsumed(true), m_bFirstFrameArrived(false)
    , m_nCameraID(0)
{
	hasGroundTruth = false;
    pthread_cond_init(&m_CondCameraImagesAvailable, 0);
    pthread_mutex_init(&m_MutCameraImagesAvailable, 0);
    pthread_mutex_init(&m_MutCameraImages, 0);

    m_Frames[0] = m_Frames[1] = 0;

    m_pFingo->addListener(this);

    // need to wait till 1st frame arrived, to get width, height
    int waitTime = 0;
    while (!m_bFirstFrameArrived && waitTime<MAX_WAIT_TIME) {
        ++waitTime;
        Sleep(100);
    }

    if (!m_bFirstFrameArrived) {
        cout << "Error: VideoSource_Fingo - Fingo Device time out, no image received..." << endl;
    }
}


//-----------------------------------------------------------------------------
VideoSource_Fingo::~VideoSource_Fingo()
{
    m_pFingo->removeListener(this);
    pthread_mutex_destroy(&m_MutCameraImagesAvailable);
    pthread_mutex_destroy(&m_MutCameraImages);
}


//-----------------------------------------------------------------------------
bool VideoSource_Fingo::GetFrame(cv::Mat& img, double& timestamp, double& exposure)
{
    if (!m_bFirstFrameArrived) {
		return false; 
	}

    // wait for new frame if we consumed last frame
    if (m_bLastImageConsumed) {
        pthread_mutex_lock(&m_MutCameraImagesAvailable);
        pthread_cond_wait(&m_CondCameraImagesAvailable, &m_MutCameraImagesAvailable);
        pthread_mutex_unlock(&m_MutCameraImagesAvailable);
    }

    // retrieve last images
    pthread_mutex_lock(&m_MutCameraImages);

    cv::Mat frame;
    if (m_BytesPerPixel == 3){
        frame = cv::Mat(m_Height, m_Width, CV_8UC3, m_Frames[0]);
    }
    else{
        frame = cv::Mat(m_Height, m_Width, CV_8UC1, m_Frames[0]);
    }

	timestamp = m_Timestamp * 1e-9;
	exposure = 1.0;

	if (frame.type() == CV_8UC3) {
		cv::cvtColor(frame, img, CV_BGR2GRAY);
	}
	else {
        img = frame.clone();
	}

    m_bLastImageConsumed = true;
    pthread_mutex_unlock(&m_MutCameraImages);
	return true;
}


//-----------------------------------------------------------------------------
bool VideoSource_Fingo::GetFrameStereo(cv::Mat& imgLeft, cv::Mat& imgRight, double& timestamp)
{
    if (!m_bFirstFrameArrived) {
        return false;
    }

    // wait for new frame if we consumed last frame
    if (m_bLastImageConsumed) {
        pthread_mutex_lock(&m_MutCameraImagesAvailable);
        pthread_cond_wait(&m_CondCameraImagesAvailable, &m_MutCameraImagesAvailable);
        pthread_mutex_unlock(&m_MutCameraImagesAvailable);
    }

    // retrieve last images
    pthread_mutex_lock(&m_MutCameraImages);

    cv::Mat frame, frameR;
    if (m_BytesPerPixel == 3){
        frame = cv::Mat(m_Width, m_Height, CV_8UC3, m_Frames[0]);
        frameR = cv::Mat(m_Width, m_Height, CV_8UC3, m_Frames[1]);
    }
    else{
        frame = cv::Mat(m_Width, m_Height, CV_8UC1, m_Frames[0]);
        frameR = cv::Mat(m_Width, m_Height, CV_8UC1, m_Frames[1]);
    }

    timestamp = m_Timestamp;

    if (frame.type() == CV_8UC3) {
        cv::cvtColor(frame, imgLeft, CV_BGR2GRAY);
        cv::cvtColor(frameR, imgRight, CV_BGR2GRAY);
    }
    else {
        imgLeft = frame.clone();
        imgRight = frameR.clone();
    }

    m_bLastImageConsumed = true;
    pthread_mutex_unlock(&m_MutCameraImages);

	return true;
}

//-----------------------------------------------------------------------------
void VideoSource_Fingo::onFingoEvent(const Fingo::EventBase& event)
{
    if (event.getType() != Fingo::EventType::EventType_Image){
        return;
    }
    const Fingo::ImageEvent& imgEvt = dynamic_cast<const Fingo::ImageEvent&>(event);

    // no input yet
    if (imgEvt.mWidth == 0 || imgEvt.mHeight == 0 || imgEvt.mImages[0] == 0) {
        return;
    }

    m_bFirstFrameArrived = true;

    // copy image
    pthread_mutex_lock(&m_MutCameraImages);
    m_Width = imgEvt.mWidth;
    m_Height = imgEvt.mHeight;
    m_BytesPerPixel = imgEvt.mBytesPerPixel;

    // convert from seconds to nano seconds
    m_Timestamp = imgEvt.mTimestamp;
    m_FrameID = imgEvt.mFrameID;
    // only copy image from camera 1
    size_t size = m_Width * m_Height * m_BytesPerPixel;

    if (size>0) {
        if (!m_Frames[0]) {
            m_Frames[0] = (unsigned char*)malloc(size);
            m_Frames[1] = (unsigned char*)malloc(size);
        }
        memcpy(m_Frames[0], imgEvt.mImages[0], size);
        memcpy(m_Frames[1], imgEvt.mImages[1], size);
    }

#if 0
    cv::Mat leftImg, rightImg;
    if (imgEvt.mBytesPerPixel == 1){
        leftImg = cv::Mat(imgEvt.mHeight, imgEvt.mWidth, CV_8UC1, (void*)imgEvt.mImages[0]);
        rightImg = cv::Mat(imgEvt.mHeight, imgEvt.mWidth, CV_8UC1, (void*)imgEvt.mImages[1]);
    }
    else if (imgEvt.mBytesPerPixel == 3){
        leftImg = cv::Mat(imgEvt.mHeight, imgEvt.mWidth, CV_8UC3, (void*)imgEvt.mImages[0]);
        rightImg = cv::Mat(imgEvt.mHeight, imgEvt.mWidth, CV_8UC3, (void*)imgEvt.mImages[1]);
    }
    cv::imwrite(std::string(CONFIG_FILE_PATH) + "leftImg.bmp", leftImg);
    cv::imwrite(std::string(CONFIG_FILE_PATH) + "rightImg.bmp", rightImg);
#endif

    pthread_mutex_unlock(&m_MutCameraImages);

    // signal waiting consumer
    pthread_mutex_lock(&m_MutCameraImagesAvailable);
    m_bLastImageConsumed = false;
    pthread_cond_signal(&m_CondCameraImagesAvailable);
    pthread_mutex_unlock(&m_MutCameraImagesAvailable);
}

}
#endif
