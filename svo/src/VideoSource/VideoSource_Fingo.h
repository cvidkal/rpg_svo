#pragma once

#include "util/Config.h"
#if USE_FINGO

#include "VideoSource.h"

#include <pthread.h>

#include <Fingo.h>

#include <vector>
using namespace std;

namespace slam {

class VideoSource_Fingo 
    : public VideoSource
    , public Fingo::FingoDevice::Listener
{
public:
    VideoSource_Fingo(Fingo::FingoDevice* fingo);

	~VideoSource_Fingo();

	bool GetFrame(cv::Mat& img, double& timestamp, double& exposure) override;
	bool GetFrameStereo(cv::Mat& imgLeft, cv::Mat& imgRight, double& timestamp) override; 


protected:
    /// overrides FingoDevice Listener
    virtual void onTrackingMessage(const Fingo::TrackingMessageBase& message){};
    virtual void onVideoFrameReceived(int cameraCount, const unsigned char* frames[], int width, int height, int bytesPerPixel, double timestamp){};
    virtual void onFingoEvent(const Fingo::EventBase& event);

    Fingo::FingoDevice*     m_pFingo;

    bool                    m_bFirstFrameArrived;
    int                     m_Width;
    int                     m_Height;
    unsigned char*          m_Frames[2];
    int                     m_BytesPerPixel;
    /// timestamp
    long long				m_Timestamp;
    unsigned int			m_FrameID;

    /// camera images available
    pthread_cond_t          m_CondCameraImagesAvailable;
    pthread_mutex_t         m_MutCameraImagesAvailable;
    pthread_mutex_t         m_MutCameraImages;

    /// flag, true if new image arrived after last frame was consumed.
    bool                    m_bLastImageConsumed;

    /// index of which camera in the stereo camera to use
    int                     m_nCameraID;
};

}
#endif
