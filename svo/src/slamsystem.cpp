#include "stdafx.h"
#include "slamsystem.h"
#include "VideoSource/VideoSource_Euroc.h"

namespace  slam {



	void SlamSystem::Init()
	{
		switch (SLAM_CONTEXT.video_source)
		{
		case 6:
			video_source_ = new VideoSourceEuroc();
		}
	}




}
