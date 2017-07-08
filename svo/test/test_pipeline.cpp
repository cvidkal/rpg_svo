// This file is part of SVO - Semi-direct Visual Odometry.
//
// Copyright (C) 2014 Christian Forster <forster at ifi dot uzh dot ch>
// (Robotics and Perception Group, University of Zurich, Switzerland).
//
// SVO is free software: you can redistribute it and/or modify it under the
// terms of the GNU General Public License as published by the Free Software
// Foundation, either version 3 of the License, or any later version.
//
// SVO is distributed in the hope that it will be useful, but WITHOUT ANY
// WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS
// FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program.  If not, see <http://www.gnu.org/licenses/>.

#include <svo/config.h>
#include <svo/frame_handler_mono.h>
#include <svo/map.h>
#include <svo/frame.h>
#include <vector>
#include <string>
#include <vikit/math_utils.h>
#include <vikit/vision.h>
#include <vikit/abstract_camera.h>
#include <vikit/atan_camera.h>
#include <vikit/pinhole_camera.h>
#include <opencv2/opencv.hpp>
#include <sophus/se3.h>
#include <iostream>
#include "test_utils.h"
#include <iomanip>
#include <svo/feature.h>
#include <svo/point.h>



using namespace cv;
namespace svo {

class BenchmarkNode
{
  vk::AbstractCamera* cam_;
  slam::FrameHandlerMono* vo_;

public:
  BenchmarkNode();
  ~BenchmarkNode();
  void runFromFolder();
};

BenchmarkNode::BenchmarkNode()
{
  //cam_ = new vk::PinholeCamera(320, 240, 240, 240, 159.5, 119.5);
  cam_ = new vk::PinholeCamera(640, 480, 591.0495, 590.7825, 321.4926, 250.3737,0.0299,-0.1178,0.0069,-0.0018);
	//cam_ = new vk::PinholeCamera(752, 480, 315.5, 315.5, 376.0, 240.0);
	vo_ = new slam::FrameHandlerMono(cam_);
  vo_->start();
}

BenchmarkNode::~BenchmarkNode()
{
  delete vo_;
  delete cam_;
}

void BenchmarkNode::runFromFolder()
{
//#define LIVE_VIDEO
#ifdef LIVE_VIDEO
	VideoCapture vc(1);
	cv::Mat img;
	int img_id = 0;
	int startId = 50;
	while (startId--)
		vc >> img;
	while (true)
	{
		vc >> img;
		Mat gray;
		cvtColor(img, gray, CV_BGR2GRAY);
		// process frame
		vo_->addImage(gray, 0.01*img_id);
		++img_id;


		{
			cv::Mat draw = img.clone();
			if (vo_->lastFrame()) {
				for (auto fts : vo_->lastFrame()->fts_)
				{
					if (fts->point) {
						cv::circle(draw, Point2f(fts->px.x(), fts->px.y()), 2, CV_RGB(255, 0, 0));
						auto px = vo_->lastFrame()->f2c(vo_->lastFrame()->T_f_w_*fts->point->pos_);
						cv::drawMarker(draw, Point2f(px.x(), px.y()), CV_RGB(0, 255, 0));
					}
				}
				imshow("draw", draw);
				cvWaitKey(1);
			}
		}
	}
#endif
  for(int img_id = 100; img_id < 1000; ++img_id)
  {
    // load image
    std::stringstream ss;/*
    ss << svo::test_utils::getDatasetDir() << "/sequence_30/images_rect/"
       << std::setw( 5 ) << std::setfill( '0' ) << img_id << ".jpg";*/
	//ss << svo::test_utils::getDatasetDir() << "/sin2_tex2_h1_v8_d/img/frame_"
	//	<< std::setw(6) << std::setfill('0') << img_id << "_0.png";
	ss << "C:/Users/Hua/Documents/dso/captureImage/" << std::setw(5) << std::setfill('0') << img_id << ".jpg";
    if(img_id == 2)
      std::cout << "reading image " << ss.str() << std::endl;
    cv::Mat img(cv::imread(ss.str().c_str(), 0));
    assert(!img.empty());

    // process frame
    vo_->addImage(img, 0.01*img_id);

    {
		cv::Mat draw;
		draw = imread(ss.str());
		for (auto fts:vo_->lastFrame()->fts_)
		{
			if (fts->point) {
				cv::circle(draw, Point2f(fts->px.x(), fts->px.y()), 2, CV_RGB(255, 0, 0));
				auto px = vo_->lastFrame()->f2c(vo_->lastFrame()->T_f_w_*fts->point->pos_);
				cv::drawMarker(draw, Point2f(px.x(), px.y()), CV_RGB(0, 255, 0));
			}
		}
		imshow("draw", draw);
		cvWaitKey(1);
    }

    // display tracking quality
    if(vo_->lastFrame() != NULL)
    {
    	std::cout << "Frame-Id: " << vo_->lastFrame()->id_ << " \t"
                  << "#Features: " << vo_->lastNumObservations() << " \t"
                  << "Proc. Time: " << vo_->lastProcessingTime()*1000 << "ms \n";

    	// access the pose of the camera via vo_->lastFrame()->T_f_w_.
    }
  }
}

} // namespace svo


void rectifyImage(vk::AbstractCamera*cam_origin,vk::AbstractCamera*cam_now,const cv::Mat&origin,cv::Mat&dst)
{
	auto width = cam_now->width();
	auto height = cam_now->height();
	dst = cv::Mat(height, width, CV_8U);
	for (int i = 0; i < height; ++i) {
		auto p = (uchar*)dst.ptr<uchar>(i);
		for (int j = 0; j < width; ++j)
		{
			auto f = cam_now->cam2world(j, i);
			auto px = cam_origin->world2cam(f);
			if (cam_origin->isInFrame(px.cast<int>()))
			{
				p[j] = vk::interpolateMat_8u(origin, px.x(), px.y());
			}
		}

	}

}



int main(int argc, char** argv)
{

	//vk::ATANCamera cam_origin(1280,1024, 0.535719308086809,0.669566858850269,0.493248545285398,0.500408664348414,0.897966326944875);
	//vk::PinholeCamera cam_now(320, 240, 240, 240, 159.5,119.5);
	//for (int i=0;i<1800;++i)
	//{
	//	char ss[200];
	//	sprintf(ss, "E:/dataset/sequence_30/images/%05d.jpg", i);
	//	cv::Mat origin = cv::imread(ss,0);
	//	cv::Mat dst;
	//	rectifyImage(&cam_origin, &cam_now, origin, dst);
	//	sprintf(ss, "E:/dataset/sequence_30/images_rect/%05d.jpg", i);
	//	imwrite(ss, dst);
	//}
	//return 0;
  {
    svo::BenchmarkNode benchmark;
    benchmark.runFromFolder();
  }
  printf("BenchmarkNode finished.\n");
  return 0;
}

