#include "stdafx.h"
#include <VideoSource/VideoSource_Euroc.h>
#include <string>
#include <stdio.h>
using namespace std;

namespace slam {

	VideoSourceEuroc::VideoSourceEuroc()
	{
		string dataPath = SLAM_CONTEXT.dataset_path;
		string cam0_Path = dataPath + "/cam0";
		string cam1_Path = dataPath + "/cam1";
		string imu_Path = dataPath + "/imu0";
		string groundTruth_Path = dataPath + "/state_groundtruth_estimate0";
		cam0 = fopen((cam0_Path + "/data.csv").c_str(), "r");
		cam1 = fopen((cam1_Path + "/data.csv").c_str(), "r");
		//imu = fopen((imu_Path + "/data.csv").c_str(), "r");
		gt = fopen((groundTruth_Path + "/data.csv").c_str(), "r");

		//Read Left Images
		{
			char buf[1000];
			int maxN = 1000;
			//read first line
			fgets(buf, maxN, cam0);
			double timestamp = 0.0;
			while (fscanf(cam0, "%lf,%s\n", &timestamp, buf)) {
				imgLeft.push_back(make_pair(timestamp, string(buf)));
			}
		}
		//Read Right Images
		{
			char buf[1000];
			int maxN = 1000;
			//read first line
			fgets(buf, maxN, cam1);
			double timestamp = 0.0;
			while (fscanf(cam1, "%lf,%s\n", &timestamp, buf)) {
				imgLeft.push_back(make_pair(timestamp, string(buf)));
			}
		}

		{
			char buf[1000];
			int maxN = 1000;
			//read first line
			fgets(buf, maxN, gt);
			double timestamp = 0.0;
			double px, py, pz;
			double qw, qx, qy, qz;
			double vx, vy, vz;
			double wx, wy, wz;
			double ax, ay, az;
			while (fscanf(cam1, "%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf\n",
				&timestamp, &px, &py, &pz, &qw, &qx, &qy, &qz, &vx, &vy, &vz, &wx, &wy, &wz, &ax, &ay, &az)) {
				Eigen::Quaterniond q(qw, qx, qy, qz);
				Eigen::Vector3d p(px, py, pz);
				poses.emplace_back(make_pair(timestamp, Sophus::SE3(q, p)));
			}
		}
		fclose(cam0); fclose(cam1); fclose(gt);
	}

	bool VideoSourceEuroc::GetFrame(cv::Mat& img, double& timestamp, double& exposure)
	{
		exposure = 1.0;
		int idx_got;
		if (internal_id >= imgLeft.size())
		{
			std::cout << "No more Images" << endl;
			return false;
		}
		img = cv::imread(imgLeft[internal_id].second);
		timestamp = imgLeft[internal_id].first;
		internal_id++;
		return true;
	}

}
