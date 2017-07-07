//
// Created by fg on 2017/4/6.
//
#include "stdafx.h"
#include "VideoSource_TUM.h"
#include <string>
#include <fstream>

using namespace std;
using  namespace cv;

namespace slam {

VideoSource_TUM::VideoSource_TUM(const std::string&assoc, const std::string&path){

	std::ifstream assoc_file;
	assoc_file.open(assoc);
	if (!assoc_file.is_open()) {
		printf("Assoc file open failed!\n");
		exit(1);
	}
	hasGroundTruth = true;
	while (!assoc_file.eof()) {
		string s;
		stringstream ss;
		getline(assoc_file, s);
		ss << s;
		double timestamp1, timestamp2;
		ss >> timestamp1;
		string path2;
		ss >> path2;
		ss >> timestamp2;
		timestamps.push_back(timestamp1);
		imgs.push_back(path + "/" + path2);
		Sophus::SE3 pose;
		Eigen::Matrix<double, 7, 1> p;
		ss >> p[0] >> p[1] >> p[2] >> p[6] >> p[3] >> p[4] >> p[5];
		pose.translation() = p.head<3>();
		pose.setQuaternion(Eigen::Quaterniond(p.tail<4>()));
		poses.push_back(pose);
	}
}

}