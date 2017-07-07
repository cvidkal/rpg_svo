#include "stdafx.h"

#include "LogManager.h"


//-----------------------------------------------------------------------------
LogManager::LogManager()
{
	if (!openLogFiles()) {
		exit(1); 
	}
}

//-----------------------------------------------------------------------------
LogManager::~LogManager()
{
	closeLogFiles(); 
}

//-----------------------------------------------------------------------------
bool LogManager::openFile(std::ofstream& file, std::string name)
{
	std::string path = SLAM_CONTEXT.getLogDirectory() + name;
	file.open(path, std::ios::out);
	if (!file.is_open()) {
		printf("ERROR: fail to open file: %s\n", path.c_str());
		return false;
	}

	return true;
}

//-----------------------------------------------------------------------------
bool LogManager::openLogFiles()
{
	// tracking log
	if (!openFile(tracking_log, TRACKING_LOG))
		return false;
	tracking_log << "#timestamp,x,y,z,vx,vy,vz,qw,qx,qy,qz,c1,c2,c3,c4,c5,c6,quality" << std::endl;

	// imu log
	if (!openFile(imu_log, IMU_LOG))
		return false;
	imu_log << "#timestamp,gx,gy,gz,ax,ay,az,mx,my,mz,qw,qx,qy,qz" << std::endl;

	// times log
	if (!openFile(times_log, TIMES_LOG))
		return false;
	times_log << "#timestamp,dt" << std::endl;

	// imu estimator log
	if (!openFile(imu_estimator_log, IMU_ESTIMATOR_LOG))
		return false;
	imu_estimator_log << "#timestamp,scale,gravity_x,gravity_y,gravity_z,bg_x,bg_y,bg_z,ba_x,ba_y,ba_z" << std::endl;

	// fusion log
	if (!openFile(fusion_log, FUSION_LOG))
		return false;
	fusion_log << "#timestamp,x,y,z,vx,vy,vz,qw,qx,qy,qz" << std::endl;

	// predict log
	if (!openFile(predict_log, PREDICT_LOG))
		return false;
	predict_log << "#timestamp,x,y,z,qw,qx,qy,qz" << std::endl;

	// tightly log
	if (!openFile(tightly_log, TIGHTLY_LOG))
		return false;

	return true;
}


//-----------------------------------------------------------------------------
bool LogManager::closeLogFiles()
{
	tracking_log.close(); 
	imu_log.close(); 
	times_log.close(); 
	imu_estimator_log.close(); 
	fusion_log.close();
	predict_log.close(); 
	tightly_log.close(); 

	return true;
}
