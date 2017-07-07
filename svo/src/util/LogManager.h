#pragma once

#include "util/Config.h"
#include <fstream>


// use LogMgr short for LogManager::GetInstance()
#define	LogMgr			LogManager::GetInstance()

#define TRACKING_LOG			"tracking_log.txt"
#define IMU_LOG					"imu.csv"
#define TIMES_LOG				"times_log.txt"
#define IMU_ESTIMATOR_LOG		"imu_estimator_log.txt"
#define FUSION_LOG				"fusion_log.txt"
#define PREDICT_LOG				"predict_log.txt"
#define TIGHTLY_LOG				"tightly_log.txt"

/*
 * Log Manager
 */
class LogManager
{
private:
	LogManager();
public:
	// singleton
	static LogManager& GetInstance() {
		static LogManager instance;
		return instance; 
	}

	~LogManager(); 

private:
	bool openLogFiles();
	bool closeLogFiles(); 

	bool openFile(std::ofstream& file, std::string name);
	
public:
	// tracking log
	std::ofstream tracking_log; 
	// imu raw data
	std::ofstream imu_log; 
	// log tracking times
	std::ofstream times_log; 
	// log imu estimator result
	std::ofstream imu_estimator_log;
	// fusion log
	std::ofstream fusion_log;
	// predict log
	std::ofstream predict_log;
	// tightly log
	std::ofstream tightly_log; 

};