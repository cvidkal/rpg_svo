#pragma once

#include "util/Config.h"
#include <string>
#include "opencv2/opencv.hpp"

// use CONTEXT short for SLAMContext::GetInstance()
#define	SLAM_CONTEXT			SLAMContext::GetInstance()

#define SLAM_SETTING_FILE			"settings.yaml"

#define CAMERA_CONFIG_FILE			"RGB_Config.xml"
#define CAMERA_CALIBRATION_FILE		"camera.txt"

#define IMU_CONFIG_FILE				"imu_cam.yaml"
#define IMU_CALIBRATION_FILE		"imu_calib.yaml"

/*
 * Global environment and settings
 */
class SLAMContext
{
private:
	SLAMContext();
public:
	// singleton
	static SLAMContext& GetInstance();

	// config root
	inline std::string getConfigDirectory() {
		return mConfigDirectory;
	}

	// log root
	inline std::string getLogDirectory() {
		return mLogDirectory;
	}

	// slam setting path
	inline std::string getSettingPath() {
		return mConfigDirectory + SLAM_SETTING_FILE;
	}

	// camera config file
	inline std::string getCameraConfigPath() {
		return mConfigDirectory + CAMERA_CONFIG_FILE;
	}

	// camera calibration file
	inline std::string getCameraCalibrationPath() {
		return mConfigDirectory + CAMERA_CALIBRATION_FILE;
	}

	// imu config file
	inline std::string getImuConfigPath() {
		return mConfigDirectory + IMU_CONFIG_FILE;
	}

	// imu calibration file
	inline std::string getImuCalibrationPath() {
		return mConfigDirectory + IMU_CALIBRATION_FILE;
	}

private:
	// config root
	std::string mConfigDirectory;
	// log root
	std::string mLogDirectory;

	// 
	bool init(); 

	bool loadSettings();

	// get setting value
	template<class T>
	inline T getSetting(cv::FileStorage& fs, std::string key, T _default) {
		T value;
		cv::FileNode node = fs[key];
		if (node.isNone())
			value = _default;
		else
			node >> value;

		return value;
	}

///----------------------------------------------------------------------------
// settings
public:

	// use imu
	bool use_imu;
	bool use_hand_occlusion;
	bool use_tracking_fusion;
	bool use_tightly_coupled; 
	bool use_tracking_thread;
	bool use_inverse_preintegration; 
	bool use_fast_reinit; 
	bool use_localMap;

	bool check_overlap;

	// video source: 0 - dataset, 1 - cameralib, 3- benchmark, 6 - Euroc
	int video_source;
	// debayer image
	bool use_debayer; 
	// ar or vr: 0 - AR, 1 - VR
	int output_rate; 

	// preset
	int preset;
	// mode
	int photometric_mode;
	bool linearizeOperation;
	// 
	int run_fps;

	// dataset path
	std::string dataset_path;
	std::string image_path;
	std::string times_path;
	std::string calib_path;
	std::string gamma_path;
	std::string vignette_path;
	void setDataset(const std::string path); 
	// image format
	std::string image_format;

	// disable all display
	bool disable_display;
    bool disable_pangolin;
	bool renderWindowFrames;
	bool show_residual;

	// debug log level
	int debug_log_level;


};