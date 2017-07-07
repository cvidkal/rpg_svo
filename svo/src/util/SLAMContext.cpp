#include "stdafx.h"

#include "SLAMContext.h"
#include <fstream>


//-----------------------------------------------------------------------------
SLAMContext::SLAMContext()
{
	init();
	loadSettings();
}

//-----------------------------------------------------------------------------
SLAMContext& SLAMContext::GetInstance()
{
	static SLAMContext instance;
	return instance;
}


//-----------------------------------------------------------------------------
bool SLAMContext::init()
{
	/// config root
	std::string device_name = "default";

	std::string config_root;
#ifdef PLATFORM_ANDROID
	config_root = "/sdcard/FingoConfig/config/";
#else
	config_root = "config/";
#endif

	std::ifstream configFile;
	configFile.open(config_root + "config.txt");
	if (configFile.is_open() && !configFile.eof()) {
		std::string s;
		std::getline(configFile, s);
		if (!s.empty()) {
			device_name = s;
		}
	}
	//std::cout << device_name << std::endl;

	mConfigDirectory = config_root + device_name + "/";

	/// log root
#ifdef PLATFORM_ANDROID
	mLogDirectory = "/sdcard/FingoConfig/logs/";
#else
	mLogDirectory = "logs/";
#endif

	return true;
}


//-----------------------------------------------------------------------------
void SLAMContext::setDataset(const std::string path)
{
	dataset_path = path;

	image_path = dataset_path + "images/";
	times_path = dataset_path + "times.txt";
	calib_path = dataset_path + "camera.txt";
	gamma_path = dataset_path + "pcalib.txt";
	vignette_path = dataset_path + "vignette.png";
}

//-----------------------------------------------------------------------------
bool SLAMContext::loadSettings()
{
	// get setting file path
	std::string setting_path = getSettingPath();

	// open setting file
	cv::FileStorage fs(setting_path, cv::FileStorage::READ);
	if (!fs.isOpened()) {
		//printf("Error opening setting file: '%s'\n", setting_path);
		return false;
	}


	//-----------------------------------------------------
	// Common Settings

	// use imu
	use_imu = getSetting(fs, "use_imu", false);
	if (use_imu) {
		use_tracking_fusion = getSetting(fs, "use_tracking_fusion", false);
		use_tightly_coupled = getSetting(fs, "use_tightly_coupled", false);
		use_inverse_preintegration = getSetting(fs, "use_inverse_preintegration", true);
	}
	else {
		use_tracking_fusion = false;
		use_tightly_coupled = false;
		use_inverse_preintegration = false;
	}
	
	// hand occulusion
	use_hand_occlusion = getSetting(fs, "use_hand_occlusion", false);

	// tracking thread
	use_tracking_thread = getSetting(fs, "use_tracking_thread", false); 

	// fast reinitialize
	use_fast_reinit = getSetting(fs, "use_fast_reinit", false); 

	//local map
	use_localMap = getSetting(fs, "use_localMap", false);

	// check overlap
	check_overlap = getSetting(fs, "check_overlap", false);

	// video source: 0 - dataset, 1 - cameralib
	video_source = getSetting(fs, "video_source", 1); 

	// debayer image
	use_debayer = getSetting(fs, "use_debayer", false);

	// output rate: 0 - image, 1 - imu
	output_rate = getSetting(fs, "output_rate", 0); 

	//-----------------------------------------------------
	// slam Settings

	// preset
	preset = getSetting(fs, "preset", 5);
	// mode
	photometric_mode = getSetting(fs, "photometric_mode", 1);
	//
	linearizeOperation = getSetting(fs, "linearizeOperation", false); 
	// 
	run_fps = getSetting(fs, "run_fps", 0);

	//-----------------------------------------------------
	// Dataset Settings

	// dataset path
	if (video_source == 0) {
		// dataset path
		dataset_path = getSetting(fs, "dataset_path", getConfigDirectory());

		// image format
		image_format = getSetting(fs, "image_format", std::string("bmp"));
	}
	else 
		dataset_path = getConfigDirectory();

	setDataset(dataset_path);

	//-----------------------------------------------------
	// Display Settings
	disable_display = getSetting(fs, "disable_display", false); 
	disable_pangolin = getSetting(fs,"disable_pangolin",false);
    renderWindowFrames = getSetting(fs,"renderWindowFrames",false);
	show_residual = getSetting(fs, "show_residual", false);

	//-----------------------------------------------------
	// Debug Settings

	// debug_log_level
	debug_log_level = getSetting(fs, "debug_log_level", 0);

	// release
	fs.release();

	return true;
}

