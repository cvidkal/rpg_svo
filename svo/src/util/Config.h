#pragma once

#include "targetplaform.h"

//-----------------------------------------------------------------------------
// sse or neon
//#if defined(__ARM_NEON)
#ifdef PLATFORM_ANDROID
#include "util/SSE2NEON.h"
#endif

//-----------------------------------------------------------------------------
// printf
#ifdef PLATFORM_ANDROID
#include <android/log.h>
//#define  printf(...)  __android_log_print(ANDROID_LOG_INFO, "SLAM", __VA_ARGS__)
#else
#include <stdio.h>
#endif

//-----------------------------------------------------------------------------
// use cameraLib
#ifdef PLATFORM_WINDOWS
#	define USE_CAMERALIB				0
#else
#	define USE_CAMERALIB				0
#endif
#if USE_CAMERALIB
#    ifdef NDEBUG
#		ifdef _WIN64
#       pragma comment(lib, "pthreadVC2.lib")
#       pragma comment(lib, "CameraLib64.lib")
#       pragma comment(lib, "FingoUtils64.lib") 
#		elif _WIN32
#       pragma comment(lib, "pthreadVC2.lib")
#       pragma comment(lib, "CameraLib.lib")
#       pragma comment(lib, "FingoUtils.lib") 
#		endif
#   else
#		ifdef _WIN64
#       pragma comment(lib, "pthreadVC2.lib")
#       pragma comment(lib, "CameraLib64d.lib")
#       pragma comment(lib, "FingoUtils64d.lib") 
#		elif _WIN32
#       pragma comment(lib, "pthreadVC2.lib")
#       pragma comment(lib, "CameraLibd.lib")
#       pragma comment(lib, "FingoUtilsd.lib") 
#		endif
#    endif
#endif

//-----------------------------------------------------------------------------
// use fingo
#define USE_FINGO					0
#if USE_FINGO
#ifdef PLATFORM_WINDOWS
#    ifdef NDEBUG
#		ifdef _WIN64
#       pragma comment(lib, "Fingo64.lib")
#		elif _WIN32
#       pragma comment(lib, "Fingo.lib")
#		endif
#   else
#		ifdef _WIN64
#       pragma comment(lib, "Fingo64_d.lib")
#		elif _WIN32
#       pragma comment(lib, "Fingo_d.lib")
#		endif
#    endif
#endif
#endif


//-----------------------------------------------------------------------------
// use imu
#ifdef PLATFORM_ANDROID
#define USE_IMU						1
#else
#define USE_IMU						1
#endif

#if USE_IMU

// use yei imu
#ifdef PLATFORM_WINDOWS
#	define USE_IMU_YEI				0

#	if USE_IMU_YEI
#		ifdef NDEBUG
#			ifdef _WIN64
#			pragma comment(lib, "ThreeSpace_API64.lib")
#			elif _WIN32
#			pragma comment(lib, "ThreeSpace_API.lib")
#			endif
#		else
#			ifdef _WIN64
#			pragma comment(lib, "ThreeSpace_API64d.lib")
#			elif _WIN32
#			pragma comment(lib, "ThreeSpace_API.lib")
#			endif
#		endif
#	endif

#endif

// imu tracking fusion
#	define USE_TRACKING_FUSION		0


#endif


//-----------------------------------------------------------------------------
// use hand occlusion algorithm
#define USE_HAND_OCCLUSION			1

#define USE_OMNIDIRECTIONAL_CAMERAMODEL 1

