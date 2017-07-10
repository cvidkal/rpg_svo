#pragma once

//
//PLATFORM_DEFINE
//
#if defined(_WIN32) || defined(_WIN64)

#define	 PLATFORM_WINDOWS

#elif defined(ANDROID) || defined(__ANDROID__)

#define  PLATFORM_ANDROID

#elif defined(__APPLE__) && defined(__MACH__) //APPLE_PLATFORM_DEFINE
/* Apple OSX and iOS (Darwin). ------------------------------ */
#define  PLATFORM_APPLE
#include <TargetConditionals.h>
#if TARGET_IPHONE_SIMULATOR == 1
/* iOS in Xcode simulator */
#define  PLATFORM_IOS
#define  PLATFORM_IOS_SIMULATOR
#elif TARGET_OS_IPHONE == 1
/* iOS on iPhone, iPad, etc. */
#define  PLATFORM_IOS
#define  PLATFORM_IOS_PHONE
#elif TARGET_OS_MAC == 1
/* OSX */
#define  PLATFORM_OSX
#else
#error "Unknown Apple OS platform"
#endif //APPLE_PLATFORM_DEFINE
#elif defined(__linux__)
#define PLATFORM_LINUX
#else

#error "Unknown OS platform"

#endif //PLATFORM_DEFINE
