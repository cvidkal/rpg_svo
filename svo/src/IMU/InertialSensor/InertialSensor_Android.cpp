/*************************************************************************
* Copyright (c) 2013-2014 Impression.Pi, Inc.
*
* All rights reserved. No warranty, explicit or implicit, provided.
*/

#include "stdafx.h"

#include "InertialSensor_Android.h"
#include <cassert>
#include <unistd.h>
#include <android/log.h>

#define  LOG_TAG    "IMU"
#define  LOGI(...)  __android_log_print(ANDROID_LOG_INFO, LOG_TAG, __VA_ARGS__)

//-----------------------------------------------------------------------------
InertialSensor_Android::InertialSensor_Android(const std::string& configFile)
	: InertialSensor(configFile)
{
	mTimstamp_acc = mTimstamp_gyro = mTimstamp_mag = -1;

	Initialize();
}

//-----------------------------------------------------------------------------
InertialSensor_Android::~InertialSensor_Android()
{
	Uninitialize();
}

//-----------------------------------------------------------------------------
bool InertialSensor_Android::Initialize(void)
{
	// get sensor manager
	mpSensorManager = ASensorManager_getInstance();
	assert(mpSensorManager != NULL);

	// get sensors
	mpAccSensor = ASensorManager_getDefaultSensor(mpSensorManager, ASENSOR_TYPE_ACCELEROMETER);
	assert(mpAccSensor != NULL);
	mpGyroSensor = ASensorManager_getDefaultSensor(mpSensorManager, ASENSOR_TYPE_GYROSCOPE);
	assert(mpGyroSensor != NULL);
	mpMagSensor = ASensorManager_getDefaultSensor(mpSensorManager, ASENSOR_TYPE_MAGNETIC_FIELD);
	assert(mpMagSensor != NULL);
	mpRotSensor = ASensorManager_getDefaultSensor(mpSensorManager, ASENSOR_TYPE_ROTATION_VECTOR);

	mpLooper = ALooper_forThread();
	if (mpLooper == NULL) {
		mpLooper = ALooper_prepare(ALOOPER_PREPARE_ALLOW_NON_CALLBACKS);
	}
	assert(mpLooper != NULL);

	mpSensorEventQueue = ASensorManager_createEventQueue(mpSensorManager, mpLooper, 3, NULL, NULL);
	assert(mpSensorEventQueue != NULL);

	// start
	Start();

	return true;
}

//-----------------------------------------------------------------------------
void InertialSensor_Android::Uninitialize()
{
	Stop();

	if (mpLooper) {
		ALooper_release(mpLooper);
	}

}

//-----------------------------------------------------------------------------
void InertialSensor_Android::Start()
{
	if (mpSensorEventQueue == NULL)
		return;

	if (mpAccSensor)
	{
		ASensorEventQueue_enableSensor(mpSensorEventQueue, mpAccSensor);
		int minDelay = ASensor_getMinDelay(mpAccSensor);
		ASensorEventQueue_setEventRate(mpSensorEventQueue, mpAccSensor, minDelay);// microsecond

		//LOGI("The minimum delay of type %d allowed between events  is %d microseconds", SENSOR_ACCELEMETER, minDelay);
	}
	if (mpGyroSensor)
	{
		ASensorEventQueue_enableSensor(mpSensorEventQueue, mpGyroSensor);
		int minDelay = ASensor_getMinDelay(mpGyroSensor);
		ASensorEventQueue_setEventRate(mpSensorEventQueue, mpGyroSensor, minDelay);

		//LOGI("The minimum delay of type %d allowed between events  is %d microseconds", SENSOR_GYROSCOPE, minDelay);
	}
	if (mpMagSensor)
	{
		ASensorEventQueue_enableSensor(mpSensorEventQueue, mpMagSensor);
		int minDelay = ASensor_getMinDelay(mpMagSensor);
		ASensorEventQueue_setEventRate(mpSensorEventQueue, mpMagSensor, minDelay);
		
		//LOGI("The minimum delay of type %d allowed between events  is %d microseconds", SENSOR_MAGNET, minDelay);
	}
	if (mpRotSensor)
	{
		ASensorEventQueue_enableSensor(mpSensorEventQueue, mpRotSensor);
		int minDelay = ASensor_getMinDelay(mpRotSensor);
		ASensorEventQueue_setEventRate(mpSensorEventQueue, mpRotSensor, minDelay);

		//LOGI("The minimum delay of type %d allowed between events  is %d microseconds", SENSOR_MAGNET, minDelay);
	}

	mbNewMag = false;
	StartThread();
}

//-----------------------------------------------------------------------------
void InertialSensor_Android::Stop()
{
	
	if (mpSensorEventQueue) {
		if (mpAccSensor) 
			ASensorEventQueue_disableSensor(mpSensorEventQueue, mpAccSensor);
		if (mpGyroSensor) 
			ASensorEventQueue_disableSensor(mpSensorEventQueue, mpGyroSensor);
		if (mpMagSensor) 
			ASensorEventQueue_disableSensor(mpSensorEventQueue, mpMagSensor);
		if (mpRotSensor)
			ASensorEventQueue_disableSensor(mpSensorEventQueue, mpRotSensor);
	}

	StopThread();

}

//-----------------------------------------------------------------------------
void InertialSensor_Android::StartThread()
{
	if (mbRun) {
		return;
	}
	mbRun = true;
	mThread = std::thread(&InertialSensor_Android::LooperThread, this);
}

//-----------------------------------------------------------------------------
void InertialSensor_Android::StopThread()
{
	mbRun = false;
	mThread.join();
}

//-----------------------------------------------------------------------------
void InertialSensor_Android::LooperThread()
{
	if (mpSensorEventQueue == NULL)
		return;

	while (mbRun) {
		//ALooper_pollAll(0, NULL, NULL, NULL);
		ASensorEvent event;
		while (ASensorEventQueue_getEvents(mpSensorEventQueue, &event, 1) > 0) {

			switch (event.type) {
				case ASENSOR_TYPE_ACCELEROMETER:
					//LOGI("acc: %lld, %f, %f, %f", (long long)event.timestamp, event.data[0],
					// event.data[1], event.data[2]);

					mTimstamp_acc = (long long) event.timestamp;
					for (int i = 0; i < 3; i++) {
						mAcc[i] = event.data[i];
					}

					if (mTimstamp_acc == mTimstamp_gyro) {
						//OnImuReceived(mAcc, mGyro, mMag, event.timestamp, mbNewMag);
						mbNewMag = false;
					}

					break;

				case ASENSOR_TYPE_GYROSCOPE:
					//LOGI("gyro: %lld, %f, %f, %f", (long long)event.timestamp, event.data[0],
					// event.data[1], event.data[2]);

					mTimstamp_gyro = (long long) event.timestamp;
					for (int i = 0; i < 3; i++) {
						mGyro[i] = event.data[i];
					}

					if (mTimstamp_acc == mTimstamp_gyro) {
						//OnImuReceived(mAcc, mGyro, mMag, event.timestamp, mbNewMag);
						mbNewMag = false;
					}

					break;

				case ASENSOR_TYPE_MAGNETIC_FIELD:
					//LOGI("mag: %lld, %f, %f, %f", (long long)event.timestamp, event.data[0] / 100.0f,
					// event.data[1] / 100.0f, event.data[2] / 100.0f);

					mTimstamp_mag = (long long) event.timestamp;
					for (int i = 0; i < 3; i++) {
						mMag[i] = event.data[i] / 100.0f;
					}

					mbNewMag = true;

					break;

				case ASENSOR_TYPE_ROTATION_VECTOR:
					//LOGI("orientation: %lld, %f, %f, %f", (long long)event.timestamp, event.data[0],
					// event.data[1], event.data[2], event.data[3]);

					for (int i = 0; i < 4; i++) {
						mOrientaion[i] = event.data[i];
					}
					break;
			}

		}

		// sleep
		usleep(1000);
	}
}
