/*************************************************************************
* Copyright (c) 2013-2014 Impression.Pi, Inc.
*
* All rights reserved. No warranty, explicit or implicit, provided.
*/

#pragma once


#include <thread>
#include <android/sensor.h>

#include "InertialSensor.h"

// Somehow this got missed in the NDK
#define ASENSOR_TYPE_ROTATION_VECTOR 11


class InertialSensor_Android : public dso::InertialSensor
{
public:
	InertialSensor_Android(const std::string& configFile);
	~InertialSensor_Android();

private:
	ASensorManager* mpSensorManager;
	ASensorEventQueue* mpSensorEventQueue;
	ALooper* mpLooper;

	const ASensor* mpAccSensor;
	const ASensor* mpGyroSensor;
	const ASensor* mpMagSensor;
	const ASensor* mpRotSensor;

	float mAcc[3];
	float mGyro[3];
	float mMag[3];
	float mOrientaion[4];

	long long mTimstamp_acc;
	long long mTimstamp_gyro;
	long long mTimstamp_mag;
	bool mbNewMag;

public:
	/// open device
	bool Initialize();
	/// setup and start imu streaming
	void Start();
	/// stop imu streaming
	void Stop();
	/// close device
	void Uninitialize();

private:
	std::thread mThread;
	bool mbRun;

	void StartThread();
	void StopThread();
	void LooperThread();
};

