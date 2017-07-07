#include "stdafx.h"

#include "InertialSensor_ThreeSpace.h"
#include "IMU/Utils/TimeUtils.h"

namespace slam {

#if USE_IMU_YEI

#define g 9.81f		// gravity, m/s^2
#define USE_YEI_QUATERNION			1

std::vector<InertialSensor_ThreeSpace*> InertialSensor_ThreeSpace::mSensorArray;
std::mutex InertialSensor_ThreeSpace::mSensorMutex;

//-----------------------------------------------------------------------------
InertialSensor_ThreeSpace::InertialSensor_ThreeSpace(const std::string& configFile)
	: InertialSensor(configFile), mDeviceID(TSS_NO_DEVICE_ID), mIsStreaming(false)
{
	Config();
}


//-----------------------------------------------------------------------------
InertialSensor_ThreeSpace::~InertialSensor_ThreeSpace()
{
	if (mDeviceID != TSS_NO_DEVICE_ID) {
		Stop();

		tss_setNewDataCallBack(mDeviceID, 0);
		tss_closeTSDevice(mDeviceID);

		// todo: lock
		//mSensorMutex.lock();
		for (auto it = mSensorArray.begin(); it != mSensorArray.end(); it++) {
			if ((*it)->mDeviceID == mDeviceID) {
				mSensorArray.erase(it);
				break;
			}
		}
		//mSensorMutex.unlock();
	}

}

//-----------------------------------------------------------------------------
void InertialSensor_ThreeSpace::AlignTimestamp()
{
	// align imu and pc timestamp
	const int N = 1;
	long long tSum = 0;
	unsigned char mode;
	unsigned int timestamp;
	for (int i = 0; i < N; i++) {
		auto start_time = getCppTimestamp_us();
		tss_getLEDMode(mDeviceID, &mode, &timestamp);
		auto end_time = getCppTimestamp_us();
		tSum += (start_time + end_time) / 2 - timestamp;
	}
	mDeltaTimestamp = tSum / N;
}

//-----------------------------------------------------------------------------
bool InertialSensor_ThreeSpace::Config()
{
	if (mComPort != "") {
		mDeviceID = tss_createTSDeviceStr(mComPort.c_str(), TSS_TIMESTAMP_CPP);
		if (mDeviceID == TSS_NO_DEVICE_ID) {
			printf("Failed to create a sensor on %s\n", mComPort.c_str());
			return false;
		}
	}
	else {
		TSS_ComPort comport;
		if (tss_getComPorts(&comport, 1, 0, TSS_FIND_ALL_KNOWN ^ TSS_FIND_DNG)){
			mDeviceID = tss_createTSDeviceStr(comport.com_port, TSS_TIMESTAMP_CPP);
			if (mDeviceID == TSS_NO_DEVICE_ID) {
				printf("Failed to create a sensor on %s\n", comport.com_port);
				return false;
			}
		}
		else{
			printf("No sensors found\n");
			return false;
		}
	}

	// add to mSensorArray
	mSensorMutex.lock();
	mSensorArray.push_back(this);
	//printf("mSensorArray size: %d\n", mSensorArray.size()); 
	mSensorMutex.unlock(); 

	// align imu and pc timestamp
	AlignTimestamp();

	// set imu range
	tss_setAccelerometerRange(mDeviceID, 0, NULL);	// 2g
	tss_setGyroscopeRange(mDeviceID, 2, NULL);		// 2000 dps
	tss_setCompassRange(mDeviceID, 2, NULL);			// 1.9G

	// set axis direction
	/**< X: Right, Y: Forward, Z: Up (right-handed system) */
	unsigned char ad = tss_generateAxisDirections(TSS_XZY, 0, 0, 0); 
	tss_setAxisDirections(mDeviceID, ad, 0);

	// set callback function
	tss_setNewDataCallBack(mDeviceID, TSS_CallBackFunc);

#if USE_YEI_QUATERNION
	// set filter mode: kalman filter mode
	tss_setFilterMode(mDeviceID, 1, NULL);

	// set stream slots
	TSS_Stream_Command stream_slots[8] = { TSS_GET_ALL_RAW_COMPONENT_SENSOR_DATA, 
		TSS_GET_UNTARED_ORIENTATION_AS_QUATERNION, TSS_NULL, TSS_NULL, TSS_NULL, TSS_NULL, TSS_NULL, TSS_NULL };
	tss_setStreamingSlots(mDeviceID, stream_slots, NULL);
#else
	// set filter mode: imu mode
	tss_setFilterMode(mDeviceID, 0, NULL);

	// set stream slots
	TSS_Stream_Command stream_slots[8] = { TSS_GET_ALL_RAW_COMPONENT_SENSOR_DATA,
		TSS_NULL, TSS_NULL, TSS_NULL, TSS_NULL, TSS_NULL, TSS_NULL, TSS_NULL };
	tss_setStreamingSlots(mDeviceID, stream_slots, NULL);
#endif

	// set sample frequency and interval (us)
	mSampleInterval = 1000000 / mSampleFreq;	// us

	return true;
}

//-----------------------------------------------------------------------------
void InertialSensor_ThreeSpace::Start()
{
	if (mDeviceID == TSS_NO_DEVICE_ID)
		return;

	// imu sampling interval (us)
	tss_setStreamingTiming(mDeviceID, mSampleInterval, TSS_INFINITE_DURATION, 0, NULL);

	// start stream
	tss_startStreaming(mDeviceID, NULL);
	mIsStreaming = true;
}

//-----------------------------------------------------------------------------
void InertialSensor_ThreeSpace::Stop()
{
	if (mDeviceID == TSS_NO_DEVICE_ID)
		return;
	if (mIsStreaming) {
		tss_stopStreaming(mDeviceID, NULL);
		mIsStreaming = false;
	}
}

//-----------------------------------------------------------------------------
void CALLBACK InertialSensor_ThreeSpace::TSS_CallBackFunc(TSS_Device_Id device, char* output_data,
	unsigned int output_data_len, unsigned int* _timestamp)
{
	//long long timestamp = (*_timestamp + mDeltaTimestamp) * 1000;
	//fprintf(pImuFile, "Now timestamp = %I64d\nIMU timestamp = %I64d\n", getCppTimestamp(), timestamp);

	float* data = (float*)output_data;
	data[3] *= g;
	data[4] *= g;
	data[5] *= g;

	int data_len = output_data_len / sizeof(float);

	InertialSensor_ThreeSpace* instance = nullptr;
	mSensorMutex.lock();
	for (auto it = mSensorArray.begin(); it != mSensorArray.end(); it++) {
		if ((*it)->mDeviceID == device) {
			instance = *it;
			break;
		}
	}
	mSensorMutex.unlock();
	if (instance != nullptr) {
		long long timestamp = (*_timestamp + instance->mDeltaTimestamp) * 1000;
		//printf("timestamp = %lld\n", timestamp); 
		float* gyro = data;
		float* acc = data + 3;
		float* mag = data + 6;
#if USE_YEI_QUATERNION
		float* quat = data + 9;
#else
		float* quat = nullptr;
#endif
		instance->OnImuReceived(gyro, acc, mag, quat, TimestampToSecond(timestamp));
	}
		
}

#endif

}