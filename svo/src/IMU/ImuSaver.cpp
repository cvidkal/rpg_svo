#include "stdafx.h"

#include "ImuSaver.h"
#include "IMU/Utils/TimeUtils.h"

namespace slam {

//-----------------------------------------------------------------------------
ImuSaver::ImuSaver(const std::string path, bool bSaveMag)
{
	mbSaveMag = bSaveMag; 

	// open file
	mpFile = fopen(path.c_str(), "w");
	if (mpFile != NULL) {
		if (mbSaveMag)
			fprintf(mpFile, "#timestamp,gx,gy,gz,ax,ay,az,mx,my,mz,qw,qx,qy,qz\n");
		else
			fprintf(mpFile, "#timestamp,omega_x,omega_y,omega_z,alpha_x,alpha_y,alpha_z\n");
	}
	else {
		printf("Open File Failed: %s\n", path.c_str());
	}

}


//-----------------------------------------------------------------------------
ImuSaver::~ImuSaver()
{
	if (mpFile != NULL) {
		fclose(mpFile); 
	}
}

//-----------------------------------------------------------------------------
void ImuSaver::onInertialSensorData(
	float gx, float gy, float gz,
	float ax, float ay, float az,
	float mx, float my, float mz,
	float qw, float qx, float qy, float qz, 
	double timestamp)
{
	if (mpFile == NULL)
		return;

	if (mbSaveMag) {
		fprintf(mpFile, "%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f\n",
			timestamp, gx, gy, gz, ax, ay, az, mx, my, mz, qw, qx, qy, qz);
	}
	else {
		fprintf(mpFile, "%lld,%f,%f,%f,%f,%f,%f\n",
			(long long)(timestamp * 1e9), gx, gy, gz, ax, ay, az);
	}
	
}

}
