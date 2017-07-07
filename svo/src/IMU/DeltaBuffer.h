#pragma once

#include "IMU/Utils/RingBuffer.h"
#include "IMU/Utils/TypeDefs.h"


#define DELTA_EXP	7	// default buffer size: 2 ^ DELTA_EXP

namespace slam {

class DeltaBuffer 
	: public RingBuffer<DeltaPose, DELTA_EXP>
{
public:
	DeltaBuffer();
	~DeltaBuffer();

public:
	void addNode(const DeltaPose& dPose);

	int getDeltaPoseByTimestamp(DeltaPose& dPose, double start_time, double end_time);

};


}
