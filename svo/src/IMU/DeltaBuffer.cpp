#include "stdafx.h"
#include "DeltaBuffer.h"

namespace slam {

//-----------------------------------------------------------------------------
DeltaBuffer::DeltaBuffer()
	: RingBuffer<DeltaPose, DELTA_EXP>()
{

}

//-----------------------------------------------------------------------------
DeltaBuffer::~DeltaBuffer()
{
}

//-----------------------------------------------------------------------------
void DeltaBuffer::addNode(const DeltaPose& dPose)
{
	auto& node = _buffer[head]; 
	node = dPose;

	IndexGoAhead();

}

//-----------------------------------------------------------------------------
int DeltaBuffer::getDeltaPoseByTimestamp(DeltaPose& dPose, double start_time, double end_time)
{
	return -1;
}

}
