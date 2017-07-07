#include "stdafx.h"
#include "OutputBuffer.h"

//FILE* pOutLog;

namespace slam {

//-----------------------------------------------------------------------------
OutputBuffer::OutputBuffer()
	: RingBuffer<OutputNode, OUTPUT_EXP>()
{

}

//-----------------------------------------------------------------------------
OutputBuffer& OutputBuffer::GetInstance()
{
	static OutputBuffer instance;
	return instance;
}

//-----------------------------------------------------------------------------
OutputBuffer::~OutputBuffer()
{
	//fclose(pOutLog); 
}

//-----------------------------------------------------------------------------
void OutputBuffer::addNode(double timestamp, Eigen::Vector3d& pos, Eigen::Vector3d& vel, 
	Eigen::Quaterniond& q, int quality)
{
	auto& node = _buffer[head]; 
	node.timestamp = timestamp;
	node.pos = pos;
	node.vel = vel;
	node.q = q; 
	node.quality = quality;

	node.acc.setZero();
	node.gyro.setZero();

	IndexGoAhead();

	// notity
	notifyListeners(node);
}

//-----------------------------------------------------------------------------
void OutputBuffer::addNode(double timestamp, Eigen::Vector3d& pos, Eigen::Vector3d& vel,
	Eigen::Vector3d& acc, Eigen::Vector3d& gyro, Eigen::Quaterniond& q, int quality)
{
	auto& node = _buffer[head];
	node.timestamp = timestamp;
	node.pos = pos;
	node.vel = vel;
	node.q = q;
	node.quality = quality;

	node.acc = acc;
	node.gyro = gyro;

	IndexGoAhead();

	// notity
	notifyListeners(node);
}


//-----------------------------------------------------------------------------
/// Add a listener.  
void OutputBuffer::addListener(Listener* l)
{
	mListeners.push_back(l);
}


//-----------------------------------------------------------------------------
void OutputBuffer::notifyListeners(const OutputNode& node)
{
	for (auto listener : mListeners) {
		listener->onPoseUpdated(node);
	}
}

}
