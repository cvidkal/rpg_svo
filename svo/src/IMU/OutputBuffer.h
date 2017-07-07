#pragma once

#include <list>
#include <Eigen/Dense>
#include "IMU/Utils/RingBuffer.h"
#include "IMU/Utils/TypeDefs.h"


#define OUTPUT_EXP	7	// default buffer size: 2 ^ OUTPUT_EXP

namespace slam {

struct OutputNode
{
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

	// timestamp. unit is second
	double timestamp;
	Eigen::Vector3d pos;
	Eigen::Vector3d vel;
	Eigen::Quaterniond q;
	int quality;

	Eigen::Vector3d acc;
	Eigen::Vector3d gyro;
};



class OutputBuffer 
	: public RingBuffer<OutputNode, OUTPUT_EXP>
{
public:
	class Listener {
	public:
		virtual void onPoseUpdated(const OutputNode& node) = 0;
	};

	/// Add a listener.  
	void addListener(Listener* l);

private:
	std::list<Listener*> mListeners;

	// notify listeners
	void notifyListeners(const OutputNode& node);

private:
	OutputBuffer();

public:
	static OutputBuffer& GetInstance();
	~OutputBuffer();

public:
	void addNode(double timestamp, Eigen::Vector3d& pos, Eigen::Vector3d& vel, Eigen::Quaterniond& q, int quality);
	void addNode(double timestamp, Eigen::Vector3d& pos, Eigen::Vector3d& vel, 
		Eigen::Vector3d& acc, Eigen::Vector3d& gyro, Eigen::Quaterniond& q, int quality);

};


}
