#pragma once 
#include <ORUtils/TimeStampedImage.h>
#include "DenseMonoMath.h"
#include <vector>


struct TrackerData {
	TrackerData()
	{
		frame = NULL;
		frame_grey = NULL;
	}

	DenseMono::SE3f trackerPose;
	ORUtils::TimeStampedImage<Vector4u> *frame;
	ORUtils::TimeStampedImage<float> *frame_grey;
	ORUtils::Image<bool> *mask;
};
