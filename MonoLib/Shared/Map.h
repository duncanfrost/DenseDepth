#pragma once 
#include "Keyframe.h"
#include "../MonoDepthEstimator.h"
struct GlobalMap
{
	GlobalMap()
	{
		monoDepthEstimator = NULL;
	}

	std::vector<KeyFrame*> keyframeList;
	std::vector<KeyFrame*> activeFrameList;
	MonoLib::MonoDepthEstimator *monoDepthEstimator;
};
