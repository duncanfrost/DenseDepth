#pragma once 
#include "Keyframe.h"
#include "../MonoDepthEstimator.h"

struct MapPoint
{
    unsigned char c1;
    unsigned char c2;
    unsigned char c3;

    Eigen::Vector3f position;
};

struct GlobalMap
{
    GlobalMap()
    {
        monoDepthEstimator = NULL;
    }

    std::vector<KeyFrame*> keyframeList;
    std::vector<KeyFrame*> activeFrameList;
    std::vector<MapPoint*> mappoints;

    MonoLib::MonoDepthEstimator *monoDepthEstimator;
};
