#pragma once
#include <MonoLib/Shared/TrackerData.h>
#include <MonoLib/Shared/Map.h>
#include <opencv2/opencv.hpp>
#include <ImageSource/PhoneSource.h>
#include "FileTracker.h"
#include <sophus/se3.hpp>
#include <MonoLib/MonoDepthEstimatorFactory.h>

class MonoEngine
{
public:
    MonoEngine(PhoneSource* source, FileTracker *tracker);
    void AddKeyFrame();

    void Process();

    TrackerData* GetARData()
    {
        return currTrackerData;
    }

    GlobalMap *GetMap()
    {
        return map;
    }

    static Sophus::SE3f invRefPose;

private:
    TrackerData* currTrackerData;
    FileTracker* tracker;
    PhoneSource* source;
    cv::Mat image;
    long long timeStamp;
    Sophus::SE3f currPose;
    GlobalMap *map;
    MonoLib::MonoDepthEstimator *monoDepthEstimator;
};
