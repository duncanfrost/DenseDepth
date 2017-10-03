#pragma once
#include <MonoLib/Shared/TrackerData.h>
#include <MonoLib/Shared/Map.h>
#include <opencv2/opencv.hpp>
#include <ImageSource/PhoneSource.h>
#include <Tracking/FileTracker.h>
#include <sophus/se3.hpp>
#include <ORUtils/ImageTypes.h>

class FusionEngine
{
public:
    FusionEngine(PhoneSource* source, FileTracker *tracker);
    void Process();
    void MakePointCloud();


    TrackerData* GetARData()
    {
        return currTrackerData;
    }

    GlobalMap *GetMap()
    {
        return map;
    }

private:
    TrackerData* currTrackerData;
    FileTracker* tracker;
    PhoneSource* source;

    cv::Mat image;
    cv::Size imgSize;

    int framesProcessed;

    long long timeStamp;
    Sophus::SE3f currPose;
    GlobalMap *map;

    float fxInv;
    float fyInv;
    float cxInv;
    float cyInv;

};
