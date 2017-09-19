#pragma once
#include <MonoLib/Shared/TrackerData.h>
#include <MonoLib/Shared/Map.h>
#include <opencv2/opencv.hpp>
#include <ImageSource/PhoneSource.h>
#include "FileTracker.h"
#include <sophus/se3.hpp>
#include <ORUtils/ImageTypes.h>
#include <MonoLib/MonoDepthEstimator_CUDA.h>

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
    void SmoothPhoto(int iterations);

private:


    void ConvertToOR();


    TrackerData* currTrackerData;
    FileTracker* tracker;
    PhoneSource* source;
    cv::Mat image;
    ORUChar4TSImage *orImage;

    long long timeStamp;
    Sophus::SE3f currPose;
    GlobalMap *map;
    MonoLib::MonoDepthEstimator *monoDepthEstimator;

    bool hasReferenceFrame;

    void Sample();



};
