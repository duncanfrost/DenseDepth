#pragma once
#include <MonoLib/Shared/TrackerData.h>
#include <MonoLib/Shared/Map.h>
#include <opencv2/opencv.hpp>
#include <ImageSource/PhoneSource.h>
#include "FileTracker.h"
#include <sophus/se3.hpp>
#include <ORUtils/ImageTypes.h>
#include <MonoLib/MonoDepthEstimator_CUDA.h>

#define BUFFERSIZE 300

class MonoEngine
{
public:
    MonoEngine(PhoneSource* source, FileTracker *tracker);

    void AddKeyFrameManual()
    {
        needsKeyFrame = true;
    }
    
    void AddKeyFrame(ORUChar4TSImage *inImage, Sophus::SE3f inPose);

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
    void SmoothPhotoBuffer(int iterations);

    void GetPointCloud(unsigned int &width,
                       unsigned int &height, Vector3f **points,
                       Vector4u **colorData, bool **goodData);
    void SampleFromBufferMid();

private:

    void MakePointCloud(bool useRawDepth);

    void SaveToBuffer(ORUChar4TSImage *inputRGBImage,
                      Sophus::SE3f inputPose);



    void ConvertToOR(cv::Mat inImage, ORUChar4TSImage *outImage);

    void WriteEmpty();


    TrackerData* currTrackerData;
    FileTracker* tracker;
    PhoneSource* source;
    cv::Mat image;
    Vector2i imgSize;
    ORUChar4TSImage *orImage;

    int bufferTop;
    int framesProcessed;
    int nMid;

    long long timeStamp;
    Sophus::SE3f currPose;
    GlobalMap *map;
    MonoLib::MonoDepthEstimator *monoDepthEstimator;

    bool hasReferenceFrame;
    bool useRawDepth;
    bool needsKeyFrame;

    ORUChar4TSImage *imageBuffer[BUFFERSIZE];
    Sophus::SE3f poseBuffer[BUFFERSIZE];
    unsigned int timeStampBuffer[BUFFERSIZE];


    void Sample();



};