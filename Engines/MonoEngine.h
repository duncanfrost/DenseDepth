#pragma once
#include <MonoLib/Shared/TrackerData.h>
#include <MonoLib/Shared/Map.h>
#include <opencv2/opencv.hpp>
#include <ImageSource/ImageSource.h>
#include <ImageSource/DepthSource.h>
#include <Tracking/FileTracker.h>
#include <sophus/se3.hpp>
#include <ORUtils/ImageTypes.h>
#include <MonoLib/MonoDepthEstimator_CUDA.h>
#include <rmd/depthmap.h>

#define BUFFERSIZE 150

class MonoEngine
{
public:

    struct Settings
    {
        float fx;
        float fy;
        float cx;
        float cy;

        int inputSizeX;
        int inputSizeY;
    };

    MonoEngine(ImageSource* source, DepthSource* depthSource,
               FileTracker *tracker, Settings settings);

    void AddKeyFrameManual()
    {
        needsKeyFrame = true;
    }
    
    void AddKeyFrame(cv::Mat inImage, Sophus::SE3f inPose);
    void AddKeyFrame_Remode(cv::Mat inImage, Sophus::SE3f inPose);

    void Process();

    TrackerData* GetARData()
    {
        return currTrackerData;
    }

    GlobalMap *GetMap()
    {
        return map;
    }

    void TogglePaused()
    {
        paused = !paused;
    }

    static Sophus::SE3f invRefPose;
    void SmoothPhoto(int iterations);
    void SmoothPhotoBuffer(int iterations);
    void SmoothPhotoRemode(int iterations);

    void GetPointCloud(unsigned int &width,
                       unsigned int &height, Vector3f **points,
                       Vector4u **colorData);
    void SampleFromBufferMid();
    void SampleFromBufferMid_Remode();
    void WritePhotoErrors(std::string path);
    void Sample();
    MonoLib::MonoDepthEstimator *monoDepthEstimator;

private:

    void MakePointCloud(bool useRawDepth);

    void SaveToBuffer();


    void LoadGTDepth(long long timestamp);
    void VisualizeDepth();

    void WriteEmpty();

    cv::Mat PreProcessImage(cv::Mat image);



    TrackerData* currTrackerData;
    FileTracker* tracker;
    ImageSource* source;
    DepthSource* depthSource;


    cv::Mat currImage;
    Vector2i imgSize;
    ORUChar4TSImage *orImage;

    int bufferTop;
    int framesProcessed;
    int nMid;

    long long timeStamp;
    Sophus::SE3f currPose;
    GlobalMap *map;
    rmd::Depthmap *depthMap;

    bool hasReferenceFrame;
    bool useRawDepth;
    bool needsKeyFrame;

    cv::Mat imageBuffer[BUFFERSIZE];
    Sophus::SE3f poseBuffer[BUFFERSIZE];
    long long timeStampBuffer[BUFFERSIZE];


    bool paused;



};
