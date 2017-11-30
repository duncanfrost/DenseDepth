#pragma once
#include <MonoLib/Shared/TrackerData.h>
#include <MonoLib/Shared/Map.h>
#include <opencv2/opencv.hpp>
#include <ImageSource/ImageSource.h>
#include <ImageSource/TUMFeatureSource.h>
#include <ImageSource/DepthSource.h>
#include <Tracking/FileTracker.h>
#include <sophus/se3.hpp>
#include <ORUtils/ImageTypes.h>
#include <MonoLib/MonoDepthEstimator_CUDA.h>
#include <rmd/depthmap.h>
#include <fstream>

#define BUFFERSIZE 150

class MonoEngine
{
public:

    struct Settings
    {

        Settings()
        {
            checkTimeDiff = false;
        }

        float fx;
        float fy;
        float cx;
        float cy;

        int inputSizeX;
        int inputSizeY;
        int targetSizeX;
        int targetSizeY;

        bool checkTimeDiff;
    };

    MonoEngine(ImageSource* source, DepthSource* depthSource,
               TUMFeatureSource* featureSource,
               FileTracker *tracker, Settings settings);

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

    TrackerData* GetDepthData()
    {
        return currDepthData;
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

    void DoSmoothProcess();
    void SmoothPhoto();
    void SmoothPhotoActive();
    void SmoothPhotoBuffer();
    void SmoothPhotoRemode();

    void GetPointCloud(unsigned int &width,
                       unsigned int &height, Vector3f **points,
                       Vector4u **colorData);
    void SampleFromBufferMid();
    void SampleFromBufferMid_Remode();
    void WritePhotoErrors(std::string path);
    void Sample();

    void MeasureDepthError();

    void ProcessDepthData();

    MonoLib::MonoDepthEstimator *monoDepthEstimator;

    void ProcessKeyFrame();
    void ProcessBuffer();

private:

    std::ofstream depthErrorFile;

    void Init();

    void MakePointCloud();

    void SaveToBuffer();


    void LoadGTDepth(long long timestamp);
    void VisualizeDepth();

    void WriteEmpty();

    cv::Mat PreProcessImage(cv::Mat image);



    TrackerData* currTrackerData;
    TrackerData* currDepthData;
    FileTracker* tracker;
    ImageSource* source;
    TUMFeatureSource* featureSource;
    DepthSource* depthSource;
    Settings settings;


    cv::Mat currImage;
    Vector2i imgSize;
    ORUChar4TSImage *orImage;

    ORUtils::MemoryBlock<float> *featureImage;
    int featureChannels;
    int featureHeight;
    int featureWidth;

    int bufferTop;
    int framesProcessed;
    int nMid;

    long long timeStamp;
    long long timeStampRef;
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

    //Active processing stuff
    bool processActive;
    float thetaEnd;
    float beta;
    float theta;

    int processCount;

    //Buffer for storing whether we consider points good or not 
    bool *goodPoint;






};
