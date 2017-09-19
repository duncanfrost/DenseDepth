#include "MonoEngine.h"
#include <ORUtils/MathTypes.h>

Sophus::SE3f MonoEngine::invRefPose;

inline ORUtils::SE3Pose SophusToOR(Sophus::SE3f pose)
{
    ORUtils::SE3Pose out;

    Matrix4f mat;
    
    for (unsigned int r = 0; r < 4; r++)
        for (unsigned int c = 0; c < 4; c++)
        {
            mat.at(c,r) = pose.matrix()(r,c);
        }

    out.SetM(mat);
    
    return out;
}

MonoEngine::MonoEngine(PhoneSource* source, FileTracker* tracker)
{
    this->source = source;
    this->tracker = tracker;
    currTrackerData = new TrackerData();
    map = new GlobalMap();

    Vector2i imgSize(640, 480);
    Vector4f intrinsics;


    float fx = 683.8249;
    float fy = 683.6179;
    float cx = 317.6438;
    float cy = 239.5907;


    intrinsics[0] = fx;
    intrinsics[1] = fy;
    intrinsics[2] = cx;
    intrinsics[3] = cy;
    
    monoDepthEstimator = new MonoLib::MonoDepthEstimator_CUDA(imgSize, intrinsics);

    orImage = new ORUChar4TSImage(imgSize, true, true, true);

    hasReferenceFrame = false;
}


void MonoEngine::Process()
{
    source->GrabNewFrame();
    image = source->Image();
    timeStamp = source->TimeStamp();

    long long timeOut;
    currPose = tracker->PoseAtTime(timeStamp, timeOut);

    ConvertToOR();
    Sample();


    currTrackerData->trackerPose = currPose;
    currTrackerData->frame = image;
}


void MonoEngine::AddKeyFrame()
{
    std::cout << "About to make new keyframe" << std::endl;

    KeyFrame *kf = new KeyFrame();
    kf->pose = currPose;
    map->keyframeList.push_back(kf);

    monoDepthEstimator->SetRefImage(orImage);
    invRefPose = kf->pose.inverse();

    monoDepthEstimator->SetLimitsManual(0.5,2);
    hasReferenceFrame = true;
}

void MonoEngine::ConvertToOR()
{
    for (int y = 0; y < image.rows; y++)
    {
        for (int x = 0; x < image.cols; x++)
        {
            cv::Vec3b val = image.at<cv::Vec3b>(y,x);
            Vector4u orVal;
            orVal[0] = val[0];
            orVal[1] = val[1];
            orVal[2] = val[2];
            orVal[3] = 0;

            int index = x + image.cols*y;
            orImage->GetData(MEMORYDEVICE_CPU)[index] = orVal;
        }
    }

    orImage->UpdateDeviceFromHost();
}

void MonoEngine::Sample()
{
    if (!hasReferenceFrame)
        return;


    std::cout << "Sampling" << std::endl;
    Sophus::SE3f inPose = currPose*invRefPose;
    monoDepthEstimator->UpdatePhotoError(SophusToOR(inPose), orImage);
}



