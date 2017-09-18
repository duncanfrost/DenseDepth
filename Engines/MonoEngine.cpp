#include "MonoEngine.h"
#include <ORUtils/MathTypes.h>

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
    

    monoDepthEstimator = MonoLib::MonoDepthEstimatorFactory::MakeMonoDepthEstimator(
        imgSize, intrinsics, "CUDA");
}


void MonoEngine::Process()
{
    source->GrabNewFrame();
    image = source->Image();
    timeStamp = source->TimeStamp();

    long long timeOut;
    currPose = tracker->PoseAtTime(timeStamp, timeOut);

    currTrackerData->trackerPose = currPose;
    currTrackerData->frame = image;
}


void MonoEngine::AddKeyFrame()
{
    std::cout << "About to make new keyframe" << std::endl;

    KeyFrame *kf = new KeyFrame();
    kf->pose = currPose;
    map->keyframeList.push_back(kf);


    // monoDepthEstimator->SetRefImage(rgbImage);
    // invRefPose = kf->pose.inverse();

    // fusionState->pose_d->SetFrom(&kf->pose);

    // view->depth->UpdateHostFromDevice();

    // monoDepthEstimator->SetLimitsFromGroundTruth(view->depth->GetData(MEMORYDEVICE_CPU),
    //                                              view->depth->noDims);

    // // monoDepthEstimator->SetLimitsManual(0.5,2);

    // hasReferenceFrame = true;

    // needsKeyFrame = false;
    // std::cout << "Keyframes: " << map->keyframeList.size() << std::endl;
}
