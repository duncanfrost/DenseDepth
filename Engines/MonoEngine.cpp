#include "MonoEngine.h"

MonoEngine::MonoEngine(PhoneSource* source, FileTracker* tracker)
{
    this->source = source;
    this->tracker = tracker;
    currTrackerData = new TrackerData();
    map = new GlobalMap();
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
