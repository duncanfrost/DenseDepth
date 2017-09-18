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
    Sophus::SE3f pose = tracker->PoseAtTime(timeStamp, timeOut);

    currTrackerData->trackerPose = pose;
    currTrackerData->frame = image;
}
