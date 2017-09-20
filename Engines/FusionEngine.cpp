#include "FusionEngine.h"
#include <ORUtils/MathTypes.h>

FusionEngine::FusionEngine(PhoneSource* source, FileTracker* tracker)
{
    this->source = source;
    this->tracker = tracker;
    currTrackerData = new TrackerData();
    map = new GlobalMap();

    imgSize.x = 640;
    imgSize.y = 480;

    imgSize.x = 160;
    imgSize.y = 120;

    Vector4f intrinsics;
    float fx = (683.8249/640)*imgSize.x;
    float fy = (683.6179/480)*imgSize.y;
    float cx = (317.6438/640)*imgSize.x;
    float cy = (239.5907/480)*imgSize.y;

    intrinsics[0] = fx;
    intrinsics[1] = fy;
    intrinsics[2] = cx;
    intrinsics[3] = cy;
}


void FusionEngine::Process()
{
    source->GrabNewFrame();
    image = source->Image();
    timeStamp = source->TimeStamp();

    long long count = source->FrameNumber();

    std::cout << "Timestamp: " << timeStamp << std::endl;

    long long timeOut;
    currPose = tracker->PoseAtTime(count, timeOut);

    currTrackerData->trackerPose = currPose;
    currTrackerData->frame = image;

    framesProcessed++;
}
