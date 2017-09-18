#include "MonoEngine.h"

MonoEngine::MonoEngine(PhoneSource* source)
{
    this->source = source;
    currTrackerData = new TrackerData();
}


void MonoEngine::Process()
{
    source->GrabNewFrame();
    image = source->Image();
    currTrackerData->frame = image;
}
