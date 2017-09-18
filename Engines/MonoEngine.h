#pragma once
#include <MonoLib/Shared/TrackerData.h>

class MonoEngine
{
public:
    MonoEngine();
    TrackerData* GetARData()
    {
        return currTrackerData;
    }


private:
    TrackerData* currTrackerData;

};
