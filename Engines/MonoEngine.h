#pragma once
#include <MonoLib/Shared/TrackerData.h>
#include <opencv2/opencv.hpp>
#include <ImageSource/PhoneSource.h>

class MonoEngine
{
public:
    MonoEngine(PhoneSource* source);

    void Process();

    TrackerData* GetARData()
    {
        return currTrackerData;
    }


private:
    TrackerData* currTrackerData;
    PhoneSource* source;
    cv::Mat image;


};
