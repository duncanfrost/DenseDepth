#pragma once
#include <string>
#include <vector>
#include <opencv2/core/core.hpp>

class DepthSource
{
public:

    DepthSource();


    void GetDepthForTimeStamp(long long timestamp);
    void SetFrameNumber(unsigned int number)
    {
        frameNumber = number;
    }

    cv::Mat Image()
    {
        return im;
    }

    long long TimeStamp()
    {
        return timeStamp;
    }

    long long FrameNumber()
    {
        return frameNumber;
    }

protected:
    std::vector<std::string> depthImagePaths;
    std::vector<long long> depthTimeStamps;

    cv::Mat im;

    long long timeStamp;

    unsigned int frameNumber;
};
