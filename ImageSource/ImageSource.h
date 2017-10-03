#pragma once
#include <string>
#include <vector>
#include <opencv2/core/core.hpp>

class ImageSource
{
public:

    ImageSource();

    void GrabNewFrame(bool downsample);

    void SetFrameNumber(unsigned int number)
    {
        frameNumber = number;
    }

    cv::Mat Image()
    {
        return imLeft;
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
    std::vector<std::string> stereoImagePaths;
    std::vector<std::string> depthImagePaths;
    std::vector<std::string> rgbImagePaths;
    std::vector<long long> stereoTimeStamps;
    std::vector<long long> depthTimeStamps;
    std::vector<long long> rgbTimeStamps;

    cv::Mat imLeft;
    cv::Mat imRight;

    long long timeStamp;

    unsigned int frameNumber;
};
