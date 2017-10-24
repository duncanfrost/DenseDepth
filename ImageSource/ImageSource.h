#pragma once
#include <string>
#include <vector>
#include <opencv2/core/core.hpp>

class ImageSource
{
public:

    ImageSource();

    void GrabNewFrame();

    void SetFrameNumber(unsigned int number)
    {
        frameNumber = number;
    }

    cv::Mat Image()
    {
        return imLeft;
    }

    cv::Mat RightImage()
    {
        return imRight;
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
    std::vector<std::string> rgbImagePaths;
    std::vector<long long> stereoTimeStamps;
    std::vector<long long> rgbTimeStamps;

    cv::Mat imLeft;
    cv::Mat imRight;

    long long timeStamp;

    unsigned int frameNumber;
};
