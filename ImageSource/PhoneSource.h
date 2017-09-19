#pragma once
#include <string>
#include <vector>
#include <opencv2/core/core.hpp>

class PhoneSource
{
public:
    enum ImageType {LEFT, RIGHT, DEPTH};

    PhoneSource(const std::string& listFile);

    void GrabNewFrame();

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

private:
    void StereoPathsFromListFile(std::vector<std::string> &imagePaths,
                                 std::vector<long long> &timeStamps,
                                 std::string listPath,
                                 ImageType imageType);

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
