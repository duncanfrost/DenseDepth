#include "DepthSource.h"
#include <iostream>
#include <sstream>
#include <fstream>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

DepthSource::DepthSource()
{
    frameNumber = 0;
}

void DepthSource::GetDepthForTimeStamp(long long timestamp)
{
    long long current = 0;
    long long time2 = depthTimeStamps[current+1];

    while (time2 <= timestamp)
    {
        current++;
        time2 = depthTimeStamps[current+1];
    }

    std::cout << "Timestamp: " << timestamp << std::endl;
    std::cout << "Depth timestamp: " << time2 << std::endl;

    // std::string filename = depthImagePaths[current];
    // std::string fullPath = directory + "/" + filename;
    // cv::Mat depth = cv::imread(fullPath,CV_LOAD_IMAGE_ANYDEPTH);
    // cv::Mat fDepth;
    // depth.convertTo(fDepth,CV_32FC1);
    // fDepth /= 5000;

    // return fDepth;

}

