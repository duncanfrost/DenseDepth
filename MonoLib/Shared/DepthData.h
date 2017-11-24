#pragma once 
#include <vector>
#include <sophus/se3.hpp>
#include <opencv2/opencv.hpp>

struct DepthData {
    DepthData()
    {
    }

    cv::Mat frame;
};
