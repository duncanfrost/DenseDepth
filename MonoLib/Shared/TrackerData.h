#pragma once 
#include "DenseMonoMath.h"
#include <vector>
#include <sophus/se3.hpp>
#include <opencv2/opencv.hpp>

struct TrackerData {
    TrackerData()
    {
    }

    Sophus::SE3f trackerPose;
    cv::Mat frame;
};
