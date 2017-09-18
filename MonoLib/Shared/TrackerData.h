#pragma once 
#include "DenseMonoMath.h"
#include <vector>
#include <sophus/se3.hpp>

struct TrackerData {
    TrackerData()
    {
    }

    Sophus::SE3f trackerPose;
};
