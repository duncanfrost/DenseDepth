#pragma once

#include <vector>
#include <string>
#include <sophus/se3.hpp>

class FileTracker
{
public:
    Sophus::SE3f PoseAtTime(long long time,
                            long long frameCount,
                            long long &timeOut);

protected:
    std::vector<std::pair<long long, Sophus::SE3f> > timedPoses;
    bool useFrameCount;
};
