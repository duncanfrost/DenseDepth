#pragma once

#include <vector>
#include <string>
#include <sophus/se3.hpp>

class FileTracker
{
public:
    FileTracker(const std::string &poseFilePath);
    Sophus::SE3f PoseAtTime(long long time, long long &timeOut);

private:
    void InitORB(const std::string &poseFilePath);
    std::vector<std::pair<long long, Sophus::SE3f> > timedPoses;
};
