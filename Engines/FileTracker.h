#pragma once

#include <vector>
#include <string>
#include <sophus/se3.hpp>

class FileTracker
{
public:
    enum FileType {TUM, ORB};
    FileTracker(const std::string &poseFilePath, FileType type = TUM);
    Sophus::SE3f PoseAtTime(double time, double &timeOut);

private:
    void InitTUM(const std::string &poseFilePath);
    void InitORB(const std::string &poseFilePath);
    std::vector<std::pair<long long, Sophus::SE3f> > timedPoses;
};
