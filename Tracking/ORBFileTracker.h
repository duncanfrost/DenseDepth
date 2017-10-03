#pragma once

#include <vector>
#include <string>
#include <sophus/se3.hpp>
#include "FileTracker.h"

class ORBFileTracker : public FileTracker
{
public:
    ORBFileTracker(const std::string &poseFilePath);

private:
    void InitORB(const std::string &poseFilePath);
};
