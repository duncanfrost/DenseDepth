#pragma once

#include <vector>
#include <string>
#include <sophus/se3.hpp>
#include "FileTracker.h"

class ARKitFileTracker : public FileTracker
{
public:
    ARKitFileTracker(const std::string file);

private:
    void Init(const std::string file);
};
