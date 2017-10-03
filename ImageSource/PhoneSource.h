#pragma once
#include <string>
#include <vector>
#include <opencv2/core/core.hpp>
#include "ImageSource.h"

class PhoneSource : public ImageSource
{
public:
    enum ImageType {LEFT, RIGHT, DEPTH};

    PhoneSource(const std::string& listFile);

private:
    void StereoPathsFromListFile(std::vector<std::string> &imagePaths,
                                 std::vector<long long> &timeStamps,
                                 std::string listPath,
                                 ImageType imageType);
};
