#pragma once
#include <string>
#include <vector>
#include <opencv2/core/core.hpp>
#include "ImageSource.h"

class TUMSource : public ImageSource
{
public:
    enum ImageType {LEFT, RIGHT, DEPTH};

    TUMSource(const std::string& listFile);

private:
    void StereoPathsFromListFile(std::vector<std::string> &imagePaths,
                                 std::vector<long long> &timeStamps,
                                 std::string listPath,
                                 ImageType imageType);
};
