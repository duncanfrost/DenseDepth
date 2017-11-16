#pragma once
#include <string>
#include <vector>
#include <opencv2/core/core.hpp>
#include "ImageSource.h"

class TUMFeatureSource : public ImageSource
{
public:
    void GrabNewFrame();

    TUMFeatureSource(const std::string& listFile);

private:
    void PathsFromListFile(std::vector<std::string> &imagePaths,
                           std::vector<long long> &timeStamps,
                           std::string listPath);
};
