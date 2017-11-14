#pragma once
#include <string>
#include <vector>
#include <opencv2/core/core.hpp>
#include "ImageSource.h"

class ARKitSource : public ImageSource
{
public:
    ARKitSource(const std::string &directory, const std::string& listFile);

private:
    void PathsFromListFile(std::vector<std::string> &imagePaths,
                           std::vector<long long> &timeStamps,
                           std::string directory, std::string file);
};
