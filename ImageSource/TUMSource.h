#pragma once
#include <string>
#include <vector>
#include <opencv2/core/core.hpp>
#include "ImageSource.h"

class TUMSource : public ImageSource
{
public:
    TUMSource(const std::string& listFile, const std::string& depthListFile);

private:
    void PathsFromListFile(std::vector<std::string> &imagePaths,
                           std::vector<long long> &timeStamps,
                           std::string listPath);
};
