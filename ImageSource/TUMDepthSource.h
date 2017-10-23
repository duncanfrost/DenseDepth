#pragma once
#include <string>
#include <vector>
#include <opencv2/core/core.hpp>
#include "DepthSource.h"

class TUMDepthSource : public DepthSource
{
public:
    TUMDepthSource(const std::string& depthListFile);

private:
    void PathsFromListFile(std::vector<std::string> &imagePaths,
                           std::vector<long long> &timeStamps,
                           std::string listPath);
};
