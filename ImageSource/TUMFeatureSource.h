#pragma once
#include <string>
#include <vector>
#include <opencv2/core/core.hpp>
#include "ImageSource.h"

class TUMFeatureSource : public ImageSource
{
public:
    void GrabNewFrame();
    void GrabNewFrameDebug();

    TUMFeatureSource(const std::string& listFile, int height, int width);

    float *GetData() {return data;}
    void SetData(float* data)
    {
        this->data = data;
    }

private:
    void PathsFromListFile(std::vector<std::string> &imagePaths,
                           std::vector<long long> &timeStamps,
                           std::string listPath);

    int height, width, nChannels;
    float *data;
};
