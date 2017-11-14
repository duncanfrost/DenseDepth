#include "ImageSource.h"
#include <iostream>
#include <sstream>
#include <fstream>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

ImageSource::ImageSource()
{
    frameNumber = 0;
}

void ImageSource::GrabNewFrame()
{
    std::string path = rgbImagePaths[frameNumber];
    timeStamp = rgbTimeStamps[frameNumber];
    cv::Mat imTemp = cv::imread(path);
    imLeft = imTemp;

    if (frameNumber < rgbImagePaths.size()-1)
        frameNumber++;
}

