#include "ImageSource.h"
#include <iostream>
#include <sstream>
#include <fstream>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

ImageSource::ImageSource(const std::string& listFile)
{
    frameNumber = 0;
}

void ImageSource::GrabNewFrame(bool downsample)
{
    std::string path =  rgbImagePaths[frameNumber];
    timeStamp = rgbTimeStamps[frameNumber];
    cv::Mat imTemp = cv::imread(path);

    cv::Size size;
    size.width = 3;
    size.height = 3;
    

    if (downsample)
    {
        cv::resize(imTemp, imLeft, cv::Size(), 0.25f, 0.25f);
        cv::GaussianBlur(imLeft, imLeft, size, 3);
    }
    else
        imLeft = imTemp;

    frameNumber++;
}

