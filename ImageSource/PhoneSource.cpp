#include "PhoneSource.h"
#include <iostream>
#include <sstream>
#include <fstream>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

PhoneSource::PhoneSource(const std::string& listFile)
{
    StereoPathsFromListFile(rgbImagePaths, rgbTimeStamps, listFile, LEFT);
    StereoPathsFromListFile(stereoImagePaths, stereoTimeStamps, listFile, RIGHT);

    frameNumber = 0;
}

void PhoneSource::GrabNewFrame(bool downsample)
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

void PhoneSource::StereoPathsFromListFile(std::vector<std::string> &imagePaths,
                                          std::vector<long long> &timeStamps,
                                          std::string listPath,
                                          ImageType imageType)
{
    imagePaths.clear();
    std::ifstream listFile(listPath.c_str());
    if (!listFile)
    {
        std::cout <<  "error: rgb image list file doesn't exist" << std::endl;
        exit(1);
    }
    std::string line;

    //This is the directory that the list file is located in.
    std::string listDir = listPath.substr(0,listPath.find_last_of("/")+1);
    std::cout << "listDir" << std::endl;
    std::cout << listDir << std::endl;

    while (std::getline(listFile, line))
    {
        //Hashtags are considered comments
        if (line[0] == '#')
            continue;

        long long timestamp;
        std::string path;

        std::istringstream iss(line);

        //Full paths are the root directory joined with whatever is in the listfile
        if (iss >> timestamp)
        {
            std::string folder;
            switch(imageType)
            {
            case LEFT:
                folder = "cam1/";
                break;
            case RIGHT:
                folder = "cam0/";
                break;
            case DEPTH:
                folder = "estdepth/";
                break;
            }

            std::stringstream fullPath;
            fullPath << listDir << folder << timestamp << "000000.bmp";

            imagePaths.push_back(fullPath.str());
            timeStamps.push_back(timestamp);
        }
    }
}
