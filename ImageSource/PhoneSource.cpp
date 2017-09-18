#include "PhoneSource.h"
#include <iostream>
#include <sstream>
#include <fstream>
#include <opencv2/highgui/highgui.hpp>

PhoneSource::PhoneSource(const std::string& listFile)
{
    StereoPathsFromListFile(rgbImagePaths, rgbTimeStamps, listFile, LEFT);
    StereoPathsFromListFile(stereoImagePaths, stereoTimeStamps, listFile, RIGHT);

    frameNumber = 0;
}

void PhoneSource::GrabNewFrame()
{
    std::string path =  rgbImagePaths[frameNumber];
    std::cout << path << std::endl;
    imLeft = cv::imread(path);

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

    long long count = 0;
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
            fullPath << listDir << folder << timestamp << "000000.png";

            imagePaths.push_back(fullPath.str());
            timeStamps.push_back(count++);
        }
    }
}
