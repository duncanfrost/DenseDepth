#include "TUMFeatureSource.h"
#include <iostream>
#include <sstream>
#include <fstream>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <iomanip>

void TUMFeatureSource::GrabNewFrame()
{
    std::cout << "Getting here" << std::endl;
    exit(1);
}

TUMFeatureSource::TUMFeatureSource(const std::string& listFile)
{
    PathsFromListFile(rgbImagePaths, rgbTimeStamps, listFile);
}

void TUMFeatureSource::PathsFromListFile(std::vector<std::string> &imagePaths,
                                  std::vector<long long> &timeStamps,
                                  std::string listPath)
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

        double timestampRaw;
        std::string path;

        std::istringstream iss(line);

        //Full paths are the root directory joined with whatever is in the listfile
        if (iss >> timestampRaw)
        {

            long long timestamp = timestampRaw * 1e6;

            std::string imagePath;
            iss >> imagePath;

            
            std::stringstream fullPath;
            fullPath << listDir << imagePath;

            // std::cout << fullPath.str() << std::endl;

            imagePaths.push_back(fullPath.str());
            timeStamps.push_back(timestamp);
        }
    }
}
