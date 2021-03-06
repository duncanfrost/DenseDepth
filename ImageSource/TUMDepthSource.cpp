#include "TUMDepthSource.h"
#include <iostream>
#include <sstream>
#include <fstream>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <iomanip>

TUMDepthSource::TUMDepthSource(const std::string& depthListFile)
{
    PathsFromListFile(depthImagePaths, depthTimeStamps, depthListFile);
}

void TUMDepthSource::PathsFromListFile(std::vector<std::string> &imagePaths,
                                  std::vector<long long> &timeStamps,
                                  std::string listPath)
{
    imagePaths.clear();
    std::ifstream listFile(listPath.c_str());
    if (!listFile)
    {
        std::cout <<  "error: depth image list file doesn't exist" << std::endl;
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
