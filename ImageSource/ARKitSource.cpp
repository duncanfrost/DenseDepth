#include "ARKitSource.h"
#include <iostream>
#include <sstream>
#include <fstream>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <iomanip>
#include <string>

ARKitSource::ARKitSource(const std::string &directory, const std::string& listFile)
{
    PathsFromListFile(rgbImagePaths, rgbTimeStamps, directory, listFile);
}

void ARKitSource::PathsFromListFile(std::vector<std::string> &imagePaths,
                                    std::vector<long long> &timeStamps,
                                    std::string directory, std::string file)
{
    std::string listPath = directory + file;

    std::cout << "List path: " << listPath << std::endl;

    imagePaths.clear();
    std::ifstream listFile(listPath.c_str());
    if (!listFile)
    {
        std::cout <<  "error: rgb image list file doesn't exist" << std::endl;
        exit(1);
    }
    std::string line;


    while (std::getline(listFile, line))
    {
        //Hashtags are considered comments
        if (line[0] == '#')
            continue;

        std::string timestampRaw;
        std::string path;

        std::istringstream iss(line);

        //Full paths are the root directory joined with whatever is in the listfile
        if (iss >> timestampRaw)
        {

            // std::cout << "Timestamp: " << std::setprecision(15) << timestampRaw << std::endl;
            // std::cout << "TimeStamp: " << timestampRaw  << std::endl;

            long long timestamp = std::stof(timestampRaw) * 1e6;
            // std::cout << "TimeStamp: " << std::setprecision(15) << timestamp  << std::endl;

            std::string imagePath;

            std::stringstream fullPath;
            fullPath << directory << "/Images/" << timestampRaw << ".png";
            // std::cout << fullPath.str() << std::endl;


            imagePaths.push_back(fullPath.str());
            timeStamps.push_back(timestamp);
        }
    }
}
